// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <Corrade/Containers/StridedArrayView.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Platform/GLContext.h>

#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/magnum.h"
#include "esp/sensor/VisualSensor.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

struct BackgroundRenderThread {
  BackgroundRenderThread(WindowlessContext* context)
      : context_{context}, done_{0}, sgLock_{0}, barrierVal_{0} {
    context_->release();
    t = std::thread(&BackgroundRenderThread::run, this);

    jobsWaiting_ = 1;
    threadStarted_ = true;
    waitThread();
    context_->makeCurrent();
  }

  void startThread() {
    barrierVal_.fetch_xor(1, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);
    cv_.notify_all();
    threadStarted_ = true;
  }

  ~BackgroundRenderThread() {
    task_ = Task::Exit;
    startThread();
    t.join();
  }

  static void spinLock(const std::atomic<int>& lk, int val) {
    while (lk.load(std::memory_order_acquire) != val)
      asm volatile("pause" ::: "memory");
  }

  void waitThread() {
    CORRADE_INTERNAL_ASSERT(threadStarted_ || jobsWaiting_ == 0);
    spinLock(done_, jobsWaiting_);

    done_.store(0);
    jobsWaiting_ = 0;
    threadStarted_ = false;
  }

  void waitSG() { spinLock(sgLock_, 0); }

  void submitJob(sensor::VisualSensor& sensor,
                 scene::SceneGraph& sceneGraph,
                 const Mn::MutableImageView2D& view,
                 RenderCamera::Flags flags) {
    jobs_.emplace_back(std::ref(sensor), std::ref(sceneGraph), std::cref(view),
                       flags);
    jobsWaiting_ += 1;
  }

  void startJobs() {
    task_ = Task::Render;
    sgLock_.store(1, std::memory_order_relaxed);
    startThread();
  }

  void releaseContext() {
    task_ = Task::ReleaseContext;

    startThread();

    jobsWaiting_ = 1;
    waitThread();
  }

  int threadRender() {
    if (!threadOwnsContext_) {
      VLOG(1)
          << "BackgroundRenderThread:: Background thread acquired GL Context";
      context_->makeCurrentPlatform();
      Mn::GL::Context::makeCurrent(threadContext_);
      threadOwnsContext_ = true;
    }

    std::vector<std::vector<RenderCamera::DrawableTransforms>> jobTransforms(
        jobs_.size());

    for (int i = 0; i < jobs_.size(); ++i) {
      auto& job = jobs_[i];
      sensor::VisualSensor& sensor = std::get<0>(job);
      scene::SceneGraph& sg = std::get<1>(job);
      RenderCamera::Flags flags = std::get<3>(job);

      auto* camera = sensor.getRenderCamera();
      jobTransforms[i].reserve(sg.getDrawableGroups().size());
      for (auto& it : sg.getDrawableGroups()) {
        it.second.prepareForDraw(*camera);
        auto transforms = camera->drawableTransformations(it.second);
        camera->filterTransforms(transforms, flags);

        jobTransforms[i].emplace_back(std::move(transforms));
      }
    }

    sgLock_.store(0, std::memory_order_release);

    for (int i = 0; i < jobs_.size(); ++i) {
      auto& job = jobs_[i];
      sensor::VisualSensor& sensor = std::get<0>(job);
      RenderCamera::Flags flags = std::get<3>(job);

      if (!(flags & RenderCamera::Flag::ObjectsOnly))
        sensor.renderTarget().renderEnter();

      auto* camera = sensor.getRenderCamera();
      for (auto& transforms : jobTransforms[i]) {
        camera->draw(transforms, flags);
      }

      if (!(flags & RenderCamera::Flag::ObjectsOnly))
        sensor.renderTarget().renderExit();
    }

    for (auto& job : jobs_) {
      sensor::VisualSensor& sensor = std::get<0>(job);
      const Mn::MutableImageView2D& view = std::get<2>(job);
      RenderCamera::Flags flags = std::get<3>(job);
      if (flags & RenderCamera::Flag::ObjectsOnly)
        continue;

      auto sensorType = sensor.specification()->sensorType;
      if (sensorType == sensor::SensorType::Color)
        sensor.renderTarget().readFrameRgba(view);

      if (sensorType == sensor::SensorType::Depth)
        sensor.renderTarget().readFrameDepth(view);

      if (sensorType == sensor::SensorType::Semantic)
        sensor.renderTarget().readFrameObjectId(view);
    }

    int jobsDone = jobs_.size();
    jobs_.clear();
    return jobsDone;
  }

  void threadReleaseContext() {
    if (threadOwnsContext_) {
      Mn::GL::Context::makeCurrent(nullptr);
      context_->releasePlatform();
      threadOwnsContext_ = false;
    }
  }

  enum Task : unsigned int { Exit = 0, ReleaseContext = 1, Render = 2 };

  void run() {
    context_->makeCurrentPlatform();
    threadContext_ = new Mn::Platform::GLContext{Mn::NoCreate};
    if (!threadContext_->tryCreate())
      Mn::Fatal{} << "BackgroundRenderThread: Failed to create OpenGL context";

    Mn::GL::Context::makeCurrent(threadContext_);
    threadOwnsContext_ = true;

    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

    threadReleaseContext();
    done_.store(1, std::memory_order_release);

    int waitVal = 0;
    bool done = false;
    while (!done) {
      waitVal ^= 1;
      {
        std::unique_lock<std::mutex> lk{mutex_};
        cv_.wait(lk, [this, waitVal] {
          return barrierVal_.load(std::memory_order_acquire) == waitVal;
        });
      }

      switch (task_) {
        case Task::Exit:
          threadReleaseContext();
          delete threadContext_;
          done = true;
          break;
        case Task::ReleaseContext:
          threadReleaseContext();
          done_.store(1, std::memory_order_relaxed);
          break;
        case Task::Render:
          const int jobsDone = threadRender();
          done_.store(jobsDone, std::memory_order_relaxed);
          break;
      }

      std::atomic_thread_fence(std::memory_order_release);
    };
  }

  WindowlessContext* context_;

  std::atomic<int> done_, sgLock_, barrierVal_;
  std::condition_variable cv_;
  std::mutex mutex_;
  std::thread t;
  bool threadStarted_ = false;

  bool threadOwnsContext_;
  Mn::Platform::GLContext* threadContext_;
  Task task_;
  std::vector<std::tuple<std::reference_wrapper<sensor::VisualSensor>,
                         std::reference_wrapper<scene::SceneGraph>,
                         std::reference_wrapper<const Mn::MutableImageView2D>,
                         RenderCamera::Flags>>
      jobs_;
  int jobsWaiting_ = 0;
};

struct Renderer::Impl {
  explicit Impl(WindowlessContext* context, Flags flags)
      : context_{context}, depthShader_{nullptr}, flags_{flags} {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

    if (flags & Flag::BackgroundThread)
      backgroundRenderer_ = std::make_unique<BackgroundRenderThread>(context_);
  }

  ~Impl() {
    acquireGlContext();
    LOG(INFO) << "Deconstructing Renderer";
  }

  void draw(RenderCamera& camera,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags) {
    acquireGlContext();
    for (auto& it : sceneGraph.getDrawableGroups()) {
      // TODO: remove || true
      if (it.second.prepareForDraw(camera) || true) {
        camera.draw(it.second, flags);
      }
    }
  }

  void draw(sensor::VisualSensor& visualSensor,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags) {
    draw(*visualSensor.getRenderCamera(), sceneGraph, flags);
  }

  void drawAsync(sensor::VisualSensor& visualSensor,
                 scene::SceneGraph& sceneGraph,
                 const Mn::MutableImageView2D& view,
                 RenderCamera::Flags flags) {
    if (!backgroundRenderer_)
      Mn::Fatal{} << "Renderer was not created with a background render "
                     "thread, cannot do async drawing";
    if (contextIsOwned_) {
      context_->release();
      contextIsOwned_ = false;
    }
    backgroundRenderer_->submitJob(visualSensor, sceneGraph, view, flags);
  }

  void startDrawJobs() {
    if (backgroundRenderer_)
      backgroundRenderer_->startJobs();
  }

  void drawWait() {
    if (backgroundRenderer_)
      backgroundRenderer_->waitThread();
  }

  void waitSG() {
    if (backgroundRenderer_)
      backgroundRenderer_->waitSG();
  }

  void acquireGlContext() {
    if (!contextIsOwned_) {
      VLOG(1) << "Renderer:: Main thread acquired GL Context";
      backgroundRenderer_->releaseContext();
      context_->makeCurrent();
      contextIsOwned_ = true;
    }
  }

  void bindRenderTarget(sensor::VisualSensor& sensor) {
    acquireGlContext();
    auto depthUnprojection = sensor.depthUnprojection();
    if (!depthUnprojection) {
      throw std::runtime_error(
          "Sensor does not have a depthUnprojection matrix");
    }

    if (!depthShader_) {
      depthShader_ = std::make_unique<DepthShader>(
          DepthShader::Flag::UnprojectExistingDepth);
    }

    sensor.bindRenderTarget(RenderTarget::create_unique(
        sensor.framebufferSize(), *depthUnprojection, depthShader_.get(),
        flags_));
  }

 private:
  WindowlessContext* context_;
  bool contextIsOwned_ = true;
  std::unique_ptr<DepthShader> depthShader_ = nullptr;
  const Flags flags_;

  std::unique_ptr<BackgroundRenderThread> backgroundRenderer_ = nullptr;
};

Renderer::Renderer(WindowlessContext* context, Flags flags)
    : pimpl_(spimpl::make_unique_impl<Impl>(context, flags)) {}

void Renderer::draw(RenderCamera& camera,
                    scene::SceneGraph& sceneGraph,
                    RenderCamera::Flags flags) {
  pimpl_->draw(camera, sceneGraph, flags);
}

void Renderer::draw(sensor::VisualSensor& visualSensor,
                    scene::SceneGraph& sceneGraph,
                    RenderCamera::Flags flags) {
  pimpl_->draw(visualSensor, sceneGraph, flags);
}

void Renderer::drawAsync(sensor::VisualSensor& visualSensor,
                         scene::SceneGraph& sceneGraph,
                         const Mn::MutableImageView2D& view,
                         RenderCamera::Flags flags) {
  pimpl_->drawAsync(visualSensor, sceneGraph, view, flags);
}

void Renderer::drawWait() {
  pimpl_->drawWait();
}

void Renderer::waitSG() {
  pimpl_->waitSG();
}

void Renderer::startDrawJobs() {
  pimpl_->startDrawJobs();
}

void Renderer::acquireGlContext() {
  pimpl_->acquireGlContext();
}

void Renderer::bindRenderTarget(sensor::VisualSensor& sensor) {
  pimpl_->bindRenderTarget(sensor);
}

}  // namespace gfx
}  // namespace esp
