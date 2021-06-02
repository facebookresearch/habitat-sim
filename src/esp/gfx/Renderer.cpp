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
#include <Magnum/GL/BufferImage.h>
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
#include <Magnum/ResourceManager.h>

#include "esp/core/Check.h"
#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/TextureVisualizerShader.h"
#include "esp/gfx/magnum.h"
#include "esp/sensor/VisualSensor.h"
#include "esp/sim/Simulator.h"

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
#include <atomic_wait.h>
#endif

// There is a depth buffer overriden even when the depth test and depth buffer
// writing is diabled. It was observed only on Mac OSX, not on linux. Suspect it
// is a bug in the GL driver on Mac.
#ifdef CORRADE_TARGET_APPLE
#define ENABLE_VISUALIZATION_WORKAROUND_ON_MAC
#endif

namespace Mn = Magnum;

namespace esp {
namespace gfx {

#if !defined(CORRADE_TARGET_EMSCRIPTEN)

struct BackgroundRenderThread {
  explicit BackgroundRenderThread(WindowlessContext* context)
      : context_{context}, done_{0}, sgLock_{0}, start_{0} {
    context_->release();
    t = std::thread(&BackgroundRenderThread::run, this);

    jobsWaiting_ = 1;
    threadStarted_ = true;
    waitThread();
    context_->makeCurrent();
  }

  void startThread() {
    std::atomic_thread_fence(std::memory_order_release);
    start_.fetch_xor(1, std::memory_order_release);
    std::atomic_notify_all(&start_);

    threadStarted_ = true;
  }

  ~BackgroundRenderThread() {
    task_ = Task::Exit;
    startThread();
    t.join();
  }

  void waitThread() {
    CORRADE_INTERNAL_ASSERT(threadStarted_ || jobsWaiting_ == 0);
    if (jobsWaiting_ != 0) {
      std::atomic_wait_explicit(&done_, 0, std::memory_order_acquire);

      CORRADE_INTERNAL_ASSERT(done_.load(std::memory_order_relaxed) ==
                              jobsWaiting_);
    }

    done_.store(0);
    jobsWaiting_ = 0;
    threadStarted_ = false;
  }

  void waitSG() {
    std::atomic_wait_explicit(&sgLock_, 1, std::memory_order_acquire);
  }

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
    std::atomic_notify_all(&sgLock_);

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
    std::atomic_notify_all(&done_);

    int oldStartVal = 0;
    bool done = false;
    while (!done) {
      std::atomic_wait_explicit(&start_, oldStartVal,
                                std::memory_order_acquire);

      oldStartVal ^= 1;

      switch (task_) {
        case Task::Exit:
          threadReleaseContext();
          delete threadContext_;
          done = true;
          break;
        case Task::ReleaseContext:
          threadReleaseContext();
          done_.store(1, std::memory_order_acq_rel);
          break;
        case Task::Render:
          const int jobsDone = threadRender();
          done_.store(jobsDone, std::memory_order_acq_rel);
          break;
      }

      std::atomic_thread_fence(std::memory_order_release);
      std::atomic_notify_all(&done_);
    };
  }

  WindowlessContext* context_;

  std::atomic<int> done_, sgLock_, start_;
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
#endif

struct Renderer::Impl {
  explicit Impl(WindowlessContext* context, Flags flags)
      : context_{context},
        depthShader_{nullptr},
        flags_{flags},
        mesh_{Cr::Containers::NullOpt} {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
    if (flags & Flag::BackgroundThread) {
      CORRADE_INTERNAL_ASSERT(context_ != nullptr);
      backgroundRenderer_ = std::make_unique<BackgroundRenderThread>(context_);
    }
#endif
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
      // TODO: remove || true and NOLINT below
      // NOLINTNEXTLINE (readability-simplify-boolean-expr)
      if (it.second.prepareForDraw(camera) || true) {
        camera.draw(it.second, flags);
      }
    }
  }

  void draw(sensor::VisualSensor& visualSensor, sim::Simulator& sim) {
    if (visualSensor.specification()->sensorType ==
        sensor::SensorType::Semantic) {
      ESP_CHECK(sim.semanticSceneExists(),
                "Renderer::Impl::draw(): SemanticSensor observation requested "
                "but no SemanticScene is loaded");
    }
    visualSensor.drawObservation(sim);
  }

  void visualize(sensor::VisualSensor& visualSensor,
                 float colorMapOffset,
                 float colorMapScale) {
    sensor::SensorType& type = visualSensor.specification()->sensorType;
    if (type == sensor::SensorType::Depth ||
        type == sensor::SensorType::Semantic) {
      Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::DepthTest);
      gfx::RenderTarget& tgt = visualSensor.renderTarget();
      if (!mesh_) {
        // prepare a big triangle mesh to cover the screen
        mesh_ = Mn::GL::Mesh{};
        mesh_->setCount(3);
      }
      esp::gfx::Renderer::Impl::RendererShaderType rendererShaderType =
          esp::gfx::Renderer::Impl::RendererShaderType::DepthTextureVisualizer;

      if (type == sensor::SensorType::Semantic) {
        rendererShaderType =
            gfx::Renderer::Impl::RendererShaderType::ObjectIdTextureVisualizer;
      }

      Magnum::Resource<Mn::GL::AbstractShaderProgram, TextureVisualizerShader>
          shader = getShader<TextureVisualizerShader>(rendererShaderType);

      if (type == sensor::SensorType::Depth) {
#ifdef ENABLE_VISUALIZATION_WORKAROUND_ON_MAC
        // create a BufferImage instance, if not already
        if (!depthBufferImage_) {
          depthBufferImage_.emplace(Mn::GL::PixelFormat::DepthComponent,
                                    Mn::GL::PixelType::Float);
        }
        tgt.getDepthTexture().image(0, *depthBufferImage_,
                                    Mn::GL::BufferUsage::StaticRead);

        // This takes the above output image (which is depth) and
        // "reinterprets" it as R32F. In other words, the image below serves
        // as an "image view".
        Mn::GL::BufferImage2D clonedDepthImage{
            depthBufferImage_->storage(), Mn::PixelFormat::R32F,
            depthBufferImage_->size(),
            Mn::GL::Buffer::wrap(depthBufferImage_->buffer().id(),
                                 Mn::GL::ObjectFlag::Created),
            depthBufferImage_->dataSize()};

        // setup a texture
        if (!visualizedTex_ ||
            visualizedTex_->imageSize(0) != tgt.framebufferSize()) {
          visualizedTex_ = Mn::GL::Texture2D{};
          (*visualizedTex_)
              .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
              .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
              .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
              .setStorage(1, Mn::GL::TextureFormat::R32F,
                          tgt.framebufferSize());
        }
        (*visualizedTex_).setSubImage(0, {}, clonedDepthImage);
        shader->bindDepthTexture(*visualizedTex_);
#else
        shader->bindDepthTexture(tgt.getDepthTexture());
#endif
        shader->setDepthUnprojection(*visualSensor.depthUnprojection());
      } else if (type == sensor::SensorType::Semantic) {
        shader->bindObjectIdTexture(tgt.getObjectIdTexture());
      }
      shader->setColorMapTransformation(colorMapOffset, colorMapScale);
      tgt.renderReEnter();
      shader->draw(*mesh_);
      tgt.renderExit();

      // TODO object id
      Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::DepthTest);
    }
  }

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
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
#else
  void acquireGlContext(){};
#endif

  void bindRenderTarget(sensor::VisualSensor& sensor, Flags bindingFlags) {
    acquireGlContext();
    auto depthUnprojection = sensor.depthUnprojection();
    CORRADE_ASSERT(depthUnprojection,
                   "Renderer::Impl::bindRenderTarget(): Sensor does not have a "
                   "depthUnprojection matrix", );

    if (!depthShader_) {
      depthShader_ = std::make_unique<DepthShader>(
          DepthShader::Flag::UnprojectExistingDepth);
    }

    RenderTarget::Flags renderTargetFlags = {};
    switch (sensor.specification()->sensorType) {
      case sensor::SensorType::Color:
        CORRADE_ASSERT(
            !(flags_ & Flag::NoTextures),
            "Renderer::Impl::bindRenderTarget(): Tried to setup a color "
            "render buffer while the simulator was initialized with "
            "requiresTextures = false", );
        renderTargetFlags |= RenderTarget::Flag::RgbaAttachment;
        break;

      case sensor::SensorType::Depth:
        if (bindingFlags & Flag::VisualizeTexture) {
          renderTargetFlags |= RenderTarget::Flag::RgbaAttachment;
        }
        renderTargetFlags |= RenderTarget::Flag::DepthTextureAttachment;
        break;

      case sensor::SensorType::Semantic:
        if (bindingFlags & Flag::VisualizeTexture) {
          renderTargetFlags |= RenderTarget::Flag::RgbaAttachment;
        }
        renderTargetFlags |= RenderTarget::Flag::ObjectIdAttachment;
        break;

      default:
        // I need this default, since sensor type list is long, and without
        // default clang-tidy will complain
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }

    sensor.bindRenderTarget(RenderTarget::create_unique(
        sensor.framebufferSize(), *depthUnprojection, depthShader_.get(),
        renderTargetFlags, &sensor));
  }

 private:
  WindowlessContext* context_;
  bool contextIsOwned_ = true;
  // TODO: shall we use shader resource manager from now?
  std::unique_ptr<DepthShader> depthShader_;
  const Flags flags_;
#if !defined(CORRADE_TARGET_EMSCRIPTEN)
  std::unique_ptr<BackgroundRenderThread> backgroundRenderer_ = nullptr;
#endif
  Cr::Containers::Optional<Mn::GL::Mesh> mesh_;
  Mn::ResourceManager<Mn::GL::AbstractShaderProgram> shaderManager_;
#ifdef ENABLE_VISUALIZATION_WORKAROUND_ON_MAC
  Cr::Containers::Optional<Mn::GL::Texture2D> visualizedTex_;
  Cr::Containers::Optional<Mn::GL::BufferImage2D> depthBufferImage_;
#endif

  enum class RendererShaderType : uint8_t {
    DepthShader = 0,
    DepthTextureVisualizer = 1,
    ObjectIdTextureVisualizer = 2,
  };
  template <typename T>
  Mn::Resource<Mn::GL::AbstractShaderProgram, T> getShader(
      RendererShaderType type) {
    Mn::ResourceKey key;
    switch (type) {
      case RendererShaderType::DepthShader:
        key = Mn::ResourceKey{"depthShader"};
        break;

      case RendererShaderType::DepthTextureVisualizer:
        key = Mn::ResourceKey{"depthVisualizer"};
        break;

      case RendererShaderType::ObjectIdTextureVisualizer:
        key = Mn::ResourceKey{"objectIdVisualizer"};
        break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
    Mn::Resource<Mn::GL::AbstractShaderProgram, T> shader =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, T>(key);

    if (!shader) {
      if (type == RendererShaderType::DepthShader) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(),
            new DepthShader{DepthShader::Flag::UnprojectExistingDepth},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      } else if (type == RendererShaderType::DepthTextureVisualizer) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(),
            new TextureVisualizerShader{
                {TextureVisualizerShader::Flag::DepthTexture}},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      } else if (type == RendererShaderType::ObjectIdTextureVisualizer) {
        shaderManager_.set<Mn::GL::AbstractShaderProgram>(
            shader.key(),
            new TextureVisualizerShader{
                {TextureVisualizerShader::Flag::ObjectIdTexture}},
            Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
      }
    }
    CORRADE_INTERNAL_ASSERT(shader);

    return shader;
  }
};

Renderer::Renderer(Flags flags) : Renderer{nullptr, flags} {}

Renderer::Renderer(WindowlessContext* context, Flags flags)
    : pimpl_(spimpl::make_unique_impl<Impl>(context, flags)) {}

void Renderer::draw(RenderCamera& camera,
                    scene::SceneGraph& sceneGraph,
                    RenderCamera::Flags flags) {
  pimpl_->draw(camera, sceneGraph, flags);
}

void Renderer::draw(sensor::VisualSensor& visualSensor, sim::Simulator& sim) {
  pimpl_->draw(visualSensor, sim);
}

void Renderer::bindRenderTarget(sensor::VisualSensor& sensor,
                                Flags bindingFlags) {
  pimpl_->bindRenderTarget(sensor, bindingFlags);
}

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
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
#endif

void Renderer::acquireGlContext() {
  pimpl_->acquireGlContext();
}

void Renderer::visualize(sensor::VisualSensor& sensor,
                         float colorMapOffset,
                         float colorMapScale) {
  pimpl_->visualize(sensor, colorMapOffset, colorMapScale);
}

}  // namespace gfx
}  // namespace esp
