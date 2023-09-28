// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BackgroundRenderer.h"
#include "RenderTarget.h"
#include "Renderer.h"

#include <atomic_wait.h>
#include <atomic>
#include <thread>

#include "esp/core/Check.h"
#include "esp/sensor/VisualSensor.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

BackgroundRenderer::BackgroundRenderer(WindowlessContext* context)
    : context_{context},
      done_{0},
      sgLock_{0},
      start_{0},
      threadIsWorking_{false},
      threadInitialized_{false} {}

void BackgroundRenderer::ensureThreadInit() {
  if (!wasInitialized()) {
    t_ = std::thread(&BackgroundRenderer::runLoopThread, this);

    jobsWaiting_ = 1;
    threadIsWorking_ = true;
    threadInitialized_ = true;
    waitThreadJobs();
  }
}

BackgroundRenderer::~BackgroundRenderer() {
  if (wasInitialized()) {
    task_ = Task::Exit;
    startThreadJobs();
    t_.join();
  }
}

void BackgroundRenderer::startThreadJobs() {
  std::atomic_thread_fence(std::memory_order_release);
  start_.fetch_xor(1, std::memory_order_release);
  cpp20::atomic_notify_all(&start_);

  threadIsWorking_ = true;
}

void BackgroundRenderer::waitThreadJobs() {
  CORRADE_INTERNAL_ASSERT(threadIsWorking_ || jobsWaiting_ == 0);
  if (jobsWaiting_ != 0) {
    cpp20::atomic_wait_explicit(&done_, 0, std::memory_order_acquire);

    CORRADE_INTERNAL_ASSERT(done_.load(std::memory_order_relaxed) ==
                            jobsWaiting_);
  }

  done_.store(0);
  jobsWaiting_ = 0;
  threadIsWorking_ = false;
}

void BackgroundRenderer::waitSceneGraph() {
  if (threadIsWorking_)
    cpp20::atomic_wait_explicit(&sgLock_, 1, std::memory_order_acquire);
}

void BackgroundRenderer::submitRenderJob(sensor::VisualSensor& sensor,
                                         scene::SceneGraph& sceneGraph,
                                         const Mn::MutableImageView2D& view,
                                         RenderCamera::Flags flags) {
  ESP_CHECK(
      sensor.specification()->sensorSubType == sensor::SensorSubType::Pinhole ||
          sensor.specification()->sensorSubType ==
              sensor::SensorSubType::Orthographic,
      "BackgroundRenderer:: Only Pinhole and Orthographic sensors are "
      "supported");
  jobs_.emplace_back(std::ref(sensor), std::ref(sceneGraph), std::cref(view),
                     flags);
}

void BackgroundRenderer::startRenderJobs() {
  waitThreadJobs();
  ensureThreadInit();
  task_ = Task::Render;
  sgLock_.store(1, std::memory_order_relaxed);
  jobsWaiting_ = jobs_.size();
  startThreadJobs();
}

void BackgroundRenderer::releaseContext() {
  if (!wasInitialized())
    return;
  waitThreadJobs();

  task_ = Task::ReleaseContext;

  startThreadJobs();

  jobsWaiting_ = 1;
  waitThreadJobs();
}

int BackgroundRenderer::threadRender() {
  if (!threadOwnsContext_) {
    ESP_VERY_VERBOSE() << "Background thread acquired GL Context";
    context_->makeCurrent();
    threadOwnsContext_ = true;
  }

  std::vector<std::vector<RenderCamera::DrawableTransforms>> jobTransforms(
      jobs_.size());

  for (size_t i = 0; i < jobs_.size(); ++i) {
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
  cpp20::atomic_notify_all(&sgLock_);

  for (size_t i = 0; i < jobs_.size(); ++i) {
    auto& job = jobs_[i];
    sensor::VisualSensor& sensor = std::get<0>(job);
    RenderCamera::Flags flags = std::get<3>(job);

    if (!(flags & RenderCamera::Flag::ObjectsOnly))
      sensor.renderTarget().renderEnter();

    auto* camera = sensor.getRenderCamera();

    for (auto& transforms : jobTransforms[i]) {
      camera->draw(transforms, flags);
    }
    auto sensorType = sensor.specification()->sensorType;
    if (sensorType == sensor::SensorType::Color) {
      sensor.renderTarget().tryDrawHbao();
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

void BackgroundRenderer::threadReleaseContext() {
  if (threadOwnsContext_) {
    context_->release();
    threadOwnsContext_ = false;
  }
}

void BackgroundRenderer::runLoopThread() {
  context_->makeCurrent();

  threadOwnsContext_ = true;

  threadReleaseContext();
  done_.store(1, std::memory_order_release);
  cpp20::atomic_notify_all(&done_);

  int oldStartVal = 0;
  bool done = false;
  while (!done) {
    cpp20::atomic_wait_explicit(&start_, oldStartVal,
                                std::memory_order_acquire);

    oldStartVal ^= 1;

    switch (task_) {
      case Task::Exit:
        threadReleaseContext();
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
    cpp20::atomic_notify_all(&done_);
  };
}

}  // namespace gfx
}  // namespace esp
