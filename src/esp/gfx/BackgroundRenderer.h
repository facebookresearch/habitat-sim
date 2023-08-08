// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BACKGROUND_RENDERER_H_
#define ESP_GFX_BACKGROUND_RENDERER_H_

#include "esp/core/configure.h"

#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER

#include <thread>

#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneGraph.h"
#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace gfx {
class BackgroundRenderer {
 public:
  explicit BackgroundRenderer(WindowlessContext* context);
  virtual ~BackgroundRenderer();

 protected:
  friend class Renderer;
  enum Task : unsigned int { Exit = 0, ReleaseContext = 1, Render = 2 };

  bool wasInitialized() const { return threadInitialized_; }

  void waitSceneGraph();
  void releaseContext();

  void ensureThreadInit();
  void startThreadJobs();
  void waitThreadJobs();

  void submitRenderJob(sensor::VisualSensor& sensor,
                       scene::SceneGraph& sceneGraph,
                       const Mn::MutableImageView2D& view,
                       RenderCamera::Flags flags);
  void startRenderJobs();

  // run loop for the thread.
  void runLoopThread();
  // thread* functions are ones that are called by the thread,
  int threadRender();
  void threadReleaseContext();

 private:
  WindowlessContext* context_;

  std::atomic<int> done_, sgLock_, start_;
  std::thread t_;
  bool threadIsWorking_, threadInitialized_;

  bool threadOwnsContext_;
  Task task_;
  std::vector<std::tuple<std::reference_wrapper<sensor::VisualSensor>,
                         std::reference_wrapper<scene::SceneGraph>,
                         std::reference_wrapper<const Mn::MutableImageView2D>,
                         RenderCamera::Flags>>
      jobs_;
  int jobsWaiting_ = 0;
};
}  // namespace gfx
}  // namespace esp

#endif  // ESP_BUILD_WITH_BACKGROUND_RENDERER

#endif  // ESP_GFX_BACKGROUND_RENDERER_H_
