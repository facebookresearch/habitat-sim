// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_RENDERER_H_
#define ESP_GFX_RENDERER_H_

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/scene/SceneGraph.h"
#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace gfx {

class Renderer {
 public:
  enum class Flag {
    /**
     * No textures for the meshes.
     * Textures take a lot of GPU memory but they are not needed unless we are
     * doing RGB rendering.
     * Note: Cannot set this flag when doing RGB rendering
     */
    NoTextures = 1 << 0,
    BackgroundThread = 1 << 1
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  /**
   * @brief Constructor
   */
  explicit Renderer(Flags flags = {});

  /**
   * @brief Constructor for when creating a background thread
   */
  explicit Renderer(WindowlessContext* context, Flags flags = {});

  // draw the scene graph with the camera specified by user
  void draw(RenderCamera& camera,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling});

  // draw the scene graph with the visual sensor provided by user
  void draw(sensor::VisualSensor& visualSensor,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling});

#if !defined(CORRADE_TARGET_EMSCRIPTEN)
  // draw the scene graph with the visual sensor provided by user
  // async
  void drawAsync(sensor::VisualSensor& visualSensor,
                 scene::SceneGraph& sceneGraph,
                 const Mn::MutableImageView2D& view,
                 RenderCamera::Flags flags = {
                     RenderCamera::Flag::FrustumCulling});

  void drawWait();
  void waitSG();

  void startDrawJobs();
#endif

  void acquireGlContext();
  /**
   * @brief Binds a @ref RenderTarget to the sensor
   */
  void bindRenderTarget(sensor::VisualSensor& sensor);

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(Renderer)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_RENDERER_H_
