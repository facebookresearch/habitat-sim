// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_RENDERER_H_
#define ESP_GFX_RENDERER_H_

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
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
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;
  CORRADE_ENUMSET_FRIEND_OPERATORS(Flags)

  enum class RenderTargetBindingFlag {
    /**
     * When binding the render target to a depth or a sementic sensor,
     * setting this flag will give the render target the ability to visualize
     * the depth, or sementic info
     */
    VisualizeTexture = 1 << 0,
  };

  typedef Corrade::Containers::EnumSet<RenderTargetBindingFlag>
      RenderTargetBindingFlags;

  CORRADE_ENUMSET_FRIEND_OPERATORS(RenderTargetBindingFlags)

  /**
   * @brief Constructor
   */
  explicit Renderer(Flags flags = {});

  // draw the scene graph with the camera specified by user
  void draw(RenderCamera& camera,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling});

  // draw the scene graph with the visual sensor provided by user
  void draw(sensor::VisualSensor& visualSensor,
            scene::SceneGraph& sceneGraph,
            RenderCamera::Flags flags = {RenderCamera::Flag::FrustumCulling});

  /**
   * @brief visualize the observation of a non-rgb visual sensor, e.g., depth,
   * semantic
   */
  void visualize(sensor::VisualSensor& visualSensor,
                 float colorMapOffset = 1.0f / 512.0f,
                 float colorMapScale = 1.0f / 256.0f);

  /**
   * @brief Binds a @ref RenderTarget to the sensor
   * @param[in] sensor the target sensor
   * @param[in] bindingFlags flags, such as to control the bindings
   */
  void bindRenderTarget(sensor::VisualSensor& sensor,
                        RenderTargetBindingFlags bindingFlags = {});

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(Renderer)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_RENDERER_H_
