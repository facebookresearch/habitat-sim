// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "magnum.h"

#include "esp/core/esp.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

class RenderCamera : public MagnumCamera {
 public:
  RenderCamera(scene::SceneNode& node);
  RenderCamera(scene::SceneNode& node,
               const vec3f& eye,
               const vec3f& target,
               const vec3f& up);
  virtual ~RenderCamera() {
    // do nothing, let magnum handle the camera
  }

  // Get the scene node being attached to.
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(MagnumCamera::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(MagnumCamera::object());
  }

  RenderCamera& setProjectionMatrix(int width,
                                    int height,
                                    float znear,
                                    float zfar,
                                    float hfov);

  /**
   * @brief Overload function to render the drawables
   * @return the number of drawables that are drawn
   */
  uint32_t draw(MagnumDrawableGroup& drawables);
  /**
   * @brief Enable or disable the frustum culling test
   * @param true, enable; false, disable;
   */
  void setFrustumCullingEnabled(bool value) { frustumCullingEnabled_ = value; }
  /**
   * @brief Get the status, if the frustum culling is enabled
   */
  bool getFrustumCullingEnabled() { return frustumCullingEnabled_; }

  /**
   * @brief performs the frustum culling
   * @param drawableTransforms, a vector of pairs of Drawable3D object and its
   * absolute transformation
   * the number of drawables that are not culled
   *
   * NOTE: user are not encouraged to call this function directly.
   * The preferred way is to enable the frustum culling by calling @ref
   * setFrustumCullingEnabled and then call @ref draw
   */
  size_t cull(std::vector<
              std::pair<std::reference_wrapper<Magnum::SceneGraph::Drawable3D>,
                        Magnum::Matrix4>>& drawableTransforms);

 protected:
  bool frustumCullingEnabled_ = false;

  ESP_SMART_POINTERS(RenderCamera)
};

}  // namespace gfx
}  // namespace esp
