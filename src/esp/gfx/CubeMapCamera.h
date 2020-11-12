// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_CUBEMAPCAMERA_H_
#define ESP_GFX_CUBEMAPCAMERA_H_

#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Tags.h>
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {
class CubeMapCamera : public RenderCamera {
 public:
  /**
   * @brief Constructor
   * @param node, the scene node to which the camera is attached
   */
  explicit CubeMapCamera(scene::SceneNode& node);
  /**
   * @brief Constructor
   * @param node, the scene node to which the camera is attached
   * @param eye, the eye position (parent node space)
   * @param target, the target position (parent node space)
   * @param up, the up direction (parent node space)
   */
  explicit CubeMapCamera(scene::SceneNode& node,
                         const vec3f& eye,
                         const vec3f& target,
                         const vec3f& up);
  /**
   * @brief Constructor
   * @param node, the scene node to which the camera is attached
   * @param eye, the eye position (parent node space)
   * @param target, the target position (parent node space)
   * @param up, the up direction (parent node space)
   */
  explicit CubeMapCamera(scene::SceneNode& node,
                         const Magnum::Vector3& eye,
                         const Magnum::Vector3& target,
                         const Magnum::Vector3& up);
  /**
   * @brief Move the camera towards a specified cube face
   * @param cubeSide, the cube map coordinate, see the following pictures
   * NOTE: +Y is top
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   * @return Reference to self (for method chaining)
   */
  CubeMapCamera& switchToFace(Magnum::GL::CubeMapCoordinate cubeSide);

 protected:
  // viewing matrix (in parent node space) computed by Mn::Matrix4::lookAt(eye,
  // target, up)
  // this is exactly the matrix set to the node in the constructor
  // default value: identity matrix
  Magnum::Matrix4 originalViewingMatrix_ =
      Magnum::Matrix4{Magnum::Math::IdentityInit};
};
}  // namespace gfx
}  // namespace esp
#endif
