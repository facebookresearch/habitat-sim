// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_CUBEMAPCAMERA_H_
#define ESP_GFX_CUBEMAPCAMERA_H_

#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Tags.h>
#include "esp/core/Esp.h"
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {
class CubeMapCamera : public RenderCamera {
 public:
  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   */
  explicit CubeMapCamera(scene::SceneNode& node);
  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   * @param eye the eye position (parent node space)
   * @param target the target position (parent node space)
   * @param up the up direction (parent node space)
   */
  explicit CubeMapCamera(scene::SceneNode& node,
                         const vec3f& eye,
                         const vec3f& target,
                         const vec3f& up);
  /**
   * @brief Constructor
   * @param node the scene node to which the camera is attached
   * @param eye the eye position (parent node space)
   * @param target the target position (parent node space)
   * @param up the up direction (parent node space)
   */
  explicit CubeMapCamera(scene::SceneNode& node,
                         const Magnum::Vector3& eye,
                         const Magnum::Vector3& target,
                         const Magnum::Vector3& up);
  ~CubeMapCamera() override = default;

  static Magnum::GL::CubeMapCoordinate cubeMapCoordinate(
      unsigned int cubeSideIndex) {
    return Mn::GL::CubeMapCoordinate(int(Mn::GL::CubeMapCoordinate::PositiveX) +
                                     cubeSideIndex);
  }
  /**
   * @brief Move the camera towards a specified cube face
   * ```
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   * ```
   * @param cubeSide the cube map coordinate, see the following pictures
   * NOTE: +Y is top
   * CAREFUL! the local transformation of the camera node will be set after
   * calling this function.
   * @return Reference to self (for method chaining)
   */
  CubeMapCamera& switchToFace(Magnum::GL::CubeMapCoordinate cubeSide);
  /**
   * @brief Overload, move the camera towards a specified cube face
   * ```
   *           +----+
   *           | -Y |
   * +----+----+----+----+
   * | -Z | -X | +Z | +X |
   * +----+----+----+----+
   *           | +Y |
   *           +----+
   * ```
   * @param cubSideIndex the index of the cube map coordinate.
   * 0: +X
   * 1: -X
   * 2: +Y
   * 3: -Y
   * 4: +Z
   * 5: -Z
   * @return Reference to self (for method chaining)
   */
  CubeMapCamera& switchToFace(unsigned int cubeSideIndex);

  /**
   * Calling the the setProjectionMatrix from the base class is not allowed.
   * Use the new one instead.
   */
  RenderCamera& setProjectionMatrix(int width,
                                    int height,
                                    float znear,
                                    float zfar,
                                    float hfov) = delete;
  /**
   * @brief Set the projection matrix
   * @param width the width of the square image plane
   * @param znear the near clipping plane
   * @param znear the far clipping plane
   */
  CubeMapCamera& setProjectionMatrix(int width, float znear, float zfar);

  /**
   * @brief Update the original viewing matrix. It MUST be called after each
   * time the local transformation of camera node has been set.
   * @return Reference to self (for method chaining)
   */
  CubeMapCamera& updateOriginalViewingMatrix();

  /**
   * @brief restore the local transformation of the camera node using the stored
   * original viewing matrix.
   * It is useful, since @ref switchToFace() will change the local
   * transformations and user want to revert such changes.
   * @return Reference to self (for method chaining)
   */
  CubeMapCamera& restoreTransformation();

  /**
   * @brief return the camera local transformation matrix when it is about to
   * render a specific cube face.
   */
  static Magnum::Matrix4 getCameraLocalTransform(
      Mn::GL::CubeMapCoordinate cubeSideIndex);

 protected:
  // viewing matrix (in parent node space) computed by
  // Mn::Matrix4::lookAt(eye, target, up) this is exactly the matrix set to
  // the node in the constructor default value: identity matrix
  Magnum::Matrix4 originalViewingMatrix_ =
      Magnum::Matrix4{Magnum::Math::IdentityInit};
  ESP_SMART_POINTERS(CubeMapCamera)
};
}  // namespace gfx
}  // namespace esp
#endif
