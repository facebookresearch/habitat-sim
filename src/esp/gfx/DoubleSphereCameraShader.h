// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DOUBLESPHERECAMERASHADER_H_
#define ESP_GFX_DOUBLESPHERECAMERASHADER_H_

#include <Corrade/Containers/EnumSet.h>

#include "CubeMapShaderBase.h"
#include "esp/core/Esp.h"

namespace esp {
namespace gfx {
class DoubleSphereCameraShader : public CubeMapShaderBase {
 public:
  explicit DoubleSphereCameraShader(CubeMapShaderBase::Flags flags = {
                                        CubeMapShaderBase::Flag::ColorTexture});

  /**
   *  @brief Set the focal length of the fisheye camera
   *  @param focalLength the focal length x, y directions
   *  @return Reference to self (for method chaining)
   */
  DoubleSphereCameraShader& setFocalLength(Magnum::Vector2 focalLength);
  /**
   *  @brief Set the offset of the principal point of the fisheye camera
   *  @param offset the offset of the principal point in pixels
   *  @return Reference to self (for method chaining)
   */
  DoubleSphereCameraShader& setPrincipalPointOffset(Magnum::Vector2 offset);
  /**
   * @brief Set the alpha value in the "Double Sphere Camera" model.
   * See details in:
   * Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
   * Camera Model, The International Conference on 3D Vision (3DV), 2018
   *  @param alpha the alpha value (0.0 <= alpha < 1.0)
   *  @return Reference to self (for method chaining)
   */
  DoubleSphereCameraShader& setAlpha(float alpha);

  /**
   * @brief Set the Xi value in the "Double Sphere Camera" model.
   * See details in:
   * Vladyslav Usenko, Nikolaus Demmel and Daniel Cremers: The Double Sphere
   * Camera Model, The International Conference on 3D Vision (3DV), 2018
   * @param xi the Xi value
   * @return Reference to self (for method chaining)
   */
  DoubleSphereCameraShader& setXi(float xi);

  /**
   * @brief deconstructor
   */
  ~DoubleSphereCameraShader() override = default;

 protected:
  int focalLengthUniform_ = ID_UNDEFINED;
  int principalPointOffsetUniform_ = ID_UNDEFINED;
  int alphaUniform_ = ID_UNDEFINED;
  int xiUniform_ = ID_UNDEFINED;
};

}  // namespace gfx
}  // namespace esp
#endif
