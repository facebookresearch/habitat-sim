// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_TEXTUREVISUALIZERSHADER_H_
#define ESP_GFX_TEXTUREVISUALIZERSHADER_H_
#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include "esp/core/esp.h"

namespace esp {
namespace gfx {

/**
@brief A shader to visualize the depth buffer information
*/
class TextureVisualizerShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /** @brief Constructor */
  explicit TextureVisualizerShader();

  /**
   * @brief Bind depth texture
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& bindDepthTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Set the depth unprojection parameters directly
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& setDepthUnprojection(
      const Magnum::Vector2& depthUnprojection);

  /**
   * @brief Set the depth scaling to adjust overall intensity of the final
   * visual result Note: the value should less than or equal to the far plane
   * @return Reference to self (for method chaining)
   */
  TextureVisualizerShader& setDepthScaling(float depthScaling);

 protected:
  GLint depthUnprojectionUniform_ = ID_UNDEFINED;
  GLint depthScalingUniform_ = ID_UNDEFINED;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_TEXTUREVISUALIZERSHADER_H_
