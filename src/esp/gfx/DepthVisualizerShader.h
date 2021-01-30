// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DEPTHVISUALIZERSHADER_H_
#define ESP_GFX_DEPTHVISUALIZERSHADER_H_
#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include "esp/core/esp.h"

namespace esp {
namespace gfx {

/**
@brief A shader to visualize the depth buffer information
*/
class DepthVisualizerShader : public Magnum::GL::AbstractShaderProgram {
 public:
  /** @brief Constructor */
  explicit DepthVisualizerShader();

  /**
   * @brief Bind depth texture
   * @return Reference to self (for method chaining)
   */
  DepthVisualizerShader& bindDepthTexture(Magnum::GL::Texture2D& texture);

  /**
   * @brief Set the depth unprojection parameters directly
   * @return Reference to self (for method chaining)
   */
  DepthVisualizerShader& setDepthUnprojection(
      const Magnum::Vector2& depthUnprojection);

  /**
   * @brief Set the far parameters directly
   * @return Reference to self (for method chaining)
   */
  DepthVisualizerShader& setFar(float far);

 protected:
  GLint depthUnprojectionUniform_ = ID_UNDEFINED;
  GLint farUniform_ = ID_UNDEFINED;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_DEPTHVISUALIZERSHADER_H_
