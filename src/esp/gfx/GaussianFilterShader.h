// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_GAUSSIANFILTERSHADER_H_
#define ESP_GFX_GAUSSIANFILTERSHADER_H_
#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Shaders/GenericGL.h>

namespace esp {
namespace gfx {
/**
@brief A shader to visualize the depth buffer information
*/
class GaussianFilterShader : public Magnum::GL::AbstractShaderProgram {
 public:
  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,
  };

  /** @brief Constructor */
  explicit GaussianFilterShader();

  /**
   * @brief Bind texture.
   * @return Reference to self (for method chaining)
   */
  GaussianFilterShader& bindTexture(Magnum::GL::Texture2D& texture);

  enum class FilteringDirection {
    Horizontal = 0,
    Vertical = 1,
  };
  GaussianFilterShader& setFilteringDirection(FilteringDirection dir);

 private:
  GLint filterDirectionUniform_ = -1;
};

}  // namespace gfx
}  // namespace esp

#endif
