// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_EQUIRECTANGULAR_TO_CUBEMAP_SHADER_H_
#define ESP_GFX_PBR_EQUIRECTANGULAR_TO_CUBEMAP_SHADER_H_

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/Shaders/GenericGL.h>

#include <string>

namespace esp {
namespace gfx {
/**
 * @brief a shader to convert a HDRi image (the environment map in
 * equirectangular form) to a cubemap
 */
class PbrEquiRectangularToCubeMapShader
    : public Magnum::GL::AbstractShaderProgram {
 public:
  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,
  };

  /**
   * @brief Constructor
   */
  explicit PbrEquiRectangularToCubeMapShader();

  /** @brief Copying is not allowed */
  PbrEquiRectangularToCubeMapShader(const PbrEquiRectangularToCubeMapShader&) =
      delete;

  /** @brief Move constructor */
  PbrEquiRectangularToCubeMapShader(
      PbrEquiRectangularToCubeMapShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PbrEquiRectangularToCubeMapShader& operator=(
      const PbrEquiRectangularToCubeMapShader&) = delete;

  /** @brief Move assignment */
  PbrEquiRectangularToCubeMapShader& operator=(
      PbrEquiRectangularToCubeMapShader&&) noexcept = default;

  void loadTexture(const std::string& filename);

  // ======== set uniforms ===========

  /**
   * @brief set the index of the cube side.
   * @param[in] index, can be 0, 1, 2, 3, 4 or 5
   * @return Reference to self (for method chaining)
   */
  PbrEquiRectangularToCubeMapShader& setCubeSideIndex(unsigned int index);

  /**
   * @brief Bind equirectangular texture.
   * @return Reference to self (for method chaining)
   */
  PbrEquiRectangularToCubeMapShader& bindEquirectangularTexture(
      Magnum::GL::Texture2D& texture);

 private:
  GLint cubeSideIndexUniform_ = -1;
};

}  // namespace gfx
}  // namespace esp

#endif
