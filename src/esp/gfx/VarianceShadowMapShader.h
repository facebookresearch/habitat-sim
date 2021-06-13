// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_VARIANCE_SHADOWMAP_SHADER_H_
#define ESP_GFX_PBR_VARIANCE_SHADOWMAP_SHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Shaders/GenericGL.h>

namespace esp {

namespace gfx {

class VarianceShadowMapShader : public Magnum::GL::AbstractShaderProgram {
 public:
  // ==== Attribute definitions ====
  /**
   * @brief vertex positions
   */
  typedef Magnum::Shaders::GenericGL3D::Position Position;

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
  explicit VarianceShadowMapShader();

  /** @brief Copying is not allowed */
  VarianceShadowMapShader(const VarianceShadowMapShader&) = delete;

  /** @brief Move constructor */
  VarianceShadowMapShader(VarianceShadowMapShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  VarianceShadowMapShader& operator=(const VarianceShadowMapShader&) = delete;

  /** @brief Move assignment */
  VarianceShadowMapShader& operator=(VarianceShadowMapShader&&) noexcept =
      default;

  // ======== set uniforms ===========
  /**
   *  @brief Set projection matrix of the light to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  VarianceShadowMapShader& setLightProjectionMatrix(
      const Magnum::Matrix4& matrix);

  /**
   *  @brief Set modelview matrix of the light to the uniform on GPU
   *  NOTE: it converts a vertex from model space to light space
   *  @return Reference to self (for method chaining)
   */
  VarianceShadowMapShader& setLightModelViewMatrix(
      const Magnum::Matrix4& matrix);

 private:
  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int lightProjectionMatrixUniform_ = -1;
  int lightModelViewMatrixUniform_ = -1;
};

}  // namespace gfx
}  // namespace esp
#endif
