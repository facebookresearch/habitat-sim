// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_DEPTHMAP_SHADER_H_
#define ESP_GFX_PBR_DEPTHMAP_SHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Shaders/GenericGL.h>

namespace esp {

namespace gfx {

class DepthMapShader : public Magnum::GL::AbstractShaderProgram {
 public:
  // ==== Attribute definitions ====
  /**
   * @brief vertex positions
   */
  typedef Magnum::Shaders::GenericGL3D::Position Position;

  /**
   * @brief Constructor
   */
  explicit DepthMapShader();

  /** @brief Copying is not allowed */
  DepthMapShader(const DepthMapShader&) = delete;

  /** @brief Move constructor */
  DepthMapShader(DepthMapShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  DepthMapShader& operator=(const DepthMapShader&) = delete;

  /** @brief Move assignment */
  DepthMapShader& operator=(DepthMapShader&&) noexcept = default;

  // ======== set uniforms ===========
  /**
   *  @brief Set projection matrix of the light to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  DepthMapShader& setLightProjectionMatrix(const Magnum::Matrix4& matrix);

  /**
   *  @brief Set modelview matrix of the light to the uniform on GPU
   *  NOTE: it converts a vertex from model space to light space
   *  @return Reference to self (for method chaining)
   */
  DepthMapShader& setLightModelViewMatrix(const Magnum::Matrix4& matrix);

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
