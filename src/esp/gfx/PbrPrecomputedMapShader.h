// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_PRECOMPUTEDMAP_SHADER_H_
#define ESP_GFX_PBR_PRECOMPUTEDMAP_SHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Shaders/GenericGL.h>

namespace esp {

namespace gfx {

/**
 * @brief A shader to output the irradiance map, applied in the IBL diffuse
 * part, or the prefiltered environment map, applied in the IBL specular part,
 * based on the user setting.
 */

class PbrPrecomputedMapShader : public Magnum::GL::AbstractShaderProgram {
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
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * This shader is set to compute the irradiance map
     */
    IrradianceMap = 1 << 0,

    /**
     * This shader is set to compute the prefiltered environment map
     */
    PrefilteredMap = 1 << 1,
  };

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief Constructor
   */
  explicit PbrPrecomputedMapShader(Flags flags);

  /** @brief Copying is not allowed */
  PbrPrecomputedMapShader(const PbrPrecomputedMapShader&) = delete;

  /** @brief Move constructor */
  PbrPrecomputedMapShader(PbrPrecomputedMapShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PbrPrecomputedMapShader& operator=(const PbrPrecomputedMapShader&) = delete;

  /** @brief Move assignment */
  PbrPrecomputedMapShader& operator=(PbrPrecomputedMapShader&&) noexcept =
      default;

  // ======== set uniforms ===========
  /**
   *  @brief Set "projection" matrix to the uniform on GPU
   *  @return Reference to self (for method chaining)
   */
  PbrPrecomputedMapShader& setProjectionMatrix(const Magnum::Matrix4& matrix);

  /**
   *  @brief Set modelview matrix to the uniform on GPU
   *         modelview = view * model
   *  @return Reference to self (for method chaining)
   */
  PbrPrecomputedMapShader& setTransformationMatrix(
      const Magnum::Matrix4& matrix);

  /**
   * @breif Set roughness to the uniform on GPU
   * NOTE: Flag::PrefilteredMap MUST be enabled
   *  @return Reference to self (for method chaining)
   */
  PbrPrecomputedMapShader& setRoughness(float roughness);

  // ======== texture binding ========
  /**
   * @brief Bind the environment map cubemap texture
   * @return Reference to self (for method chaining)
   */
  PbrPrecomputedMapShader& bindEnvironmentMap(
      Magnum::GL::CubeMapTexture& texture);

 protected:
  Flags flags_;
  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int modelviewMatrixUniform_ = -1;
  int projMatrixUniform_ = -1;
  int roughnessUniform_ = -1;
};

CORRADE_ENUMSET_OPERATORS(PbrPrecomputedMapShader::Flags)

}  // namespace gfx
}  // namespace esp

#endif
