// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PBR_IRRADIANCEMAP_SHADER_H_
#define ESP_GFX_PBR_IRRADIANCEMAP_SHADER_H_

#include <initializer_list>

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Shaders/GenericGL.h>

#include "esp/core/esp.h"

namespace esp {

namespace gfx {

class PbrIrradianceMapShader : public Magnum::GL::AbstractShaderProgram {
 public:
  // ==== Attribute definitions ====
  /**
   * @brief vertex positions
   */
  typedef Magnum::Shaders::GenericGL3D::Position Position;

  /**
   * @brief normal direction
   */
  typedef Magnum::Shaders::GenericGL3D::Normal Normal;

  /**
   * @brief 2D texture coordinates
   *
   */
  typedef Magnum::Shaders::GenericGL3D::TextureCoordinates TextureCoordinates;

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
  enum class Flag : Magnum::UnsignedShort {};

  /**
   * @brief Flags
   */
  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief Constructor
   * @param flags         Flags
   */
  explicit PbrIrradianceMapShader(Flags flags = {});

  /** @brief Copying is not allowed */
  PbrIrradianceMapShader(const PbrIrradianceMapShader&) = delete;

  /** @brief Move constructor */
  PbrIrradianceMapShader(PbrIrradianceMapShader&&) noexcept = default;

  /** @brief Copying is not allowed */
  PbrIrradianceMapShader& operator=(const PbrIrradianceMapShader&) = delete;

  /** @brief Move assignment */
  PbrIrradianceMapShader& operator=(PbrIrradianceMapShader&&) noexcept =
      default;

  /** @brief Flags */
  Flags flags() const { return flags_; }

  // ======== texture binding ========
  /**
   * @brief Bind the BaseColor texture
   * @return Reference to self (for method chaining)
   */
  PbrIrradianceMapShader& bindBaseColorTexture(Magnum::GL::Texture2D& texture);

 protected:
  Flags flags_;

  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int modelviewMatrixUniform_ = ID_UNDEFINED;
  int projMatrixUniform_ = ID_UNDEFINED;
};

CORRADE_ENUMSET_OPERATORS(PbrIrradianceMapShader::Flags)

}  // namespace gfx
}  // namespace esp

#endif
