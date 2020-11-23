// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DOUBLESPHERECAMERASHADER_H_
#define ESP_GFX_DOUBLESPHERECAMERASHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Shaders/Generic.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {
class DoubleSphereCameraShader : public Magnum::GL::AbstractShaderProgram {
 public:
  // ==== Attribute definitions ====
  /**
   * @brief vertex positions
   */
  typedef Magnum::Shaders::Generic3D::Position Position;

  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::Generic3D::ColorOutput,

    // TODO
    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     */
    // ObjectIdOutput = Magnum::Shaders::Generic3D::ObjectIdOutput,
  };

  /**
   * @brief Flag
   *
   * @see @ref Flags, @ref flags()
   */
  enum class Flag : Magnum::UnsignedShort {
    /**
     * Multiply base color with the baseColor texture.
     * @see @ref setBaseColor(), @ref bindBaseColorTexture()
     */
    ColorTexture = 1 << 0,
    // TODO
    // DepthTexture = 1 << 1,
    // ObjectIdTexture = 1 << 2,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;

  explicit DoubleSphereCameraShader(Flags flags = {Flag::ColorTexture});

 protected:
  Flags flags_;

  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // material uniforms
  int mvpUniform_ = ID_UNDEFINED;
  int colorTextureUniform_ = ID_UNDEFINED;
  // int depthTextureUniform_ = ID_UNDEFINED;
  // int objectIdTextureUniform_ = ID_UNDEFINED;
};

CORRADE_ENUMSET_OPERATORS(DoubleSphereCameraShader::Flags)

}  // namespace gfx
}  // namespace esp
#endif
