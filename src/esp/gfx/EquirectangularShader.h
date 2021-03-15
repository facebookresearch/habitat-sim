// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_EQUIRECTANGULARSHADER_H_
#define ESP_GFX_EQUIRECTANGULARSHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Macros.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Shaders/Generic.h>
#include "Magnum/GL/Mesh.h"
#include "esp/gfx/CubeMap.h"

#include "esp/core/esp.h"

namespace esp {
namespace gfx {
namespace equirectangularShaderTexUnitSpace {
enum TextureUnit : uint8_t {
  Color = 0,
  Depth = 1,
  // TODO
  // ObjectId = 2,
};
}
class EquirectangularShader : public Magnum::GL::AbstractShaderProgram {
 public:
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
     * cubemap color texture
     */
    ColorTexture = 1 << 0,
    /**
     * cubemap depth texture
     */
    DepthTexture = 1 << 1,
    // ObjectIdTexture = 1 << 2,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;

  /**
   * @brief constructor
   * @param[in] flags equirectangular shader flags
   */
  explicit EquirectangularShader(Flags flags = {Flag::ColorTexture});

  ~EquirectangularShader() override = default;

  /** @brief Flags */
  Flags flags() const { return flags_; }

  /**
   * @brief bind cubemap color texture
   * @param[in] texture cubemap color texture
   */
  EquirectangularShader& bindColorTexture(Magnum::GL::CubeMapTexture& texture);
  /**
   * @brief bind cubemap depth texture
   * @param[in] texture cubemap depth texture
   */
  EquirectangularShader& bindDepthTexture(Magnum::GL::CubeMapTexture& texture);
  // EquirectangularShader& bindObjectIdTexture(Magnum::GL::Texture2D&
  // texture);

  /**
   * @brief Set ViewportSize for calculations in vert
   * @param[in] Vector2i viewportSize
   */
  EquirectangularShader& setViewportSize(esp::vec2i viewportSize);

 protected:
  Flags flags_;

  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // using the function @ref cacheUniforms()
  // common material uniforms
  int viewportHeight_ = ID_UNDEFINED;
  int viewportWidth_ = ID_UNDEFINED;
  /**
   * @brief cache the uniform locations
   */
  void cacheUniforms();

  /**
   * @brief set texture binding points in the shader
   * NOTE: Subclass must implement this function, and call it at the end of
   * the constructor
   */
  void setTextureBindingPoints();
};
CORRADE_ENUMSET_OPERATORS(EquirectangularShader::Flags)
}  // namespace gfx
}  // namespace esp
#endif
