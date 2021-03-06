// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_FISHEYESHADER_H_
#define ESP_GFX_FISHEYESHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Macros.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/Shaders/Generic.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {
namespace fisheyeShaderTexUnitSpace {
enum TextureUnit : uint8_t {
  Color = 0,
  Depth = 1,
  // TODO
  // ObjectId = 2,
};
}
// Interface class for various fisheye camera shaders, such as "Double Sphere
// Camera", "Field-of-View Camera", etc.
class FisheyeShader : public Magnum::GL::AbstractShaderProgram {
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
   * @param[in] flags fisheye shader flags
   */
  explicit FisheyeShader(Flags flags = {Flag::ColorTexture});

  virtual ~FisheyeShader() = default;

  /** @brief Flags */
  Flags flags() const { return flags_; }

  /**
   * @brief bind cubemap color texture
   * @param[in] texture cubemap color texture
   */
  virtual FisheyeShader& bindColorTexture(Magnum::GL::CubeMapTexture& texture);
  /**
   * @brief bind cubemap depth texture
   * @param[in] texture cubemap depth texture
   */
  virtual FisheyeShader& bindDepthTexture(Magnum::GL::CubeMapTexture& texture);
  // virtual FisheyeShader& bindObjectIdTexture(Magnum::GL::Texture2D&
  // texture);

 protected:
  Flags flags_;

  // ======= uniforms =======
  // it hurts the performance to call glGetUniformLocation() every frame due
  // to string operations. therefore, cache the locations in the constructor
  // using the function @ref cacheUniforms()

  // common material uniforms

  /**
   * @brief cache the uniform locations
   * NOTE: Subclass must implement this function, and call it at the end of
   * the constructor
   */
  virtual void cacheUniforms() = 0;

  /**
   * @brief set texture binding points in the shader
   * NOTE: Subclass must implement this function, and call it at the end of
   * the constructor
   */
  virtual void setTextureBindingPoints() = 0;
};
CORRADE_ENUMSET_OPERATORS(FisheyeShader::Flags)
}  // namespace gfx
}  // namespace esp
#endif
