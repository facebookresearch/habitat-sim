// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_CUBEMAP_SHADER_BASE_H_
#define ESP_GFX_CUBEMAP_SHADER_BASE_H_

#include <Corrade/Containers/EnumSet.h>
#include <Corrade/Utility/Macros.h>
#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/ResourceManager.h>
#include <Magnum/Shaders/GenericGL.h>

#include "esp/core/Esp.h"

namespace esp {
namespace gfx {
namespace CubeMapShaderBaseTexUnitSpace {
enum TextureUnit : uint8_t {
  Color = 0,
  Depth = 1,
  ObjectId = 2,
};
}
// Interface class for various cubemap based camera shaders, such as "Double
// Sphere Camera" (fisheye), "Field-of-View Camera" (fisheye), "equiRectangular"
// etc.
class CubeMapShaderBase : public Magnum::GL::AbstractShaderProgram {
 public:
  enum : Magnum::UnsignedInt {
    /**
     * Color shader output. @ref shaders-generic "Generic output",
     * present always. Expects three- or four-component floating-point
     * or normalized buffer attachment.
     */
    ColorOutput = Magnum::Shaders::GenericGL3D::ColorOutput,

    /**
     * Object ID shader output. @ref shaders-generic "Generic output",
     * present only if @ref Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     */
    ObjectIdOutput = Magnum::Shaders::GenericGL3D::ObjectIdOutput,
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
    /**
     * cubemap object id texture
     */
    ObjectIdTexture = 1 << 2,
  };

  typedef Corrade::Containers::EnumSet<Flag> Flags;

  ~CubeMapShaderBase() override = default;

  /** @brief Flags */
  Flags flags() const { return flags_; }

  /**
   * @brief bind cubemap color texture
   * @param[in] texture cubemap color texture
   */
  virtual CubeMapShaderBase& bindColorTexture(
      Magnum::GL::CubeMapTexture& texture);
  /**
   * @brief bind cubemap depth texture
   * @param[in] texture cubemap depth texture
   */
  virtual CubeMapShaderBase& bindDepthTexture(
      Magnum::GL::CubeMapTexture& texture);

  /**
   * @brief bind cubemap object id texture
   * @param[in] texture cubemap object id texture
   */
  virtual CubeMapShaderBase& bindObjectIdTexture(
      Magnum::GL::CubeMapTexture& texture);

 protected:
  /**
   * @brief constructor
   * @param[in] flags cubemap shader flags
   */
  explicit CubeMapShaderBase(Flags flags = {Flag::ColorTexture});

  Flags flags_;
};

CORRADE_ENUMSET_OPERATORS(CubeMapShaderBase::Flags)
}  // namespace gfx
}  // namespace esp
#endif
