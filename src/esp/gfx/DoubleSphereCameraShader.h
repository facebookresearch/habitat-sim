// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_DOUBLESPHERECAMERASHADER_H_
#define ESP_GFX_DOUBLESPHERECAMERASHADER_H_

#include <Corrade/Containers/EnumSet.h>
#include <Magnum/Shaders/Generic.h>

#include "FisheyeShader.h"
#include "esp/core/esp.h"

namespace esp {
namespace gfx {
class DoubleSphereCameraShader : public FisheyeShader {
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
     * present only if @ref FisheyeShader::Flag::ObjectId is set. Expects a
     * single-component unsigned integral attachment. Writes the value
     * set in @ref setObjectId() there.
     */
    // ObjectIdOutput = Magnum::Shaders::Generic3D::ObjectIdOutput,
  };

  explicit DoubleSphereCameraShader(FisheyeShader::Flags flags = {
                                        FisheyeShader::Flag::ColorTexture});

  virtual ~DoubleSphereCameraShader(){};

  virtual DoubleSphereCameraShader& bindColorTexture(
      Magnum::GL::Texture2D& texture);

 protected:
  int focalLengthUniform_ = ID_UNDEFINED;
  int principalPointOffsetUniform_ = ID_UNDEFINED;
  int alphaUniform_ = ID_UNDEFINED;
  int xiUniform_ = ID_UNDEFINED;
  virtual void cacheUniforms() override;
  virtual void setTextureBindingPoints() override;
};

}  // namespace gfx
}  // namespace esp
#endif
