// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <memory>
#include <vector>

#include <Magnum/GL/AbstractShaderProgram.h>
#include <Magnum/Math/Matrix4.h>

#include "esp/assets/PTexMeshData.h"

namespace esp {

// forward declaration
namespace assets {
class PTexMeshData;
};

namespace gfx {

class PTexMeshShader : public Magnum::GL::AbstractShaderProgram {
 public:
  typedef Magnum::GL::Attribute<0, Magnum::Vector4> Position;

  explicit PTexMeshShader();

  // ======== texture binding ========
  PTexMeshShader& bindAtlasTexture(Magnum::GL::Texture2D& texture);
  PTexMeshShader& bindAdjFacesBufferTexture(Magnum::GL::BufferTexture& texture);

  // ======== set uniforms ===========
  PTexMeshShader& setMVPMatrix(const Magnum::Matrix4& matrix);
  PTexMeshShader& setExposure(float exposure);
  PTexMeshShader& setGamma(float gamma);
  PTexMeshShader& setSaturation(float saturation);
  PTexMeshShader& setAtlasTextureSize(Magnum::GL::Texture2D& texture,
                                      uint32_t tileSize);

 protected:
  // it hurts the performance to call glGetUniformLocation() every frame due to
  // string operations.
  // therefore, cache the locations in the constructor
  int MVPMatrixUniform_;
  int exposureUniform_;
  int gammaInverseUniform_;
  int saturationUniform_;
  int tileSizeUniform_;
  int widthInTilesUniform_;
};

}  // namespace gfx
}  // namespace esp
