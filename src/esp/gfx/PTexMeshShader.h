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

  PTexMeshShader& bindTexture(Magnum::GL::Texture2D& texture,
                              uint32_t textureUnit = 0) {
    texture.bind(textureUnit);
    return *this;
  }

  PTexMeshShader& setMVPMatrix(const Magnum::Matrix4& matrix) {
    setUniform(uniformLocation("MVP"), matrix);
    return *this;
  }

  PTexMeshShader& setPTexUniforms(assets::PTexMeshData& ptexMeshData,
                                  int submeshID,
                                  uint32_t tileSize,
                                  float exposure) {
    setPTexUniforms(ptexMeshData.getRenderingBuffer(submeshID)->tex, tileSize,
                    exposure);
    return *this;
  }

  PTexMeshShader& setPTexUniforms(Magnum::GL::Texture2D& tex,
                                  uint32_t tileSize,
                                  float exposure) {
    setUniform(uniformLocation("atlasTex"), 0);
    setUniform(uniformLocation("tileSize"), (int)tileSize);
    // Image size in given mip level 0
    {
      int mipLevel = 0;
      int widthEntry = 0;
      const auto width = tex.imageSize(mipLevel)[widthEntry];
      setUniform(uniformLocation("widthInTiles"), int(width / tileSize));
    }
    setUniform(uniformLocation("exposure"), exposure);
    return *this;
  }
};

}  // namespace gfx
}  // namespace esp
