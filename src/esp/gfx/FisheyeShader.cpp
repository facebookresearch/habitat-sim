// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "FisheyeShader.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {
FisheyeShader::FisheyeShader(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "FisheyeShader::FisheyeShader(): shader "
                 "flags cannot be empty.", );
}
FisheyeShader& FisheyeShader::bindColorTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & FisheyeShader::Flag::ColorTexture,
                 "FisheyeShader::bindColorTexture(): the shader was not "
                 "created with color texture enabled",
                 *this);
  texture.bind(fisheyeShaderTexUnitSpace::TextureUnit::Color);
  return *this;
}

FisheyeShader& FisheyeShader::bindDepthTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & FisheyeShader::Flag::DepthTexture,
                 "FisheyeShader::bindDepthTexture(): the shader was not "
                 "created with depth texture enabled",
                 *this);
  texture.bind(fisheyeShaderTexUnitSpace::TextureUnit::Depth);
  return *this;
}

}  // namespace gfx
}  // namespace esp
