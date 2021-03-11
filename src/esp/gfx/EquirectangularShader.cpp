// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "EquirectangularShader.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

#include <sstream>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

EquirectangularShader::EquirectangularShader(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "EquirectangularShader::EquirectangularShader(): shader "
                 "flags cannot be empty.", );

    setTextureBindingPoints();
}

void EquirectangularShader::setTextureBindingPoints() {
  if (flags_ & EquirectangularShader::Flag::ColorTexture) {
    setUniform(uniformLocation("ColorTexture"),
               equirectangularShaderTexUnitSpace::TextureUnit::Color);
  }
  if (flags_ & EquirectangularShader::Flag::DepthTexture) {
    setUniform(uniformLocation("DepthTexture"),
               equirectangularShaderTexUnitSpace::TextureUnit::Depth);
  }
  // TODO: handle the other flags, ObjectIdTexture
}

EquirectangularShader& EquirectangularShader::bindColorTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & EquirectangularShader::Flag::ColorTexture,
                 "EquirectangularShader::bindColorTexture(): the shader was not "
                 "created with color texture enabled",
                 *this);
  texture.bind(equirectangularShaderTexUnitSpace::TextureUnit::Color);
  return *this;
}

EquirectangularShader& EquirectangularShader::bindDepthTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & EquirectangularShader::Flag::DepthTexture,
                 "EquirectangularShader::bindDepthTexture(): the shader was not "
                 "created with depth texture enabled",
                 *this);
  texture.bind(equirectangularShaderTexUnitSpace::TextureUnit::Depth);
  return *this;
}

}  // namespace gfx
}  // namespace esp
