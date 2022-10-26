// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "CubeMapShaderBase.h"

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
CubeMapShaderBase::CubeMapShaderBase(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "CubeMapShaderBase::CubeMapShaderBase(): shader "
                 "flags cannot be empty.", );
}

CubeMapShaderBase& CubeMapShaderBase::bindColorTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & CubeMapShaderBase::Flag::ColorTexture,
                 "CubeMapShaderBase::bindColorTexture(): the shader was not "
                 "created with color texture enabled",
                 *this);
  texture.bind(CubeMapShaderBaseTexUnitSpace::TextureUnit::Color);
  return *this;
}

CubeMapShaderBase& CubeMapShaderBase::bindDepthTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & CubeMapShaderBase::Flag::DepthTexture,
                 "CubeMapShaderBase::bindDepthTexture(): the shader was not "
                 "created with depth texture enabled",
                 *this);
  texture.bind(CubeMapShaderBaseTexUnitSpace::TextureUnit::Depth);
  return *this;
}

CubeMapShaderBase& CubeMapShaderBase::bindObjectIdTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & CubeMapShaderBase::Flag::ObjectIdTexture,
                 "CubeMapShaderBase::bindObjectIdTexture(): the shader was not "
                 "created with object id texture enabled",
                 *this);
  texture.bind(CubeMapShaderBaseTexUnitSpace::TextureUnit::ObjectId);
  return *this;
}

}  // namespace gfx
}  // namespace esp
