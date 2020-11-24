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

#include <sstream>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {
FisheyeShader::FisheyeShader(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "FisheyeShader::FisheyeShader(): shader "
                 "flags cannot be empty.", );
}

FisheyeShader& FisheyeShader::bindColorTexture(Magnum::GL::Texture2D& texture) {
  return *this;
}

}  // namespace gfx
}  // namespace esp
