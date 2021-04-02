// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "TextureVisualizerShader.h"

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

#include "esp/core/esp.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

enum { DepthTextureUnit = 1 };

TextureVisualizerShader::TextureVisualizerShader() {
  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  const Corrade::Utility::Resource rs{"default-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  vert.addSource("#define UNPROJECT_EXISTING_DEPTH\n")
      .addSource(rs.get("depth.vert"));

  frag.addSource("#define UNPROJECT_EXISTING_DEPTH\n")
      .addSource("#define DEPTH_VISUALIZATER\n")
      .addSource(rs.get("depth.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  setUniform(uniformLocation("depthTexture"), DepthTextureUnit);

  depthUnprojectionUniform_ = uniformLocation("depthUnprojection");
  CORRADE_INTERNAL_ASSERT(depthUnprojectionUniform_ != ID_UNDEFINED);
  depthScalingUniform_ = uniformLocation("depthScaling");
  CORRADE_INTERNAL_ASSERT(depthScalingUniform_ != ID_UNDEFINED);
}

TextureVisualizerShader& TextureVisualizerShader::bindDepthTexture(
    Mn::GL::Texture2D& texture) {
  texture.bind(DepthTextureUnit);
  return *this;
}

TextureVisualizerShader& TextureVisualizerShader::setDepthUnprojection(
    const Mn::Vector2& depthUnprojection) {
  setUniform(depthUnprojectionUniform_, depthUnprojection);
  return *this;
}

TextureVisualizerShader& TextureVisualizerShader::setDepthScaling(
    float depthScaling) {
  setUniform(depthScalingUniform_, depthScaling);
  return *this;
}

}  // namespace gfx
}  // namespace esp
