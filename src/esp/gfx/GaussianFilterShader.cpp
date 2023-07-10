// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GaussianFilterShader.h"
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/DebugTools/ColorMap.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

namespace Cr = Corrade;
namespace Mn = Magnum;

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxShaderResources)
}

namespace esp {
namespace gfx {

enum {
  SourceTextureUnit = 1,
};

GaussianFilterShader::GaussianFilterShader() {
  if (!Corrade::Utility::Resource::hasGroup("gfx-shaders")) {
    importShaderResources();
  }

  const Corrade::Utility::Resource rs{"gfx-shaders"};

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  vert.addSource("#define OUTPUT_UV\n")
      .addSource(rs.getString("bigTriangle.vert"));

  frag.addSource("#define EXPLICIT_ATTRIB_LOCATION\n")
      .addSource(Cr::Utility::formatString(
          "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput))
      .addSource(rs.getString("gaussianFilter.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(vert.compile() && frag.compile());

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // setup texture binding point
  setUniform(uniformLocation("SourceTexture"), SourceTextureUnit);

  // setup uniforms
  filterDirectionUniform_ = uniformLocation("FilterDirection");
  CORRADE_INTERNAL_ASSERT(filterDirectionUniform_ >= 0);
}

GaussianFilterShader& GaussianFilterShader::bindTexture(
    Magnum::GL::Texture2D& texture) {
  texture.bind(SourceTextureUnit);
  return *this;
}

GaussianFilterShader& GaussianFilterShader::setFilteringDirection(
    FilteringDirection dir) {
  if (dir == FilteringDirection::Horizontal) {
    setUniform(filterDirectionUniform_, Mn::Vector2(1.0, 0.0));
  } else {
    setUniform(filterDirectionUniform_, Mn::Vector2(0.0, 1.0));
  }
  return *this;
}

}  // namespace gfx
}  // namespace esp
