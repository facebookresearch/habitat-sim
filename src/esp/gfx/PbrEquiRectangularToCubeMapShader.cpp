// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#include "PbrEquiRectangularToCubeMapShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <sstream>

// This is to import the "resources" at runtime. When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxShaderResources)
}

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

enum {
  EquirectangularTextureUnit = 1,
};

PbrEquiRectangularToCubeMapShader::PbrEquiRectangularToCubeMapShader() {
  if (!Corrade::Utility::Resource::hasGroup("gfx-shaders")) {
    importShaderResources();
  }

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  // this is not the file name, but the group name in the config file
  // see Shaders.conf in the shaders folder
  const Cr::Utility::Resource rs{"gfx-shaders"};

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  // Add macros
  vert.addSource("#define OUTPUT_UV\n")
      .addSource(rs.getString("bigTriangle.vert"));

  frag
      .addSource(Cr::Utility::formatString(
          "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput))
      .addSource(rs.getString("equirectangularToCubeMap.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(vert.compile() && frag.compile());

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // setup texture binding point
  setUniform(uniformLocation("uEquirectangularTexture"),
             EquirectangularTextureUnit);

  // setup uniforms
  cubeSideIndexUniform_ = uniformLocation("uCubeSideIndex");
}

PbrEquiRectangularToCubeMapShader&
PbrEquiRectangularToCubeMapShader::bindEquirectangularTexture(
    Magnum::GL::Texture2D& texture) {
  texture.bind(EquirectangularTextureUnit);
  return *this;
}

PbrEquiRectangularToCubeMapShader&
PbrEquiRectangularToCubeMapShader::setCubeSideIndex(unsigned int index) {
  setUniform(cubeSideIndexUniform_, index);
  return *this;
}

}  // namespace gfx
}  // namespace esp
