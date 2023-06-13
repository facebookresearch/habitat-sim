// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrPrecomputedMapShader.h"
#include "PbrTextureUnit.h"

#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Matrix4.h>

#include <initializer_list>
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

PbrPrecomputedMapShader::PbrPrecomputedMapShader(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(
      !((flags_ & Flag::IrradianceMap) && (flags_ & Flag::PrefilteredMap)),
      "PbrPrecomputedMapShader::PbrPrecomputedMapShader: "
      "Flag::IrradianceMap and "
      "Flag::PrefilteredMap are mutually exclusive.", );

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
  vert
      .addSource(Cr::Utility::formatString(
          "#define ATTRIBUTE_LOCATION_POSITION {}\n", Position::Location))
      .addSource(rs.getString("pbrPrecomputedMap.vert"));

  frag
      .addSource(Cr::Utility::formatString(
          "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput))
      .addSource(rs.getString("pbrCommon.glsl") + "\n");

  if (flags & Flag::IrradianceMap) {
    frag.addSource(rs.getString("pbrIrradianceMap.frag"));
  } else if (flags & Flag::PrefilteredMap) {
    frag.addSource(rs.getString("pbrPrefilteredMap.frag"));
  }

  CORRADE_INTERNAL_ASSERT_OUTPUT(vert.compile() && frag.compile());

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // set texture unit
  setUniform(uniformLocation("EnvironmentMap"),
             pbrTextureUnitSpace::TextureUnit::EnvironmentMap);

  // setup uniforms
  modelviewMatrixUniform_ = uniformLocation("ModelViewMatrix");
  projMatrixUniform_ = uniformLocation("ProjectionMatrix");
}

PbrPrecomputedMapShader& PbrPrecomputedMapShader::bindEnvironmentMap(
    Mn::GL::CubeMapTexture& texture) {
  texture.bind(pbrTextureUnitSpace::TextureUnit::EnvironmentMap);
  return *this;
}

PbrPrecomputedMapShader& PbrPrecomputedMapShader::setProjectionMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(projMatrixUniform_, matrix);
  return *this;
}

PbrPrecomputedMapShader& PbrPrecomputedMapShader::setTransformationMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(modelviewMatrixUniform_, matrix);
  return *this;
}

PbrPrecomputedMapShader& PbrPrecomputedMapShader::setRoughness(
    float roughness) {
  CORRADE_ASSERT(flags_ & Flag::PrefilteredMap,
                 "PbrPrecomputedMapShader::setRoughness(): shader is NOT "
                 "created to compute the prefiltered maps.",
                 *this);
  setUniform(roughnessUniform_, roughness);
  return *this;
}

}  // namespace gfx
}  // namespace esp
