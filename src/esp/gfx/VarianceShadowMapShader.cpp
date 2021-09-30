// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VarianceShadowMapShader.h"

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
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

VarianceShadowMapShader::VarianceShadowMapShader() {
  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL410;
#endif

  // this is not the file name, but the group name in the config file
  // see Shaders.conf in the shaders folder
  const Cr::Utility::Resource rs{"default-shaders"};

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  // Add macros
  vert
      .addSource(Cr::Utility::formatString(
          "#define ATTRIBUTE_LOCATION_POSITION {}\n", Position::Location))
      .addSource(rs.get("varianceShadowMap.vert"));

  frag
      .addSource(Cr::Utility::formatString(
          "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput))
      .addSource(rs.get("varianceShadowMap.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // setup uniforms
  lightModelViewMatrixUniform_ = uniformLocation("LightModelViewMatrix");
  lightProjectionMatrixUniform_ = uniformLocation("LightProjectionMatrix");
}
VarianceShadowMapShader& VarianceShadowMapShader::setLightProjectionMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(lightProjectionMatrixUniform_, matrix);
  return *this;
}

VarianceShadowMapShader& VarianceShadowMapShader::setLightModelViewMatrix(
    const Mn::Matrix4& matrix) {
  setUniform(lightModelViewMatrixUniform_, matrix);
  return *this;
}

}  // namespace gfx
}  // namespace esp
