// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DoubleSphereCameraShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

#include <sstream>

// This is to import the "resources" at runtime. // When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called // *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {
DoubleSphereCameraShader::DoubleSphereCameraShader(FisheyeShader::Flags flags)
    : FisheyeShader(flags) {
  if (!Cr::Utility::Resource::hasGroup("default-shaders")) {
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
  vert.addSource(rs.get("bigTriangle.vert"));

  std::stringstream outputAttributeLocationsStream;

  if (flags_ & FisheyeShader::Flag::ColorTexture) {
    outputAttributeLocationsStream << Cr::Utility::formatString(
        "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput);
  }
  /* TODO:
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID {}\n", ObjectIdOutput);
  */

  frag.addSource(outputAttributeLocationsStream.str())
      .addSource(flags_ & FisheyeShader::Flag::ColorTexture
                     ? "#define COLOR_TEXTURE\n"
                     : "")
      .addSource(flags_ & FisheyeShader::Flag::DepthTexture
                     ? "#define DEPTH_TEXTURE\n"
                     : "")
      .addSource(rs.get("doubleSphereCamera.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  setTextureBindingPoints();
  cacheUniforms();
}

void DoubleSphereCameraShader::cacheUniforms() {
  focalLengthUniform_ = uniformLocation("FocalLength");
  principalPointOffsetUniform_ = uniformLocation("PrincipalPointOffset");
  alphaUniform_ = uniformLocation("Alpha");
  xiUniform_ = uniformLocation("Xi");
}

void DoubleSphereCameraShader::setTextureBindingPoints() {
  if (flags_ & FisheyeShader::Flag::ColorTexture) {
    setUniform(uniformLocation("ColorTexture"),
               fisheyeShaderTexUnitSpace::TextureUnit::Color);
  }
  if (flags_ & FisheyeShader::Flag::DepthTexture) {
    setUniform(uniformLocation("DepthTexture"),
               fisheyeShaderTexUnitSpace::TextureUnit::Depth);
  }
  // TODO: handle the other flags, ObjectIdTexture
}

DoubleSphereCameraShader& DoubleSphereCameraShader::setFocalLength(
    Magnum::Vector2 focalLength) {
  setUniform(focalLengthUniform_, focalLength);
  return *this;
}

DoubleSphereCameraShader& DoubleSphereCameraShader::setPrincipalPointOffset(
    Magnum::Vector2 offset) {
  setUniform(principalPointOffsetUniform_, offset);
  return *this;
}

DoubleSphereCameraShader& DoubleSphereCameraShader::setAlpha(float alpha) {
  setUniform(alphaUniform_, alpha);
  return *this;
}

DoubleSphereCameraShader& DoubleSphereCameraShader::setXi(float xi) {
  setUniform(xiUniform_, xi);
  return *this;
}

}  // namespace gfx
}  // namespace esp
