// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "EquirectangularShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include "esp/gfx/CubeMap.h"

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

EquirectangularShader::EquirectangularShader(Flags flags) : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "EquirectangularShader::EquirectangularShader(): shader "
                 "flags cannot be empty.", );

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

  if (flags_ & EquirectangularShader::Flag::ColorTexture) {
    outputAttributeLocationsStream << Cr::Utility::formatString(
        "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput);
  }
  /* TODO:
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID {}\n", ObjectIdOutput);
  */

  frag.addSource(outputAttributeLocationsStream.str())
      .addSource(flags_ & EquirectangularShader::Flag::ColorTexture
                     ? "#define COLOR_TEXTURE\n"
                     : "")
      .addSource(flags_ & EquirectangularShader::Flag::DepthTexture
                     ? "#define DEPTH_TEXTURE\n"
                     : "")
      .addSource(rs.get("equirectangular.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  setTextureBindingPoints();
  cacheUniforms();
}

void EquirectangularShader::cacheUniforms() {
  viewportHeight_ = uniformLocation("ViewportHeight");
  viewportWidth_ = uniformLocation("ViewportWidth");
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

EquirectangularShader& EquirectangularShader::setViewportSize(
    esp::vec2i viewportSize) {
  setUniform(viewportHeight_, viewportSize[0]);
  setUniform(viewportWidth_, viewportSize[1]);
  return *this;
}

EquirectangularShader& EquirectangularShader::bindColorTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(
      flags_ & EquirectangularShader::Flag::ColorTexture,
      "EquirectangularShader::bindColorTexture(): the shader was not "
      "created with color texture enabled",
      *this);
  texture.bind(equirectangularShaderTexUnitSpace::TextureUnit::Color);
  return *this;
}

EquirectangularShader& EquirectangularShader::bindDepthTexture(
    Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(
      flags_ & EquirectangularShader::Flag::DepthTexture,
      "EquirectangularShader::bindDepthTexture(): the shader was not "
      "created with depth texture enabled",
      *this);
  texture.bind(equirectangularShaderTexUnitSpace::TextureUnit::Depth);
  return *this;
}

}  // namespace gfx
}  // namespace esp
