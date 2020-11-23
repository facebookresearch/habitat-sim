// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "DoubleSphereCameraShader.h"

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
namespace {
enum TextureUnit : uint8_t {
  Color = 0,
  Depth = 1,
  ObjectId = 2,
};
}  // namespace
DoubleSphereCameraShader::DoubleSphereCameraShader(Flags flags)
    : flags_(flags) {
  CORRADE_ASSERT(flags != Flags{},
                 "DoubleSphereCameraShader::DoubleSphereCameraShader(): shader "
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

  std::stringstream attributeLocationsStream;
  attributeLocationsStream << Cr::Utility::formatString(
      "#define ATTRIBUTE_LOCATION_POSITION {}\n", Position::Location);

  // Add macros
  vert.addSource(attributeLocationsStream.str())
      .addSource(rs.get("doubleSphereCamera.vert"));

  std::stringstream outputAttributeLocationsStream;
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput);
  /* TODO:
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID {}\n", ObjectIdOutput);
  */

  frag.addSource(outputAttributeLocationsStream.str())
      .addSource(flags_ & Flag::ColorTexture ? "#define COLOR_TEXTURE\n" : "")
      // TODO
      //.addSource(flags_ & Flag::DepthTexture ? "#define DEPTH_TEXTURE\n" : "")
      .addSource(rs.get("doubleSphereCamera.frag"));
}

}  // namespace gfx
}  // namespace esp
