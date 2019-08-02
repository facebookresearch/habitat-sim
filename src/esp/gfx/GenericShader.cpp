// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

// This is to import the "resources" at runtime.
// When the resource is compiled into static library,
// it must be explicitly initialized via this macro, and should be called
// *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

// TODO: Use Corrade resource file for shader code instead of hard-coding here

namespace {
enum { TextureLayer = 0 };
}

GenericShader::GenericShader(const Flags flags) : flags_(flags) {
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Magnum::GL::Version::GL410);

  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  // this is not the file name, but the group name in the config file
  const Corrade::Utility::Resource rs{"default-shaders"};

  Magnum::GL::Shader vert{Magnum::GL::Version::GL410,
                          Magnum::GL::Shader::Type::Vertex};
  Magnum::GL::Shader frag{Magnum::GL::Version::GL410,
                          Magnum::GL::Shader::Type::Fragment};

  vert.addSource(flags & Flag::Textured ? "#define TEXTURED\n" : "")
      .addSource(flags & Flag::VertexColored ? "#define VERTEX_COLORED\n" : "")
      .addSource(flags & Flag::PerVertexIds ? "#define PER_VERTEX_IDS\n" : "")
      .addSource(rs.get("generic-default-gl410.vert"));
  frag.addSource(flags & Flag::Textured ? "#define TEXTURED\n" : "")
      .addSource(flags & Flag::VertexColored ? "#define VERTEX_COLORED\n" : "")
      .addSource(flags & Flag::PerVertexIds ? "#define PER_VERTEX_IDS\n" : "")
      .addSource(flags & Flag::PrimitiveIDTextured ? "#define ID_TEXTURED\n"
                                                   : "")
      .addSource(rs.get("generic-default-gl410.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Magnum::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  if (flags & Flag::Textured) {
    setUniform(uniformLocation("textureData"), TextureLayer);
  }

  if (flags & Flag::PrimitiveIDTextured) {
    setUniform(uniformLocation("primTexture"), TextureLayer);
  }
}

GenericShader& GenericShader::bindTexture(Magnum::GL::Texture2D& texture) {
  ASSERT((flags_ & Flag::Textured) || (flags_ & Flag::PrimitiveIDTextured));

  texture.bind(TextureLayer);

  if (flags_ & Flag::PrimitiveIDTextured)
    setUniform(uniformLocation("texSize"), texture.imageSize(0).x());

  return *this;
}

}  // namespace gfx
}  // namespace esp
