// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericShader.h"

#include <Corrade/Containers/Reference.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>

namespace esp {
namespace gfx {

// TODO: Use Corrade resource file for shader code instead of hard-coding here

const static std::string GENERIC_SHADER_VS = R"(
uniform highp mat4 transformationProjectionMatrix;
uniform highp mat4 projectionMatrix;

layout(location = 0) in highp vec4 position;

#ifdef TEXTURED
layout(location = 1) in mediump vec2 textureCoordinates;
out mediump vec2 interpolatedTextureCoordinates;
#endif

#ifdef VERTEX_COLORED
layout(location = 1) in mediump vec3 color;
#endif

out mediump vec3 v_color;
out highp float v_depth;

#ifdef PER_VERTEX_IDS
flat out uint v_objectId;
#endif

void main() {
  gl_Position = transformationProjectionMatrix * vec4(position.xyz, 1.0);

  #ifdef TEXTURED
  interpolatedTextureCoordinates = textureCoordinates;
  #endif

  vec4 pointInCameraCoords = projectionMatrix * vec4(position.xyz, 1.0);
  pointInCameraCoords /= pointInCameraCoords.w;

  #ifdef VERTEX_COLORED
  v_color = color;
  #endif
  #ifdef PER_VERTEX_IDS
  v_objectId = uint(position.w);
  #endif

  v_depth = -pointInCameraCoords.z;
}
)";

const static std::string GENERIC_SHADER_FS = R"(
in mediump vec3 v_color;
in highp float v_depth;


#ifdef PER_VERTEX_IDS
flat in uint v_objectId;
#else
uniform highp int objectIdUniform;
#endif

#ifdef TEXTURED
uniform lowp sampler2D textureData;
in mediump vec2 interpolatedTextureCoordinates;
#endif

#ifdef ID_TEXTURED
uniform highp sampler2D primTexture;
uniform highp int texSize;
#endif

#ifndef VERTEX_COLORED
uniform lowp vec4 colorUniform;
#endif

layout(location = 0) out mediump vec4 color;
layout(location = 1) out highp float depth;
layout(location = 2) out uint objectId;

void main () {
  mediump vec4 baseColor =
    #ifdef VERTEX_COLORED
    vec4(v_color, 1.0);
    #else
    colorUniform;
    #endif
  color =
    #ifdef TEXTURED
    texture(textureData, interpolatedTextureCoordinates) *
    #endif
    baseColor;
  depth = v_depth;
  objectId =
  #ifdef PER_VERTEX_IDS
    v_objectId;
  #else
    uint(objectIdUniform);
  #endif

  #ifdef ID_TEXTURED
  objectId = uint(
      texture(primTexture,
              vec2((float(gl_PrimitiveID % texSize) + 0.5f) / float(texSize),
                   (float(gl_PrimitiveID / texSize) + 0.5f) / float(texSize)))
          .r + 0.5);
  #endif
}
)";

namespace {
enum { TextureLayer = 0 };
}

GenericShader::GenericShader(const Flags flags) : flags_(flags) {
  // MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Magnum::GL::Version::GL410);
#ifdef MAGNUM_TARGET_WEBGL
  Magnum::GL::Version glVersion = Magnum::GL::Version::GLES300;
#else
  Magnum::GL::Version glVersion = Magnum::GL::Version::GL410;
#endif
  Magnum::GL::Shader vert{glVersion, Magnum::GL::Shader::Type::Vertex};
  Magnum::GL::Shader frag{glVersion, Magnum::GL::Shader::Type::Fragment};

  vert.addSource(flags & Flag::Textured ? "#define TEXTURED\n" : "")
      .addSource(flags & Flag::VertexColored ? "#define VERTEX_COLORED\n" : "")
      .addSource(flags & Flag::PerVertexIds ? "#define PER_VERTEX_IDS\n" : "")
      .addSource(GENERIC_SHADER_VS);
  frag.addSource(flags & Flag::Textured ? "#define TEXTURED\n" : "")
      .addSource(flags & Flag::VertexColored ? "#define VERTEX_COLORED\n" : "")
      .addSource(flags & Flag::PerVertexIds ? "#define PER_VERTEX_IDS\n" : "")
      .addSource(flags & Flag::PrimitiveIDTextured ? "#define ID_TEXTURED\n"
                                                   : "")
      .addSource(GENERIC_SHADER_FS);

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

// TODO this is a hack and terrible! Properly set texSize for WebGL builds
#ifndef MAGNUM_TARGET_WEBGL
  if (flags_ & Flag::PrimitiveIDTextured)
    setUniform(uniformLocation("texSize"), texture.imageSize(0).x());
#endif

  return *this;
}

}  // namespace gfx
}  // namespace esp
