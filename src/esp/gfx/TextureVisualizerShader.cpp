// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "TextureVisualizerShader.h"
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

#include "esp/core/esp.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace esp {
namespace gfx {

enum {
  SourceTextureUnit = 1,
  ColorMapTextureUnit = 2,
};

TextureVisualizerShader::TextureVisualizerShader(Flags flags) : flags_(flags) {
  int countMutuallyExclusive = 0;
  if (flags_ & Flag::DepthTexture) {
    ++countMutuallyExclusive;
  }
  if (flags & Flag::ObjectIdTexture) {
    ++countMutuallyExclusive;
  }
  CORRADE_ASSERT(countMutuallyExclusive <= 1,
                 "TextureVisualizerShader::TextureVisualizerShader: "
                 "Flag::DepthTexture and "
                 "Flag::ObjectIdTexture are mutually exclusive.", );

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

  vert.addSource("#define OUTPUT_UV\n").addSource(rs.get("bigTriangle.vert"));

  frag.addSource("#define EXPLICIT_ATTRIB_LOCATION\n")
      .addSource(Cr::Utility::formatString(
          "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput))
      .addSource(flags_ & Flag::DepthTexture ? "#define DEPTH_TEXTURE\n" : "")
      .addSource(flags_ & Flag::ObjectIdTexture ? "#define OBJECT_ID_TEXTURE\n"
                                                : "")
      .addSource(rs.get("textureVisualizer.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // setup texture binding point
  setUniform(uniformLocation("sourceTexture"), SourceTextureUnit);
  setUniform(uniformLocation("colorMapTexture"), ColorMapTextureUnit);

  // setup uniforms
  colorMapOffsetScaleUniform_ = uniformLocation("colorMapOffsetScale");
  CORRADE_INTERNAL_ASSERT(colorMapOffsetScaleUniform_ >= 0);
  if (flags_ & Flag::DepthTexture) {
    depthUnprojectionUniform_ = uniformLocation("depthUnprojection");
    CORRADE_INTERNAL_ASSERT(depthUnprojectionUniform_ >= 0);
  }

  // setup color map texture
  const auto map = Mn::DebugTools::ColorMap::turbo();
  const Mn::Vector2i size{int(map.size()), 1};
  colorMapTexture_.setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setStorage(1, Mn::GL::TextureFormat::SRGB8Alpha8, size)
      .setSubImage(0, {},
                   Mn::ImageView2D{Mn::PixelFormat::RGB8Srgb, size, map});
  if (flags_ & Flag::DepthTexture) {
    colorMapTexture_.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge);
  } else if (flags_ & Flag::ObjectIdTexture) {
    colorMapTexture_.setWrapping(Mn::GL::SamplerWrapping::Repeat);
  }
  colorMapTexture_.bind(ColorMapTextureUnit);

  // set default offset, scale based on flags
  if (flags_ & Flag::DepthTexture) {
    setColorMapTransformation(1.0f / 512.0f, 1.0f / 1000.0f);
  } else if (flags & Flag::ObjectIdTexture) {
    setColorMapTransformation(1.0f / 512.0f,
                              1.0f / 108.0f);  // initial guess: 108 objects
  }
}

TextureVisualizerShader& TextureVisualizerShader::setColorMapTransformation(
    float offset,
    float scale) {
  CORRADE_ASSERT(offset >= 0.0,
                 "TextureVisualizerShader::setColorMapTransformation(): offset"
                     << offset << "is illegal.",
                 *this);
  CORRADE_ASSERT(scale >= 0.0,
                 "TextureVisualizerShader::setColorMapTransformation(): scale"
                     << scale << "is illegal.",
                 *this);
  setUniform(colorMapOffsetScaleUniform_, Mn::Vector2{offset, scale});
  return *this;
}

TextureVisualizerShader& TextureVisualizerShader::bindDepthTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::DepthTexture,
                 "TextureVisualizerShader::bindDepthTexture(): the shader was "
                 "not created with depth texture enabled",
                 *this);
  texture.bind(SourceTextureUnit);
  return *this;
}

TextureVisualizerShader& TextureVisualizerShader::bindObjectIdTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::ObjectIdTexture,
      "TextureVisualizerShader::bindObjectIdTexture(): the shader was "
      "not created with object Id texture enabled",
      *this);
  texture.bind(SourceTextureUnit);
  return *this;
}

TextureVisualizerShader& TextureVisualizerShader::setDepthUnprojection(
    const Mn::Vector2& depthUnprojection) {
  CORRADE_ASSERT(flags_ & Flag::DepthTexture,
                 "TextureVisualizerShader::bindDepthTexture(): the shader was "
                 "not created with depth texture enabled",
                 *this);
  setUniform(depthUnprojectionUniform_, depthUnprojection);
  return *this;
}

}  // namespace gfx
}  // namespace esp
