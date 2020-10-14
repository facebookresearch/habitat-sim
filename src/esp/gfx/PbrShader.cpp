// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrShader.h"

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/PixelFormat.h>

#include "esp/core/esp.h"
#include "esp/io/io.h"

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

namespace {
enum TextureUnit : uint8_t {
  BaseColor = 0,
  Roughness = 1,
  Metallic = 2,
  Normal = 3,
  // NoneRoughnessMetallic or OcclusionRoughnessMetallic texture
  Packed = 4,
  Emissive = 5,
};
}  // namespace

PbrShader::Flags PbrShader::generateCorrectFlags(Flags originalFlags) {
  Flags result = originalFlags;
  // NOTE:
  // The priority of different kind of textures is as follows (priority
  // means if two textures with different priorities exist at the same time,
  // shader will adopt the texture with the higher priority, and ignore the
  // other one.)
  // 0 (highest): OcclusionRoughnessMetallicTexture
  // 1          : NoneRoughnessMetallicTexture
  // 2          : RoughnessTexture, MetallicTexture
  if (result & Flag::OcclusionRoughnessMetallicTexture) {
    result &= ~Flag::NoneRoughnessMetallicTexture;
    result &= ~Flag::RoughnessTexture;
    result &= ~Flag::MetallicTexture;
  } else if (result & Flag::NoneRoughnessMetallicTexture) {
    result &= ~Flag::RoughnessTexture;
    result &= ~Flag::MetallicTexture;
  }

  return result;
}

PbrShader::PbrShader(Flags originalFlags, unsigned int lightCount)
    : flags_(generateCorrectFlags(originalFlags)), lightCount_(lightCount) {
  CORRADE_ASSERT(
      lightCount_ > 0,
      "PbrShader::PbrShader(): light count must be larger than 1.", );
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
  attributeLocationsStream << Cr::Utility::formatString(
      "#define ATTRIBUTE_LOCATION_NORMAL {}\n", Normal::Location);
  if (flags_ & (Flag::NormalTexture | Flag::PrecomputedTangent) && lightCount) {
    attributeLocationsStream << Cr::Utility::formatString(
        "#define ATTRIBUTE_LOCATION_TANGENT4 {}\n", Tangent4::Location);
  }
  const bool isTextured = bool(
      flags_ & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                Flag::MetallicTexture | Flag::NormalTexture |
                Flag::EmissiveTexture | Flag::NoneRoughnessMetallicTexture |
                Flag::OcclusionRoughnessMetallicTexture));

  if (isTextured) {
    attributeLocationsStream
        << Cr::Utility::formatString("#define ATTRIBUTE_LOCATION_TEXCOORD {}\n",
                                     TextureCoordinates::Location);
  }

  // Add macros
  vert.addSource(attributeLocationsStream.str())
      .addSource(isTextured ? "#define TEXTURED\n" : "")
      .addSource(flags_ & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags_ & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(flags_ & Flag::TextureTransformation
                     ? "#define TEXTURE_TRANSFORMATION\n"
                     : "")
      .addSource(rs.get("pbr.vert"));

  std::stringstream outputAttributeLocationsStream;
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput);
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID {}\n", ObjectIdOutput);

  frag.addSource(attributeLocationsStream.str())
      .addSource(outputAttributeLocationsStream.str())
      .addSource(isTextured ? "#define TEXTURED\n" : "")
      .addSource(flags_ & Flag::BaseColorTexture ? "#define BASECOLOR_TEXTURE\n"
                                                 : "")
      .addSource(flags_ & Flag::EmissiveTexture ? "#define EMISSIVE_TEXTURE\n"
                                                : "")
      .addSource(flags_ & Flag::RoughnessTexture ? "#define ROUGHNESS_TEXTURE\n"
                                                 : "")
      .addSource(flags_ & Flag::MetallicTexture ? "#define METALLIC_TEXTURE\n"
                                                : "")
      .addSource(flags_ & Flag::NoneRoughnessMetallicTexture
                     ? "#define NONE_ROUGHNESS_METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags_ & Flag::OcclusionRoughnessMetallicTexture
                     ? "#define OCCLUSION_ROUGHNESS_METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags_ & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags_ & Flag::NormalTextureScale
                     ? "#define NORMAL_TEXTURE_SCALE\n"
                     : "")
      .addSource(flags_ & Flag::ObjectId ? "#define OBJECT_ID\n" : "")
      .addSource(flags_ & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(
          Cr::Utility::formatString("#define LIGHT_COUNT {}\n", lightCount))
      .addSource(rs.get("pbr.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // bind attributes
#ifndef MAGNUM_TARGET_GLES
  if (!Mn::GL::Context::current()
           .isExtensionSupported<
               Mn::GL::Extensions::ARB::explicit_attrib_location>(glVersion))
#endif
  {
    bindAttributeLocation(Position::Location, "vertexPosition");
    if (lightCount) {
      bindAttributeLocation(Normal::Location, "vertexNormal");
    }
    if (flags_ & (Flag::NormalTexture | Flag::PrecomputedTangent) &&
        lightCount) {
      bindAttributeLocation(Tangent4::Location, "vertexTangent");
    }
    if (isTextured) {
      bindAttributeLocation(TextureCoordinates::Location, "vertexTexCoord");
    }
  }

  // set texture binding points in the shader;
  // see PBR vertex, fragment shader code for details
  if (flags_ & Flag::BaseColorTexture) {
    setUniform(uniformLocation("BaseColorTexture"), TextureUnit::BaseColor);
  }
  if (flags_ & Flag::RoughnessTexture) {
    setUniform(uniformLocation("RoughnessTexture"), TextureUnit::Roughness);
  }
  if (flags_ & Flag::MetallicTexture) {
    setUniform(uniformLocation("MetallicTexture"), TextureUnit::Metallic);
  }
  if (flags_ & Flag::NormalTexture) {
    setUniform(uniformLocation("NormalTexture"), TextureUnit::Normal);
  }
  if (flags_ & Flag::EmissiveTexture) {
    setUniform(uniformLocation("EmissiveTexture"), TextureUnit::Emissive);
  }
  if ((flags_ & Flag::NoneRoughnessMetallicTexture) ||
      (flags_ & Flag::OcclusionRoughnessMetallicTexture)) {
    setUniform(uniformLocation("PackedTexture"), TextureUnit::Packed);
  }

  // cache the uniform locations
  modelviewMatrixUniform_ = uniformLocation("ModelViewMatrix");
  normalMatrixUniform_ = uniformLocation("NormalMatrix");
  projMatrixUniform_ = uniformLocation("ProjectionMatrix");
  if (flags_ & Flag::ObjectId) {
    objectIdUniform_ = uniformLocation("ObjectId");
  }
  if (flags_ & Flag::TextureTransformation) {
    textureMatrixUniform_ = uniformLocation("TextureMatrix");
  }

  // materials
  baseColorUniform_ = uniformLocation("Material.baseColor");
  roughnessUniform_ = uniformLocation("Material.roughness");
  metallicUniform_ = uniformLocation("Material.metallic");
  emissiveColorUniform_ = uniformLocation("Material.emissiveColor");

  // lights
  lightRangesUniform_ = uniformLocation("LightRanges");
  lightColorsUniform_ = uniformLocation("LightColors");
  lightDirectionsUniform_ = uniformLocation("LightDirections");

  if (flags_ & Flag::NormalTexture && flags_ & Flag::NormalTextureScale) {
    normalTextureScaleUniform_ = uniformLocation("NormalTextureScale");
  }
}

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PbrShader& PbrShader::bindBaseColorTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::BaseColorTexture,
      "Shaders::PbrShader::bindBaseColorTexture(): the shader was not "
      "created with base color texture enabled",
      *this);
  texture.bind(TextureUnit::BaseColor);
  return *this;
}

PbrShader& PbrShader::bindRoughnessTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::RoughnessTexture,
      "Shaders::PbrShader::bindRoughnessTexture(): the shader was not "
      "created with independent roughness texture enabled. Are you using "
      "packed texture? E.g., OcclusionRoughnessMetallic texture? ",
      *this);
  texture.bind(TextureUnit::Roughness);
  return *this;
}

PbrShader& PbrShader::bindMetallicTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::MetallicTexture,
      "Shaders::PbrShader::bindMetallicTexture(): the shader was not "
      "created with independent metallic texture enabled. Are you using packed "
      "texture? E.g., OcclusionRoughnessMetallic texture? ",
      *this);
  texture.bind(TextureUnit::Metallic);
  return *this;
}

PbrShader& PbrShader::bindNormalTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::NormalTexture,
                 "Shaders::PbrShader::bindNormalTexture(): the shader was not "
                 "created with normal texture enabled",
                 *this);
  texture.bind(TextureUnit::Normal);
  return *this;
}

PbrShader& PbrShader::bindNoneRoughnessMetallicTexture(
    Magnum::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::NoneRoughnessMetallicTexture,
                 "Shaders::PbrShader::bindNoneRoughnessMetallicTexture(): the "
                 "shader was not "
                 "created with NoneRoughnessMetallic texture enabled",
                 *this);
  texture.bind(TextureUnit::Packed);
  return *this;
}

PbrShader& PbrShader::bindOcclusionRoughnessMetallicTexture(
    Magnum::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::OcclusionRoughnessMetallicTexture,
      "Shaders::PbrShader::bindOcclusionRoughnessMetallicTexture(): the "
      "shader was not "
      "created with OcclusionRoughnessMetallic texture enabled",
      *this);
  texture.bind(TextureUnit::Packed);
  return *this;
}

PbrShader& PbrShader::bindEmissiveTexture(Magnum::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & Flag::EmissiveTexture,
      "Shaders::PbrShader::bindEmissiveTexture(): the shader was not "
      "created with emissive texture enabled",
      *this);
  texture.bind(TextureUnit::Emissive);
  return *this;
}

PbrShader& PbrShader::setProjectionMatrix(const Mn::Matrix4& matrix) {
  setUniform(projMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setNormalMatrix(const Mn::Matrix3x3& matrix) {
  setUniform(normalMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setTransformationMatrix(const Mn::Matrix4& matrix) {
  setUniform(modelviewMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setObjectId(unsigned int objectId) {
  if (flags_ & Flag::ObjectId) {
    setUniform(objectIdUniform_, objectId);
  }
  return *this;
}

PbrShader& PbrShader::setBaseColor(const Mn::Color4& color) {
  setUniform(baseColorUniform_, color);
  return *this;
}

PbrShader& PbrShader::setEmissiveColor(const Magnum::Color3& color) {
  setUniform(emissiveColorUniform_, color);
  return *this;
}

PbrShader& PbrShader::setRoughness(float roughness) {
  setUniform(roughnessUniform_, roughness);
  return *this;
}

PbrShader& PbrShader::setMetallic(float metallic) {
  setUniform(metallicUniform_, metallic);
  return *this;
}

PbrShader& PbrShader::setTextureMatrix(const Mn::Matrix3& matrix) {
  CORRADE_ASSERT(flags_ & Flag::TextureTransformation,
                 "PbrShader::setTextureMatrix(): the shader was not "
                 "created with texture transformation enabled",
                 *this);
  setUniform(textureMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setLightVectors(
    Cr::Containers::ArrayView<const Mn::Vector4> vectors) {
  CORRADE_ASSERT(lightCount_ == vectors.size(),
                 "PbrShader::setLightVectors(): expected"
                     << lightCount_ << "items but got" << vectors.size(),
                 *this);
  setUniform(lightDirectionsUniform_, vectors);
  return *this;
}

PbrShader& PbrShader::setLightVectors(
    std::initializer_list<Mn::Vector4> vectors) {
  return setLightVectors(Cr::Containers::arrayView(vectors));
}

PbrShader& PbrShader::setLightPosition(unsigned int lightIndex,
                                       const Mn::Vector3& pos) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightPosition: lightIndex" << lightIndex << "is illegal.",
      *this);

  setUniform(lightDirectionsUniform_ + lightIndex, Mn::Vector4{pos, 1.0});
  return *this;
}

PbrShader& PbrShader::setLightDirection(unsigned int lightIndex,
                                        const Mn::Vector3& dir) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightDirection: lightIndex" << lightIndex << "is illegal.",
      *this);
  setUniform(lightDirectionsUniform_ + lightIndex, Mn::Vector4{dir, 0.0});
  return *this;
}

PbrShader& PbrShader::setLightVector(unsigned int lightIndex,
                                     const Mn::Vector4& vec) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightVector: lightIndex" << lightIndex << "is illegal.",
      *this);

  CORRADE_ASSERT(vec.w() == 1 || vec.w() == 0,
                 "PbrShader::setLightVector"
                     << vec
                     << "is expected to have w == 0 for a directional light or "
                        "w == 1 for a point light",
                 *this);

  setUniform(lightDirectionsUniform_ + lightIndex, vec);

  return *this;
}

PbrShader& PbrShader::setLightRange(unsigned int lightIndex, float range) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightRange: lightIndex" << lightIndex << "is illegal.",
      *this);
  setUniform(lightRangesUniform_ + lightIndex, range);
  return *this;
}
PbrShader& PbrShader::setLightColor(unsigned int lightIndex,
                                    const Mn::Vector3& color,
                                    float intensity) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightColor: lightIndex" << lightIndex << "is illegal.",
      *this);
  Mn::Vector3 finalColor = intensity * color;
  setUniform(lightColorsUniform_ + lightIndex, finalColor);
  return *this;
}

PbrShader& PbrShader::setLightColors(
    Cr::Containers::ArrayView<const Mn::Color3> colors) {
  CORRADE_ASSERT(lightCount_ == colors.size(),
                 "PbrShader::setLightColors(): expected"
                     << lightCount_ << "items but got" << colors.size(),
                 *this);

  setUniform(lightColorsUniform_, colors);
  return *this;
}

PbrShader& PbrShader::setLightColors(std::initializer_list<Mn::Color3> colors) {
  return setLightColors(Cr::Containers::arrayView(colors));
}

PbrShader& PbrShader::setNormalTextureScale(float scale) {
  CORRADE_ASSERT(flags_ & Flag::NormalTexture,
                 "PbrShader::setNormalTextureScale(): the shader was not "
                 "created with normal texture enabled",
                 *this);
  if (flags_ & Flag::NormalTextureScale) {
    setUniform(normalTextureScaleUniform_, scale);
  }
  return *this;
}

PbrShader& PbrShader::setLightRanges(
    Corrade::Containers::ArrayView<const float> ranges) {
  CORRADE_ASSERT(lightCount_ == ranges.size(),
                 "PbrShader::setLightRanges(): expected"
                     << lightCount_ << "items but got" << ranges.size(),
                 *this);

  setUniform(lightRangesUniform_, ranges);
  return *this;
}

PbrShader& PbrShader::setLightRanges(std::initializer_list<float> ranges) {
  return setLightRanges(Cr::Containers::arrayView(ranges));
}

}  // namespace gfx
}  // namespace esp
