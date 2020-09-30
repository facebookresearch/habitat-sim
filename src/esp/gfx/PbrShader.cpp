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
enum TextureBindingPointIndex : uint8_t {
  BaseColor = 0,
  Roughness = 1,
  Metallic = 2,
  Normal = 3,
};
}  // namespace

PbrShader::PbrShader(Flags flags, unsigned int lightCount)
    : flags_(flags), lightCount_(lightCount) {
  if (!Cr::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  // this is not the file name, but the group name in the config file
  // see Shaders.conf in the shaders folder
  const Cr::Utility::Resource rs{"default-shaders"};

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  std::stringstream attributeLocationsStream;
  attributeLocationsStream << Cr::Utility::formatString(
      "#define ATTRIBUTE_LOCATION_POSITION {}\n", Position::Location);
  if (lightCount) {
    attributeLocationsStream << Cr::Utility::formatString(
        "#define ATTRIBUTE_LOCATION_NORMAL {}\n", Normal::Location);
  }
  if (flags & (Flag::NormalTexture | Flag::PrecomputedTangent) && lightCount) {
    attributeLocationsStream << Cr::Utility::formatString(
        "#define ATTRIBUTE_LOCATION_TANGENT4 {}\n", Tangent4::Location);
  }
  if (flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
               Flag::MetallicTexture | Flag::NoneRoughnessMetallicTexture |
               Flag::OcclusionRoughnessMetallicTexture)) {
    attributeLocationsStream
        << Cr::Utility::formatString("#define ATTRIBUTE_LOCATION_TEXCOORD {}\n",
                                     TextureCoordinates::Location);
  }

  // Add macros
  vert.addSource(attributeLocationsStream.str())
      .addSource(flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                          Flag::MetallicTexture | Flag::NormalTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(rs.get("pbr.vert"));

  frag.addSource(attributeLocationsStream.str())
      .addSource(flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                          Flag::MetallicTexture | Flag::NormalTexture |
                          Flag::NoneRoughnessMetallicTexture |
                          Flag::OcclusionRoughnessMetallicTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::BaseColorTexture ? "#define BASECOLOR_TEXTURE\n"
                                                : "")
      .addSource(((flags & Flag::RoughnessTexture) &&
                  !(flags & Flag::NoneRoughnessMetallicTexture) &&
                  !(flags & Flag::OcclusionRoughnessMetallicTexture))
                     ? "#define ROUGHNESS_TEXTURE\n"
                     : "")
      .addSource(((flags & Flag::MetallicTexture) &&
                  !(flags & Flag::NoneRoughnessMetallicTexture) &&
                  !(flags & Flag::OcclusionRoughnessMetallicTexture))
                     ? "#define METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags & Flag::NoneRoughnessMetallicTexture
                     ? "#define NON_ROUGHNESS_METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags & Flag::OcclusionRoughnessMetallicTexture
                     ? "#define OCCLUSION_ROUGHNESS_METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::NormalTextureScale
                     ? "#define NORMAL_TEXTURE_SCALE\n"
                     : "")
      .addSource(flags & Flag::ObjectId ? "#define OBJECT_ID\n" : "")
      .addSource(flags & Flag::PrecomputedTangent
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
    if (flags & (Flag::NormalTexture | Flag::PrecomputedTangent) &&
        lightCount) {
      bindAttributeLocation(Tangent4::Location, "vertexTangent");
    }
    if (flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                 Flag::MetallicTexture | Flag::NoneRoughnessMetallicTexture |
                 Flag::OcclusionRoughnessMetallicTexture)) {
      bindAttributeLocation(TextureCoordinates::Location, "vertexTexCoord");
    }
  }

  // set texture binding points in the shader;
  // see PBR vertex, fragment shader code for details
  if (flags & Flag::BaseColorTexture) {
    setUniform(uniformLocation("baseColorTexture"),
               TextureBindingPointIndex::BaseColor);
  }
  if (flags & Flag::RoughnessTexture) {
    setUniform(uniformLocation("RoughnessTexture"),
               TextureBindingPointIndex::Roughness);
  }
  if (flags & Flag::MetallicTexture) {
    setUniform(uniformLocation("MetallicTexture"),
               TextureBindingPointIndex::Metallic);
  }
  if (flags & Flag::NormalTexture) {
    setUniform(uniformLocation("NormalTexture"),
               TextureBindingPointIndex::Normal);
  }

  // cache the uniform locations
  modelviewMatrixUniform_ = uniformLocation("ModelViewMatrix");
  normalMatrixUniform_ = uniformLocation("NormalMatrix");
  mvpMatrixUniform_ = uniformLocation("MVP");
  if (flags & Flag::ObjectId) {
    objectIdUniform_ = uniformLocation("objectId");
  }
  if (flags & Flag::TextureTransformation) {
    textureMatrixUniform_ = uniformLocation("textureMatrix");
  }

  // materials
  baseColorUniform_ = uniformLocation("Material.baseColor");
  roughnessUniform_ = uniformLocation("Material.roughness");
  metallicUniform_ = uniformLocation("Material.metallic");

  // lights
  if (lightCount_) {
    lightRangesUniform_ = uniformLocation("LightRanges");
    lightColorsUniform_ = uniformLocation("LightColors");
    lightDirectionsUniform_ = uniformLocation("LightDirections");

    if (flags & Flag::NormalTexture) {
      normalTextureScaleUniform_ = uniformLocation("normalTextureScale");
    }
  }
}

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PbrShader& PbrShader::bindBaseColorTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::BaseColorTexture,
                 "PbrShader::bindBaseColorTexture: the shader was not "
                 "created with BaseColor texture enabled",
                 *this);
  if (lightCount_) {
    texture.bind(TextureBindingPointIndex::BaseColor);
  }
  return *this;
}

PbrShader& PbrShader::bindRoughnessTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::RoughnessTexture,
                 "PbrShader::bindRoughnessTexture: the shader was not "
                 "created with roughness texture enabled",
                 *this);
  if (lightCount_) {
    texture.bind(TextureBindingPointIndex::Roughness);
  }
  return *this;
}

PbrShader& PbrShader::bindMetallicTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::MetallicTexture,
                 "PbrShader::bindMetallicTexture: the shader was not "
                 "created with metallic texture enabled",
                 *this);
  if (lightCount_) {
    texture.bind(TextureBindingPointIndex::Metallic);
  }
  return *this;
}

PbrShader& PbrShader::bindNormalTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::NormalTexture,
                 "PbrShader::bindNormalTexture: the shader was not "
                 "created with normal texture enabled",
                 *this);
  if (lightCount_) {
    texture.bind(TextureBindingPointIndex::Normal);
  }
  return *this;
}

PbrShader& PbrShader::setMVPMatrix(const Mn::Matrix4& matrix) {
  setUniform(mvpMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setNormalMatrix(const Mn::Matrix3& matrix) {
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
  if (lightCount_) {
    setUniform(baseColorUniform_, color);
  }
  return *this;
}

PbrShader& PbrShader::setRoughness(float roughness) {
  if (lightCount_) {
    setUniform(roughnessUniform_, roughness);
  }
  return *this;
}

PbrShader& PbrShader::setMetallic(float metallic) {
  if (lightCount_) {
    setUniform(metallicUniform_, metallic);
  }
  return *this;
}

PbrShader& PbrShader::bindTextures(Mn::GL::Texture2D* BaseColor,
                                   Mn::GL::Texture2D* roughness,
                                   Mn::GL::Texture2D* metallic,
                                   Mn::GL::Texture2D* normal) {
  if (BaseColor) {
    bindBaseColorTexture(*BaseColor);
  }
  if (roughness) {
    bindRoughnessTexture(*roughness);
  }
  if (metallic) {
    bindMetallicTexture(*metallic);
  }
  if (normal) {
    bindNormalTexture(*normal);
  }
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
  if (lightCount_) {
    setUniform(lightDirectionsUniform_, vectors);
  }
  return *this;
}

PbrShader& PbrShader::setLightVectors(
    std::initializer_list<Mn::Vector4> vectors) {
  return setLightVectors(Cr::Containers::arrayView(vectors));
  return *this;
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
  if (lightCount_) {
    setUniform(lightRangesUniform_ + lightIndex, range);
  }
  return *this;
}
PbrShader& PbrShader::setLightColor(unsigned int lightIndex,
                                    const Mn::Vector3& color,
                                    float intensity) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightColor: lightIndex" << lightIndex << "is illegal.",
      *this);
  if (lightCount_) {
    Mn::Vector3 finalColor = intensity * color;
    setUniform(lightColorsUniform_ + lightIndex, finalColor);
  }
  return *this;
}

PbrShader& PbrShader::setLightColors(
    Cr::Containers::ArrayView<const Mn::Color3> colors) {
  CORRADE_ASSERT(lightCount_ == colors.size(),
                 "PbrShader::setLightColors(): expected"
                     << lightCount_ << "items but got" << colors.size(),
                 *this);

  if (lightCount_) {
    setUniform(lightColorsUniform_, colors);
  }
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
  if (lightCount_ && flags_ & Flag::NormalTextureScale) {
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

  if (lightCount_) {
    setUniform(lightRangesUniform_, ranges);
  }
  return *this;
}

PbrShader& PbrShader::setLightRanges(std::initializer_list<float> ranges) {
  return setLightRanges(Cr::Containers::arrayView(ranges));
}

}  // namespace gfx
}  // namespace esp
