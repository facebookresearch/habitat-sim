// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrShader.h"
#include "PbrTextureUnit.h"

#include <Corrade/Containers/Array.h>
#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/CubeMapTexture.h>
#include <Magnum/GL/Extensions.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/PixelFormat.h>

#include "esp/core/Esp.h"

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

inline bool PbrShader::lightingIsEnabled() const {
  return (lightCount_ != 0u || flags_ & Flag::ImageBasedLighting);
}

PbrShader::PbrShader(Flags originalFlags, unsigned int lightCount)
    : flags_(originalFlags), lightCount_(lightCount) {
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
  if ((flags_ & Flag::NormalTexture) && (flags_ & Flag::PrecomputedTangent) &&
      lightingIsEnabled()) {
    attributeLocationsStream << Cr::Utility::formatString(
        "#define ATTRIBUTE_LOCATION_TANGENT4 {}\n", Tangent4::Location);
  }
  // TODO: Occlusion texture to be added.
  const bool isTextured =
      bool(flags_ & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                     Flag::MetallicTexture | Flag::NormalTexture |
                     Flag::EmissiveTexture));

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

  frag.addSource(outputAttributeLocationsStream.str())
      .addSource(flags_ & Flag::ShadowsVSM ? "#define SHADOWS_VSM\n" : "")
      .addSource(isTextured ? "#define TEXTURED\n" : "")
      .addSource(flags_ & Flag::BaseColorTexture ? "#define BASECOLOR_TEXTURE\n"
                                                 : "")
      .addSource(flags_ & Flag::EmissiveTexture ? "#define EMISSIVE_TEXTURE\n"
                                                : "")
      .addSource(flags_ & Flag::RoughnessTexture ? "#define ROUGHNESS_TEXTURE\n"
                                                 : "")
      .addSource(flags_ & Flag::MetallicTexture ? "#define METALLIC_TEXTURE\n"
                                                : "")
      .addSource(flags_ & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags_ & Flag::NormalTextureScale
                     ? "#define NORMAL_TEXTURE_SCALE\n"
                     : "")
      .addSource(flags_ & Flag::ObjectId ? "#define OBJECT_ID\n" : "")
      .addSource(flags_ & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(flags_ & Flag::ImageBasedLighting
                     ? "#define IMAGE_BASED_LIGHTING\n"
                     : "")
      .addSource(flags_ & Flag::ImageBasedLighting ? "#define TONE_MAP\n" : "")
      .addSource(flags_ & Flag::DebugDisplay ? "#define PBR_DEBUG_DISPLAY\n"
                                             : "")
      .addSource(
          Cr::Utility::formatString("#define LIGHT_COUNT {}\n", lightCount_))
      .addSource(flags_ & Flag::ShadowsVSM ? rs.get("shadowsVSM.glsl") + "\n"
                                           : "")
      .addSource(rs.get("pbrCommon.glsl") + "\n")
      .addSource(rs.get("pbr.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, frag}));

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // set texture binding points in the shader;
  // see PBR vertex, fragment shader code for details
  if (lightingIsEnabled()) {
    if (flags_ & Flag::BaseColorTexture) {
      setUniform(uniformLocation("BaseColorTexture"),
                 pbrTextureUnitSpace::TextureUnit::BaseColor);
    }
    if (flags_ & (Flag::RoughnessTexture | Flag::MetallicTexture)) {
      setUniform(uniformLocation("MetallicRoughnessTexture"),
                 pbrTextureUnitSpace::TextureUnit::MetallicRoughness);
    }
    // TODO: explore the normal mapping without the precomputer tangent.
    // see http://www.thetenthplanet.de/archives/1180
    // also:
    // https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/data/shaders/pbr_khr.frag
    if ((flags_ & Flag::NormalTexture) && (flags_ & Flag::PrecomputedTangent)) {
      setUniform(uniformLocation("NormalTexture"),
                 pbrTextureUnitSpace::TextureUnit::Normal);
    }
    // TODO occlusion texture
  }
  // emissive texture does not depend on lights
  if (flags_ & Flag::EmissiveTexture) {
    setUniform(uniformLocation("EmissiveTexture"),
               pbrTextureUnitSpace::TextureUnit::Emissive);
  }

  // IBL related textures
  if (flags_ & Flag::ImageBasedLighting) {
    setUniform(uniformLocation("IrradianceMap"),
               pbrTextureUnitSpace::TextureUnit::IrradianceMap);
    setUniform(uniformLocation("BrdfLUT"),
               pbrTextureUnitSpace::TextureUnit::BrdfLUT);
    setUniform(uniformLocation("PrefilteredMap"),
               pbrTextureUnitSpace::TextureUnit::PrefilteredMap);
  }

  // VSM shadows
  if (flags_ & Flag::ShadowsVSM) {
    setUniform(uniformLocation("ShadowMap[0]"),
               pbrTextureUnitSpace::TextureUnit::ShadowMap0);
    setUniform(uniformLocation("ShadowMap[1]"),
               pbrTextureUnitSpace::TextureUnit::ShadowMap1);
    setUniform(uniformLocation("ShadowMap[2]"),
               pbrTextureUnitSpace::TextureUnit::ShadowMap2);
  }

  // cache the uniform locations
  viewMatrixUniform_ = uniformLocation("ViewMatrix");
  modelMatrixUniform_ = uniformLocation("ModelMatrix");
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
  if (lightCount_ != 0u) {
    lightRangesUniform_ = uniformLocation("LightRanges");
    lightColorsUniform_ = uniformLocation("LightColors");
    lightDirectionsUniform_ = uniformLocation("LightDirections");
  }

  if ((flags_ & Flag::NormalTexture) && (flags_ & Flag::NormalTextureScale) &&
      lightingIsEnabled()) {
    normalTextureScaleUniform_ = uniformLocation("NormalTextureScale");
  }

  cameraWorldPosUniform_ = uniformLocation("CameraWorldPos");

  // IBL related uniform
  if (flags_ & Flag::ImageBasedLighting) {
    prefilteredMapMipLevelsUniform_ =
        uniformLocation("PrefilteredMapMipLevels");
  }

  // pbr equation scales
  componentScalesUniform_ = uniformLocation("ComponentScales");

  // for debug info
  if (flags_ & Flag::DebugDisplay) {
    pbrDebugDisplayUniform_ = uniformLocation("PbrDebugDisplay");
  }

  // initialize the shader with some "reasonable defaults"
  setViewMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  setModelMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  setProjectionMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  if (lightingIsEnabled()) {
    setBaseColor(Magnum::Color4{0.7f});
    setRoughness(0.9f);
    setMetallic(0.1f);
    if (flags_ & Flag::NormalTexture) {
      setNormalTextureScale(1.0f);
    }
    setNormalMatrix(Mn::Matrix3x3{Mn::Math::IdentityInit});
  }

  if (lightCount_ != 0u) {
    setLightVectors(Cr::Containers::Array<Mn::Vector4>{
        Cr::DirectInit, lightCount_,
        // a single directional "fill" light, coming from the center of the
        // camera.
        Mn::Vector4{0.0f, 0.0f, 1.0f, 0.0f}});
    Cr::Containers::Array<Mn::Color3> colors{Cr::DirectInit, lightCount_,
                                             Mn::Color3{1.0f}};
    setLightColors(colors);
    setLightRanges(Cr::Containers::Array<Mn::Float>{Cr::DirectInit, lightCount_,
                                                    Mn::Constants::inf()});
  }

  setEmissiveColor(Magnum::Color3{0.0f});
  PbrShader::PbrEquationScales scales;
  if (flags_ & Flag::ImageBasedLighting) {
    // These are empirical numbers. Discount the diffuse light from IBL so the
    // ambient light will not be too strong. Also keeping the IBL specular
    // component relatively low can guarantee the super glossy surface would not
    // reflect the environment like a mirror.
    scales.iblDiffuse = 0.8;
    scales.iblSpecular = 0.3;
  }
  setPbrEquationScales(scales);
  if (flags_ & Flag::DebugDisplay) {
    setDebugDisplay(PbrDebugDisplay::None);
  }
}

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PbrShader& PbrShader::bindBaseColorTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::BaseColorTexture,
                 "PbrShader::bindBaseColorTexture(): the shader was not "
                 "created with base color texture enabled",
                 *this);
  if (lightingIsEnabled()) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::BaseColor);
  }
  return *this;
}

PbrShader& PbrShader::bindMetallicRoughnessTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ & (Flag::RoughnessTexture | Flag::MetallicTexture),
      "PbrShader::bindMetallicRoughnessTexture(): the shader was not "
      "created with metallicRoughness texture enabled.",
      *this);
  if (lightingIsEnabled()) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::MetallicRoughness);
  }
  return *this;
}

PbrShader& PbrShader::bindNormalTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::NormalTexture,
                 "PbrShader::bindNormalTexture(): the shader was not "
                 "created with normal texture enabled",
                 *this);
  if (lightingIsEnabled()) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::Normal);
  }
  return *this;
}

PbrShader& PbrShader::bindEmissiveTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::EmissiveTexture,
                 "PbrShader::bindEmissiveTexture(): the shader was not "
                 "created with emissive texture enabled",
                 *this);
  // emissive texture does not depend on lights
  texture.bind(pbrTextureUnitSpace::TextureUnit::Emissive);
  return *this;
}

PbrShader& PbrShader::bindIrradianceCubeMap(Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & Flag::ImageBasedLighting,
                 "PbrShader::bindIrradianceCubeMap(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::IrradianceMap);
  return *this;
}

PbrShader& PbrShader::bindBrdfLUT(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ & Flag::ImageBasedLighting,
                 "PbrShader::bindBrdfLUT(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::BrdfLUT);
  return *this;
}

PbrShader& PbrShader::bindPrefilteredMap(Magnum::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ & Flag::ImageBasedLighting,
                 "PbrShader::bindPrefilteredMap(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::PrefilteredMap);
  return *this;
}

PbrShader& PbrShader::bindPointShadowMap(int index,
                                         Magnum::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(
      index >= 0 && index < 3,
      "PbrShader::bindPointShadowMap(): the texture index was illegal.", *this);
  CORRADE_ASSERT(flags_ & Flag::ShadowsVSM,
                 "PbrShader::bindPointShadowMap(): the shader was not "
                 "created with shadows enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::ShadowMap0 + index);
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

PbrShader& PbrShader::setViewMatrix(const Mn::Matrix4& matrix) {
  setUniform(viewMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setModelMatrix(const Mn::Matrix4& matrix) {
  setUniform(modelMatrixUniform_, matrix);
  return *this;
}

PbrShader& PbrShader::setObjectId(unsigned int objectId) {
  if (flags_ & Flag::ObjectId) {
    setUniform(objectIdUniform_, objectId);
  }
  return *this;
}

PbrShader& PbrShader::setPrefilteredMapMipLevels(unsigned int mipLevels) {
  CORRADE_ASSERT(flags_ & Flag::ImageBasedLighting,
                 "PbrShader::setPrefilteredMapMipLevels(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  setUniform(prefilteredMapMipLevelsUniform_, mipLevels);
  return *this;
}

PbrShader& PbrShader::setBaseColor(const Mn::Color4& color) {
  if (lightingIsEnabled()) {
    setUniform(baseColorUniform_, color);
  }
  return *this;
}

PbrShader& PbrShader::setEmissiveColor(const Magnum::Color3& color) {
  setUniform(emissiveColorUniform_, color);
  return *this;
}

PbrShader& PbrShader::setRoughness(float roughness) {
  if (lightingIsEnabled()) {
    setUniform(roughnessUniform_, roughness);
  }
  return *this;
}

PbrShader& PbrShader::setMetallic(float metallic) {
  if (lightingIsEnabled()) {
    setUniform(metallicUniform_, metallic);
  }
  return *this;
}

PbrShader& PbrShader::setPbrEquationScales(const PbrEquationScales& scales) {
  Mn::Vector4 componentScales{scales.directDiffuse, scales.directSpecular,
                              scales.iblDiffuse, scales.iblSpecular};
  setUniform(componentScalesUniform_, componentScales);
  return *this;
}

PbrShader& PbrShader::setDebugDisplay(PbrDebugDisplay index) {
  CORRADE_ASSERT(flags_ & Flag::DebugDisplay,
                 "PbrShader::setDebugDisplay(): the shader was not "
                 "created with DebugDisplay enabled",
                 *this);
  setUniform(pbrDebugDisplayUniform_, int(index));
  return *this;
}

PbrShader& PbrShader::setCameraWorldPosition(
    const Magnum::Vector3& cameraWorldPos) {
  setUniform(cameraWorldPosUniform_, cameraWorldPos);
  return *this;
}

PbrShader& PbrShader::setTextureMatrix(const Mn::Matrix3& matrix) {
  CORRADE_ASSERT(flags_ & Flag::TextureTransformation,
                 "PbrShader::setTextureMatrix(): the shader was not "
                 "created with texture transformation enabled",
                 *this);

  // since emissive texture may need it, so no if (lightCount_) here
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
  if ((flags_ & Flag::NormalTextureScale) && lightingIsEnabled()) {
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
