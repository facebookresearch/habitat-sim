// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

#include <sstream>

// This is to import the "resources" at runtime. When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(GfxShaderResources)
}

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

PbrShader::PbrShader(Flags originalFlags, unsigned int lightCount)
    : flags_(originalFlags), lightCount_(lightCount) {
  if (!Cr::Utility::Resource::hasGroup("gfx-shaders")) {
    importShaderResources();
  }

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif
  directLightingIsEnabled_ =
      ((lightCount_ != 0u) && flags_ >= Flag::DirectLighting);
  lightingIsEnabled_ =
      (directLightingIsEnabled_ || flags_ >= Flag::ImageBasedLighting);
  directAndIBLisEnabled_ =
      (directLightingIsEnabled_ && flags_ >= Flag::ImageBasedLighting);

  bool mapMatInToLinear = (flags_ >= Flag::MapMatTxtrToLinear &&
                           ((flags_ >= Flag::BaseColorTexture) ||
                            (flags_ >= Flag::EmissiveTexture) ||
                            (flags_ >= Flag::SpecularLayerColorTexture)));
  bool mapIBLInToLinear = ((flags_ >= Flag::ImageBasedLighting) &&
                           (flags_ >= Flag::MapIBLTxtrToLinear));

  mapInputToLinear_ = (mapMatInToLinear || mapIBLInToLinear);

  isTextured_ =
      (((flags_ >= Flag::BaseColorTexture) ||
        (flags_ >= Flag::NoneRoughnessMetallicTexture) ||
        (flags_ >= Flag::NormalTexture) || (flags_ >= Flag::EmissiveTexture)) ||
       // clear coat
       ((flags_ >= Flag::ClearCoatTexture) ||
        (flags_ >= Flag::ClearCoatRoughnessTexture) ||
        (flags_ >= Flag::ClearCoatNormalTexture)) ||
       // specular layer
       ((flags_ >= Flag::SpecularLayerTexture) ||
        (flags_ >= Flag::SpecularLayerColorTexture)) ||
       // anisotropy - always needs texCoords to get tangentspace map
       (flags_ >= Flag::AnisotropyLayer) ||
       // transmission layer
       (flags_ >= Flag::TransmissionLayerTexture) ||
       // volume layer
       (flags_ >= Flag::VolumeLayerThicknessTexture));

  // this is not the file name, but the group name in the config file
  // see Shaders.conf in the shaders folder
  const Cr::Utility::Resource rs{"gfx-shaders"};

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  std::stringstream attributeLocationsStream;
  attributeLocationsStream << Cr::Utility::formatString(
      "#define ATTRIBUTE_LOCATION_POSITION {}\n", Position::Location);
  attributeLocationsStream << Cr::Utility::formatString(
      "#define ATTRIBUTE_LOCATION_NORMAL {}\n", Normal::Location);
  if ((flags_ >= Flag::NormalTexture) && (flags_ >= Flag::PrecomputedTangent) &&
      lightingIsEnabled_) {
    attributeLocationsStream << Cr::Utility::formatString(
        "#define ATTRIBUTE_LOCATION_TANGENT4 {}\n", Tangent4::Location);
  }
  // TODO: Occlusion texture to be added.

  if (isTextured_) {
    attributeLocationsStream
        << Cr::Utility::formatString("#define ATTRIBUTE_LOCATION_TEXCOORD {}\n",
                                     TextureCoordinates::Location);
  }

  // Add macros
  vert.addSource(attributeLocationsStream.str())
      .addSource(isTextured_ ? "#define TEXTURED\n" : "")
      .addSource(flags_ >= Flag::NormalTexture ? "#define NORMAL_TEXTURE\n"
                                               : "")
      .addSource(flags_ >= Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(isTextured_ && (flags_ >= Flag::TextureTransformation)
                     ? "#define TEXTURE_TRANSFORMATION\n"
                     : "")
      .addSource(rs.getString("pbr.vert"));

  std::stringstream outputAttributeLocationsStream;
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_COLOR {}\n", ColorOutput);
  outputAttributeLocationsStream << Cr::Utility::formatString(
      "#define OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID {}\n", ObjectIdOutput);

  frag.addSource(outputAttributeLocationsStream.str())
      .addSource(isTextured_ ? "#define TEXTURED\n" : "")
      .addSource(
          flags_ >= Flag::BaseColorTexture ? "#define BASECOLOR_TEXTURE\n" : "")
      .addSource(flags_ >= Flag::EmissiveTexture ? "#define EMISSIVE_TEXTURE\n"
                                                 : "")
      .addSource(flags_ >= Flag::NoneRoughnessMetallicTexture
                     ? "#define NONE_ROUGHNESS_METALLIC_TEXTURE\n"
                     : "")
      .addSource(flags_ >= Flag::NormalTexture ? "#define NORMAL_TEXTURE\n"
                                               : "")
      .addSource(flags_ >= Flag::ObjectId ? "#define OBJECT_ID\n" : "")

      // Clearcoat layer
      .addSource(flags_ >= Flag::ClearCoatLayer ? "#define CLEAR_COAT\n" : "")
      .addSource(flags_ >= Flag::ClearCoatTexture
                     ? "#define CLEAR_COAT_TEXTURE\n"
                     : "")
      .addSource(flags_ >= Flag::ClearCoatRoughnessTexture
                     ? "#define CLEAR_COAT_ROUGHNESS_TEXTURE\n"
                     : "")
      .addSource(flags_ >= Flag::ClearCoatNormalTexture
                     ? "#define CLEAR_COAT_NORMAL_TEXTURE\n"
                     : "")

      // Specular Layer
      .addSource(flags_ >= Flag::SpecularLayer ? "#define SPECULAR_LAYER\n"
                                               : "")
      .addSource(flags_ >= Flag::SpecularLayerTexture
                     ? "#define SPECULAR_LAYER_TEXTURE\n"
                     : "")
      .addSource(flags_ >= Flag::SpecularLayerColorTexture
                     ? "#define SPECULAR_LAYER_COLOR_TEXTURE\n"
                     : "")

      // Anisotropy Layer
      .addSource(flags_ >= Flag::AnisotropyLayer ? "#define ANISOTROPY_LAYER\n"
                                                 : "")
      .addSource(flags_ >= Flag::AnisotropyLayerTexture
                     ? "#define ANISOTROPY_LAYER_TEXTURE\n"
                     : "")
      .addSource(flags_ >= Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(flags_ >= Flag::ImageBasedLighting
                     ? "#define IMAGE_BASED_LIGHTING\n"
                     : "")
      // Lights exist and direct lighting is enabled
      .addSource(directLightingIsEnabled_ ? "#define DIRECT_LIGHTING\n" : "")
      // Whether to skip the TBN calc if no precomputed tangents are provided.
      // NOTE : This will disable normal textures if no precomp tangents, which
      // will result in a severe degredation in quality.
      // TODO : implement this in shader. This should only be done if the TBN
      // calc negatively impacts shader performance too dramatically.
      .addSource(flags_ >= Flag::SkipMissingTBNCalc ? "#define SKIP_TBN_CALC\n"
                                                    : "")

      // Whether to use Mikkelsen TBN Calc. Otherwise use faster simplification
      .addSource(flags_ >= Flag::UseMikkelsenTBN ? "#define USE_MIKKELSEN_TBN\n"
                                                 : "")
      // Whether to use the Burley/Disney diffuse calculation, or simple
      // lambertian
      .addSource(flags_ >= Flag::UseBurleyDiffuse
                     ? "#define USE_BURLEY_DIFFUSE\n"
                     : "")
      // Use the shader to approximate srgb<-> linear remapping on specific
      // textures so that all color calcs take place in linear space. This is
      // temporary until we can improve the cpu-side functionality (make sure
      // all loaded textures are converted to linear space, and implement the
      // framebuffer that can map back to sRGB)
      .addSource(mapMatInToLinear ? "#define MAP_MAT_TXTRS_TO_LINEAR\n" : "")

      .addSource(mapIBLInToLinear ? "#define MAP_IBL_TXTRS_TO_LINEAR\n" : "")

      // Whether we should map the output from linear space to SRGB. This will
      // eventually be performed by a framebuffer instead of in the shader.
      .addSource((flags_ >= Flag::MapOutputToSRGB)
                     ? "#define MAP_OUTPUT_TO_SRGB\n"
                     : "")

      .addSource(flags_ >= Flag::UseDirectLightTonemap
                     ? "#define DIRECT_TONE_MAP\n"
                     : "")
      .addSource(flags_ >= Flag::UseIBLTonemap ? "#define IBL_TONE_MAP\n" : "")

      .addSource(flags_ >= Flag::DebugDisplay ? "#define PBR_DEBUG_DISPLAY\n"
                                              : "")

      .addSource(
          Cr::Utility::formatString("#define LIGHT_COUNT {}\n", lightCount_))
      .addSource(rs.getString("pbrCommon.glsl"))
      .addSource(rs.getString("pbrStructs.glsl"))
      .addSource(rs.getString("pbrUniforms.glsl"))
      .addSource(rs.getString("pbrLighting.glsl"))
      .addSource(rs.getString("pbrBSDF.glsl"))
      .addSource(rs.getString("pbrMaterials.glsl"))
      .addSource(rs.getString("pbr.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(vert.compile() && frag.compile());

  attachShaders({vert, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // set texture binding points in the shader;
  // see PBR vertex, fragment shader code for details
  if (lightingIsEnabled_) {
    if (flags_ >= Flag::BaseColorTexture) {
      setUniform(uniformLocation("uBaseColorTexture"),
                 pbrTextureUnitSpace::TextureUnit::BaseColor);
    }
    if (flags_ >= Flag::NoneRoughnessMetallicTexture) {
      setUniform(uniformLocation("uMetallicRoughnessTexture"),
                 pbrTextureUnitSpace::TextureUnit::MetallicRoughness);
    }
    if (flags_ >= Flag::NormalTexture) {
      normalTextureScaleUniform_ = uniformLocation("uNormalTextureScale");
      setUniform(uniformLocation("uNormalTexture"),
                 pbrTextureUnitSpace::TextureUnit::Normal);
    }
    // TODO occlusion texture
  }
  // emissive texture does not depend on lights
  if (flags_ >= Flag::EmissiveTexture) {
    setUniform(uniformLocation("uEmissiveTexture"),
               pbrTextureUnitSpace::TextureUnit::Emissive);
  }

  // IBL related textures
  if (flags_ >= Flag::ImageBasedLighting) {
    setUniform(uniformLocation("uIrradianceMap"),
               pbrTextureUnitSpace::TextureUnit::IrradianceMap);
    setUniform(uniformLocation("uBrdfLUT"),
               pbrTextureUnitSpace::TextureUnit::BrdfLUT);
    setUniform(uniformLocation("uPrefilteredMap"),
               pbrTextureUnitSpace::TextureUnit::PrefilteredMap);
  }

  // cache the uniform locations
  viewMatrixUniform_ = uniformLocation("uViewMatrix");
  modelMatrixUniform_ = uniformLocation("uModelMatrix");
  normalMatrixUniform_ = uniformLocation("uNormalMatrix");
  projMatrixUniform_ = uniformLocation("uProjectionMatrix");

  if (flags_ >= Flag::ObjectId) {
    objectIdUniform_ = uniformLocation("uObjectId");
  }
  if (isTextured_ && (flags_ >= Flag::TextureTransformation)) {
    textureMatrixUniform_ = uniformLocation("uTextureMatrix");
  }

  // materials
  baseColorUniform_ = uniformLocation("uMaterial.baseColor");
  roughnessUniform_ = uniformLocation("uMaterial.roughness");
  metallicUniform_ = uniformLocation("uMaterial.metallic");
  iorUniform_ = uniformLocation("uMaterial.ior");
  emissiveColorUniform_ = uniformLocation("uMaterial.emissiveColor");

  // clearcoat, specular and anisotropy layer data and textures
  if (lightingIsEnabled_) {
    if (flags_ >= Flag::ClearCoatLayer) {
      clearCoatFactorUniform_ = uniformLocation("uClearCoat.factor");
      clearCoatRoughnessUniform_ = uniformLocation("uClearCoat.roughness");
      if (flags_ >= Flag::ClearCoatTexture) {
        setUniform(uniformLocation("uClearCoatTexture"),
                   pbrTextureUnitSpace::TextureUnit::ClearCoatFactor);
      }
      if (flags_ >= Flag::ClearCoatRoughnessTexture) {
        setUniform(uniformLocation("uClearCoatRoughnessTexture"),
                   pbrTextureUnitSpace::TextureUnit::ClearCoatRoughness);
      }
      if (flags_ >= Flag::ClearCoatNormalTexture) {
        clearCoatTextureScaleUniform_ =
            uniformLocation("uClearCoat.normalTextureScale");
        setUniform(uniformLocation("uClearCoatNormalTexture"),
                   pbrTextureUnitSpace::TextureUnit::ClearCoatNormal);
      }
    }
    // specular layer data and textures
    if (flags_ >= Flag::SpecularLayer) {
      specularLayerFactorUniform_ = uniformLocation("uSpecularLayer.factor");
      specularLayerColorFactorUniform_ =
          uniformLocation("uSpecularLayer.colorFactor");
      if (flags_ >= Flag::SpecularLayerTexture) {
        setUniform(uniformLocation("uSpecularLayerTexture"),
                   pbrTextureUnitSpace::TextureUnit::SpecularLayer);
      }
      if (flags_ >= Flag::SpecularLayerColorTexture) {
        setUniform(uniformLocation("uSpecularLayerColorTexture"),
                   pbrTextureUnitSpace::TextureUnit::SpecularLayerColor);
      }
    }

    // anisotropy layer data and texture
    if (flags_ >= Flag::AnisotropyLayer) {
      anisotropyLayerFactorUniform_ =
          uniformLocation("uAnisotropyLayer.factor");
      anisotropyLayerDirectionUniform_ =
          uniformLocation("uAnisotropyLayer.direction");
      if (flags_ >= Flag::AnisotropyLayerTexture) {
        setUniform(uniformLocation("uAnisotropyLayerTexture"),
                   pbrTextureUnitSpace::TextureUnit::AnisotropyLayer);
      }
    }

  }  // if lighting is enabled

  // lights
  if (directLightingIsEnabled_) {
    lightRangesUniform_ = uniformLocation("uLightRanges");
    lightColorsUniform_ = uniformLocation("uLightColors");
    lightDirectionsUniform_ = uniformLocation("uLightDirections");
    // global light intensity across all direct lights
    directLightingIntensityUniform_ = uniformLocation("uDirectLightIntensity");
  }

  cameraWorldPosUniform_ = uniformLocation("uCameraWorldPos");

  if ((directLightingIsEnabled_ && flags_ >= Flag::UseDirectLightTonemap) ||
      (flags_ >= Flag::ImageBasedLighting && flags_ >= Flag::UseIBLTonemap)) {
    tonemapExposureUniform_ = uniformLocation("uExposure");
  }

  if (mapInputToLinear_) {
    gammaUniform_ = uniformLocation("uGamma");
  }

  if (flags_ >= Flag::MapOutputToSRGB) {
    invGammaUniform_ = uniformLocation("uInvGamma");
  }

  // IBL related uniform
  if (flags_ >= Flag::ImageBasedLighting) {
    prefilteredMapMipLevelsUniform_ =
        uniformLocation("uPrefilteredMapMipLevels");
  }

  if (directAndIBLisEnabled_) {
    // Apply scaling if -both- lights and IBL are enabled
    // pbr equation scales - use to mix IBL and direct lighting
    // Should never be set to 0 or will cause warnings to occur in shader
    componentScalesUniform_ = uniformLocation("uComponentScales");
  }

  // for debug info
  if (flags_ >= Flag::DebugDisplay) {
    pbrDebugDisplayUniform_ = uniformLocation("uPbrDebugDisplay");
  }

  /////////////////////////////
  // Initializations

  // initialize the shader with some "reasonable defaults"
  setViewMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  setModelMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  setProjectionMatrix(Mn::Matrix4{Mn::Math::IdentityInit});
  if (lightingIsEnabled_) {
    setBaseColor(Mn::Color4{0.7f});
    setRoughness(0.0f);
    setMetallic(1.0f);
    setIndexOfRefraction(1.5);
    if (flags_ >= Flag::NormalTexture) {
      setNormalTextureScale(1.0f);
    }
    setNormalMatrix(Mn::Matrix3x3{Mn::Math::IdentityInit});
    if (flags_ >= Flag::ClearCoatLayer) {
      setClearCoatFactor(0.0f);
      setClearCoatRoughness(0.0f);
      if (flags_ >= Flag::ClearCoatNormalTexture) {
        setClearCoatNormalTextureScale(1.0f);
      }
    }
    if (flags_ >= Flag::SpecularLayer) {
      setSpecularLayerFactor(1.0f);
      setSpecularLayerColorFactor(Mn::Color3{1.0f});
    }
    if (flags_ >= Flag::AnisotropyLayer) {
      setAnisotropyLayerFactor(0.0f);
      // Default to 0 rotation
      setAnisotropyLayerDirection({1.0f, 0.0f});
    }
  }

  if (directLightingIsEnabled_) {
    setLightVectors(Cr::Containers::Array<Mn::Vector4>{
        Cr::DirectInit, lightCount_,
        // a single directional "fill" light, coming from the center of the
        // camera.
        Mn::Vector4{0.0f, 0.0f, -1.0f, 0.0f}});
    Cr::Containers::Array<Mn::Color3> colors{Cr::DirectInit, lightCount_,
                                             Mn::Color3{1.0f}};
    setLightColors(colors);
    setLightRanges(Cr::Containers::Array<Mn::Float>{Cr::DirectInit, lightCount_,
                                                    Mn::Constants::inf()});
    // initialize global, config-driven light intensity
    setDirectLightIntensity(1.0f);
    // initialize tonemap exposure
  }

  if (lightingIsEnabled_) {
    // TODO : gate this based on whether tonemapping is being used.
    setTonemapExposure(4.5f);
  }

  setGamma({2.2f, 2.2f, 2.2f});

  setEmissiveColor(Mn::Color3{0.0f});

  PbrShader::PbrEquationScales scales;
  // Set mix if both lights and IBL are enabled
  // Should never be 0 or will cause shader warnings
  if (directAndIBLisEnabled_) {
    // These are empirical numbers. Discount the diffuse light from IBL so the
    // ambient light will not be too strong. Also keeping the IBL specular
    // component relatively low can guarantee the super glossy surface would
    // not reflect the environment like a mirror.
    scales.iblDiffuse = 0.5;
    scales.iblSpecular = 0.5;
    scales.directDiffuse = 0.5;
    scales.directSpecular = 0.5;
  }
  setPbrEquationScales(scales);

  if (flags_ >= Flag::DebugDisplay) {
    setDebugDisplay(PbrDebugDisplay::None);
  }
}  // constructor

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PbrShader& PbrShader::bindBaseColorTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::BaseColorTexture,
                 "PbrShader::bindBaseColorTexture(): the shader was not "
                 "created with base color texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::BaseColor);
  }
  return *this;
}

PbrShader& PbrShader::bindMetallicRoughnessTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ >= (Flag::NoneRoughnessMetallicTexture),
      "PbrShader::bindMetallicRoughnessTexture(): the shader was not "
      "created with metallicRoughness texture enabled.",
      *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::MetallicRoughness);
  }
  return *this;
}

PbrShader& PbrShader::bindNormalTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::NormalTexture,
                 "PbrShader::bindNormalTexture(): the shader was not "
                 "created with normal texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::Normal);
  }
  return *this;
}

PbrShader& PbrShader::bindEmissiveTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::EmissiveTexture,
                 "PbrShader::bindEmissiveTexture(): the shader was not "
                 "created with emissive texture enabled",
                 *this);
  // emissive texture does not depend on lights
  texture.bind(pbrTextureUnitSpace::TextureUnit::Emissive);
  return *this;
}

PbrShader& PbrShader::bindClearCoatFactorTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::ClearCoatTexture,
                 "PbrShader::bindClearCoatFactorTexture(): the shader was not "
                 "created with clearcoat factor texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::ClearCoatFactor);
  }
  return *this;
}

PbrShader& PbrShader::bindClearCoatRoughnessTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ >= Flag::ClearCoatRoughnessTexture,
      "PbrShader::bindClearCoatRoughnessTexture(): the shader was not "
      "created with clearcoat roughness texture enabled",
      *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::ClearCoatRoughness);
  }
  return *this;
}

PbrShader& PbrShader::bindClearCoatNormalTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::ClearCoatNormalTexture,
                 "PbrShader::bindClearCoatNormalTexture(): the shader was not "
                 "created with clearcoat normal texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::ClearCoatNormal);
  }
  return *this;
}

PbrShader& PbrShader::bindSpecularLayerTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::SpecularLayerTexture,
                 "PbrShader::bindSpecularLayerTexture(): the shader was not "
                 "created with specular layer texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::SpecularLayer);
  }
  return *this;
}

PbrShader& PbrShader::bindSpecularLayerColorTexture(
    Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(
      flags_ >= Flag::SpecularLayerColorTexture,
      "PbrShader::bindSpecularLayerColorTexture(): the shader was not "
      "created with specular layer color texture enabled",
      *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::SpecularLayerColor);
  }
  return *this;
}

PbrShader& PbrShader::bindAnisotropyLayerTexture(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::AnisotropyLayerTexture,
                 "PbrShader::bindAnisotropyLayerTexture(): the shader was not "
                 "created with anisotropy layer texture enabled",
                 *this);
  if (lightingIsEnabled_) {
    texture.bind(pbrTextureUnitSpace::TextureUnit::AnisotropyLayer);
  }
  return *this;
}

PbrShader& PbrShader::bindIrradianceCubeMap(Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ >= Flag::ImageBasedLighting,
                 "PbrShader::bindIrradianceCubeMap(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::IrradianceMap);
  return *this;
}

PbrShader& PbrShader::bindBrdfLUT(Mn::GL::Texture2D& texture) {
  CORRADE_ASSERT(flags_ >= Flag::ImageBasedLighting,
                 "PbrShader::bindBrdfLUT(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::BrdfLUT);
  return *this;
}

PbrShader& PbrShader::bindPrefilteredMap(Mn::GL::CubeMapTexture& texture) {
  CORRADE_ASSERT(flags_ >= Flag::ImageBasedLighting,
                 "PbrShader::bindPrefilteredMap(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  texture.bind(pbrTextureUnitSpace::TextureUnit::PrefilteredMap);
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
  if (flags_ >= Flag::ObjectId) {
    setUniform(objectIdUniform_, objectId);
  }
  return *this;
}

PbrShader& PbrShader::setPrefilteredMapMipLevels(unsigned int mipLevels) {
  CORRADE_ASSERT(flags_ >= Flag::ImageBasedLighting,
                 "PbrShader::setPrefilteredMapMipLevels(): the shader was not "
                 "created with image based lighting enabled",
                 *this);
  setUniform(prefilteredMapMipLevelsUniform_, mipLevels);
  return *this;
}

PbrShader& PbrShader::setBaseColor(const Mn::Color4& color) {
  if (lightingIsEnabled_) {
    setUniform(baseColorUniform_, color);
  }
  return *this;
}

PbrShader& PbrShader::setEmissiveColor(const Mn::Color3& color) {
  setUniform(emissiveColorUniform_, color);
  return *this;
}

PbrShader& PbrShader::setRoughness(float roughness) {
  if (lightingIsEnabled_) {
    setUniform(roughnessUniform_, roughness);
  }
  return *this;
}

PbrShader& PbrShader::setMetallic(float metallic) {
  if (lightingIsEnabled_) {
    setUniform(metallicUniform_, metallic);
  }
  return *this;
}

PbrShader& PbrShader::setIndexOfRefraction(float ior) {
  if (lightingIsEnabled_) {
    setUniform(iorUniform_, ior);
  }
  return *this;
}

PbrShader& PbrShader::setClearCoatFactor(float ccFactor) {
  if (lightingIsEnabled_) {
    setUniform(clearCoatFactorUniform_, ccFactor);
  }
  return *this;
}

PbrShader& PbrShader::setClearCoatRoughness(float ccRoughness) {
  if (lightingIsEnabled_) {
    setUniform(clearCoatRoughnessUniform_, ccRoughness);
  }
  return *this;
}

PbrShader& PbrShader::setClearCoatNormalTextureScale(float ccTextureScale) {
  if (lightingIsEnabled_) {
    setUniform(clearCoatTextureScaleUniform_, ccTextureScale);
  }
  return *this;
}

PbrShader& PbrShader::setSpecularLayerFactor(float specLayerFactor) {
  if (lightingIsEnabled_) {
    setUniform(specularLayerFactorUniform_, specLayerFactor);
  }
  return *this;
}

PbrShader& PbrShader::setAnisotropyLayerFactor(float anisoLayerFactor) {
  if (lightingIsEnabled_) {
    setUniform(anisotropyLayerFactorUniform_, anisoLayerFactor);
  }
  return *this;
}

PbrShader& PbrShader::setAnisotropyLayerDirection(
    const Magnum::Vector2& anisoLayerDirection) {
  if (lightingIsEnabled_) {
    setUniform(anisotropyLayerDirectionUniform_, anisoLayerDirection);
  }
  return *this;
}

PbrShader& PbrShader::setSpecularLayerColorFactor(
    const Mn::Color3& specLayerColorFactor) {
  if (lightingIsEnabled_) {
    setUniform(specularLayerColorFactorUniform_, specLayerColorFactor);
  }
  return *this;
}
PbrShader& PbrShader::setPbrEquationScales(const PbrEquationScales& scales) {
  if (directAndIBLisEnabled_) {
    Mn::Vector4 componentScales{scales.directDiffuse, scales.directSpecular,
                                scales.iblDiffuse, scales.iblSpecular};
    setUniform(componentScalesUniform_, componentScales);
  }
  return *this;
}

PbrShader& PbrShader::setDebugDisplay(PbrDebugDisplay index) {
  CORRADE_ASSERT(flags_ >= Flag::DebugDisplay,
                 "PbrShader::setDebugDisplay(): the shader was not "
                 "created with DebugDisplay enabled",
                 *this);
  setUniform(pbrDebugDisplayUniform_, int(index));
  return *this;
}

PbrShader& PbrShader::setCameraWorldPosition(
    const Mn::Vector3& cameraWorldPos) {
  setUniform(cameraWorldPosUniform_, cameraWorldPos);
  return *this;
}

PbrShader& PbrShader::setTextureMatrix(const Mn::Matrix3& matrix) {
  CORRADE_ASSERT(flags_ >= Flag::TextureTransformation,
                 "PbrShader::setTextureMatrix(): the shader was not "
                 "created with texture transformation enabled",
                 *this);
  if (isTextured_) {
    // Only required if textures are present (including emissive, which is
    // independent of lighting)
    setUniform(textureMatrixUniform_, matrix);
  }
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
  for (size_t i = 0; i < colors.size(); ++i) {
    setLightColor(i, colors[i]);
  }
  // setUniform(lightColorsUniform_, colors);
  return *this;
}

PbrShader& PbrShader::setLightColors(std::initializer_list<Mn::Color3> colors) {
  return setLightColors(Cr::Containers::arrayView(colors));
}

PbrShader& PbrShader::setNormalTextureScale(float scale) {
  CORRADE_ASSERT(flags_ >= Flag::NormalTexture,
                 "PbrShader::setNormalTextureScale(): the shader was not "
                 "created with normal texture enabled",
                 *this);
  if (lightingIsEnabled_) {
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

PbrShader& PbrShader::setDirectLightIntensity(float lightIntensity) {
  if (lightingIsEnabled_) {
    setUniform(directLightingIntensityUniform_, lightIntensity);
  }
  return *this;
}

PbrShader& PbrShader::setGamma(const Mn::Vector3& gamma) {
  // if any input values are mapped to linear, should set gamma uniform value
  if (mapInputToLinear_) {
    setUniform(gammaUniform_, gamma);
  }
  if (flags_ >= Flag::MapOutputToSRGB) {
    setUniform(invGammaUniform_, 1.0f / gamma);
  }
  return *this;
}

PbrShader& PbrShader::setTonemapExposure(float exposure) {
  if (lightingIsEnabled_) {
    setUniform(tonemapExposureUniform_, exposure);
  }
  return *this;
}
}  // namespace gfx
}  // namespace esp
