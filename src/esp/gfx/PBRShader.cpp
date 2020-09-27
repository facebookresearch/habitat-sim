// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/PixelFormat.h>

#include "PBRShader.h"
#include "esp/core/esp.h"
#include "esp/io/io.h"

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
  albedo = 0,
  roughness = 1,
  metallic = 2,
  normal = 3,
};
}  // namespace

PBRShader::PBRShader(Flags flags, unsigned int lightCount)
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

  // Add macros
  vert.addSource(flags & (Flag::AlbedoTexture | Flag::RoughnessTexture |
                          Flag::MetallicTexture | Flag::NormalTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(rs.get("pbr.vert"));

  frag.addSource(flags & (Flag::AlbedoTexture | Flag::RoughnessTexture |
                          Flag::MetallicTexture | Flag::NormalTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::AlbedoTexture ? "#define ALBEDO_TEXTURE\n" : "")
      .addSource(flags & Flag::RoughnessTexture ? "#define ROUGHNESS_TEXTURE\n"
                                                : "")
      .addSource(flags & Flag::MetallicTexture ? "#define METALLIC_TEXTURE\n"
                                               : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
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

  // set texture binding points in the shader;
  // see PBR vertex, fragment shader code for details
  if (flags & Flag::AlbedoTexture) {
    setUniform(uniformLocation("albedoTexture"),
               TextureBindingPointIndex::albedo);
  }
  if (flags & Flag::RoughnessTexture) {
    setUniform(uniformLocation("RoughnessTexture"),
               TextureBindingPointIndex::roughness);
  }
  if (flags & Flag::MetallicTexture) {
    setUniform(uniformLocation("MetallicTexture"),
               TextureBindingPointIndex::metallic);
  }
  if (flags & Flag::NormalTexture) {
    setUniform(uniformLocation("NormalTexture"),
               TextureBindingPointIndex::normal);
  }

  // cache the uniform locations
  modelviewMatrixUniform_ = uniformLocation("ModelViewMatrix");
  normalMatrixUniform_ = uniformLocation("NormalMatrix");
  mvpMatrixUniform_ = uniformLocation("MVP");
  if (flags & Flag::ObjectId) {
    objectIdUniform_ = uniformLocation("objectId");
  }

  // materials
  baseColorUniform_ = uniformLocation("Material.baseColor");
  roughnessUniform_ = uniformLocation("Material.roughness");
  metallicUniform_ = uniformLocation("Material.metallic");

  // lights
  if (lightCount_) {
    lightsUniform_.resize(lightCount_);
    for (int iLight = 0; iLight < lightCount_; ++iLight) {
      lightsUniform_[iLight].lightColor = uniformLocation(
          Cr::Utility::formatString("Light[{}].lightColor", iLight));
      lightsUniform_[iLight].intensity = uniformLocation(
          Cr::Utility::formatString("Light[{}].intensity", iLight));
      lightsUniform_[iLight].range =
          uniformLocation(Cr::Utility::formatString("Light[{}].range", iLight));
      lightDirectionsUniform_[iLight] = uniformLocation(
          Cr::Utility::formatString("LightDirections[{}]", iLight));
    }
  }
}

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PBRShader& PBRShader::bindAlbedoTexture(Mn::GL::Texture2D& texture) {
  texture.bind(TextureBindingPointIndex::albedo);
  return *this;
}

PBRShader& PBRShader::bindRoughnessTexture(Mn::GL::Texture2D& texture) {
  texture.bind(TextureBindingPointIndex::roughness);
  return *this;
}

PBRShader& PBRShader::bindMetallicTexture(Mn::GL::Texture2D& texture) {
  texture.bind(TextureBindingPointIndex::metallic);
  return *this;
}

PBRShader& PBRShader::bindNormalTexture(Mn::GL::Texture2D& texture) {
  texture.bind(TextureBindingPointIndex::normal);
  return *this;
}

PBRShader& PBRShader::setMVPMatrix(const Mn::Matrix4& matrix) {
  setUniform(mvpMatrixUniform_, matrix);
  return *this;
}

PBRShader& PBRShader::setObjectId(unsigned int objectId) {
  if (flags_ & Flag::ObjectId) {
    setUniform(objectIdUniform_, objectId);
  }
  return *this;
}

PBRShader& PBRShader::setBaseColor(const Mn::Color4& color) {
  setUniform(baseColorUniform_, color);
  return *this;
}

PBRShader& PBRShader::setRoughness(float roughness) {
  setUniform(roughnessUniform_, roughness);
  return *this;
}

PBRShader& PBRShader::setMetallic(float metallic) {
  setUniform(metallicUniform_, metallic);
  return *this;
}

PBRShader& PBRShader::bindTextures(Magnum::GL::Texture2D* albedo,
                                   Magnum::GL::Texture2D* roughness,
                                   Magnum::GL::Texture2D* metallic,
                                   Magnum::GL::Texture2D* normal) {
  if (albedo) {
    bindAlbedoTexture(*albedo);
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

}  // namespace gfx
}  // namespace esp
