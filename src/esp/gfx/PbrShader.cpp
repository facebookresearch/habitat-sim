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
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/PixelFormat.h>

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

  // Add macros
  vert.addSource(flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
                          Flag::MetallicTexture | Flag::NormalTexture)
                     ? "#define TEXTURED\n"
                     : "")
      .addSource(flags & Flag::NormalTexture ? "#define NORMAL_TEXTURE\n" : "")
      .addSource(flags & Flag::PrecomputedTangent
                     ? "#define PRECOMPUTED_TANGENT\n"
                     : "")
      .addSource(rs.get("pbr.vert"));

  frag.addSource(flags & (Flag::BaseColorTexture | Flag::RoughnessTexture |
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

  // materials
  baseColorUniform_ = uniformLocation("Material.baseColor");
  roughnessUniform_ = uniformLocation("Material.roughness");
  metallicUniform_ = uniformLocation("Material.metallic");

  // lights
  if (lightCount_) {
    lightsUniform_.resize(lightCount_);
    lightDirectionsUniform_.resize(lightCount_);
    for (int iLight = 0; iLight < lightCount_; ++iLight) {
      lightsUniform_[iLight].color =
          uniformLocation(Cr::Utility::formatString("Light[{}].color", iLight));
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

PbrShader& PbrShader::setLightPosition(unsigned int lightIndex,
                                       const Mn::Vector3& pos) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightPosition: lightIndex" << lightIndex << "is illegal.",
      *this);

  setUniform(lightDirectionsUniform_[lightIndex], Mn::Vector4{pos, 1.0});
  return *this;
}

PbrShader& PbrShader::setLightDirection(unsigned int lightIndex,
                                        const Mn::Vector3& dir) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightDirection: lightIndex" << lightIndex << "is illegal.",
      *this);
  setUniform(lightDirectionsUniform_[lightIndex], Mn::Vector4{dir, 0.0});
  return *this;
}

PbrShader& PbrShader::setLightVector(unsigned int lightIndex,
                                     const Magnum::Vector4& vec) {
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

  setUniform(lightDirectionsUniform_[lightIndex], vec);

  return *this;
}

PbrShader& PbrShader::setLightRange(unsigned int lightIndex, float range) {
  CORRADE_ASSERT(
      lightIndex < lightCount_,
      "PbrShader::setLightRange: lightIndex" << lightIndex << "is illegal.",
      *this);
  if (lightCount_) {
    setUniform(lightsUniform_[lightIndex].range, range);
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
    setUniform(lightsUniform_[lightIndex].color, finalColor);
  }
  return *this;
}

}  // namespace gfx
}  // namespace esp
