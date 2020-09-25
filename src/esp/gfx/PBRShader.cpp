// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Reference.h>
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

PBRShader::PBRShader(Flags flags, Mn::UnsignedInt lightCount) {
  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

#ifdef MAGNUM_TARGET_WEBGL
  Mn::GL::Version glVersion = Mn::GL::Version::GLES300;
#else
  Mn::GL::Version glVersion = Mn::GL::Version::GL330;
#endif

  // this is not the file name, but the group name in the config file
  const Corrade::Utility::Resource rs{"default-shaders"};

  Mn::GL::Shader vert{glVersion, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader frag{glVersion, Mn::GL::Shader::Type::Fragment};

  if (flags & Flag::AlbedoTexture) {
    vert.addSource("#define ALBEDO_TEXTURE\n");
    frag.addSource("#define ALBEDO_TEXTURE\n");
  }

  if (flags & Flag::RoughnessTexture) {
    vert.addSource("#define ROUGHNESS_TEXTURE\n");
    frag.addSource("#define ROUGHNESS_TEXTURE\n");
  }

  if (flags & Flag::MetallicTexture) {
    vert.addSource("#define METALLIC_TEXTURE\n");
    frag.addSource("#define METALLIC_TEXTURE\n");
  }

  if (flags & Flag::NormalTexture) {
    vert.addSource("#define NORMAL_TEXTURE\n");
    frag.addSource("#define NORMAL_TEXTURE\n");
  }

  vert.addSource(rs.get("ptex-default-gl410.vert"));
  frag.addSource(rs.get("ptex-default-gl410.frag"));

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

  // uniform mat4 ModelViewMatrix;
  // uniform mat3 NormalMatrix;  // inverse transpose of 3x3 modelview matrix

  mvpMatrixUniform_ = uniformLocation("MVP");
  objectIdUniform_ = uniformLocation("objectId");
  baseColorUniform_ = uniformLocation("Material.BaseColor");
  roughnessUniform_ = uniformLocation("Material.Roughness");
  metallicUniform_ = uniformLocation("Material.Metallic");
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
  setUniform(objectIdUniform_, objectId);
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

}  // namespace gfx
}  // namespace esp
