// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/Reference.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/BufferTextureFormat.h>
#include <Magnum/GL/Context.h>
#include <Magnum/GL/Shader.h>
#include <Magnum/GL/Version.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include "PTexMeshShader.h"
#include "esp/assets/PTexMeshData.h"
#include "esp/core/esp.h"
#include "esp/io/io.h"

// This is to import the "resources" at runtime. // When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called // *outside* of any namespace.
static void importShaderResources() {
  CORRADE_RESOURCE_INITIALIZE(ShaderResources)
}

namespace Mn = Magnum;

namespace esp {
namespace gfx {

namespace {
enum TextureBindingPointIndex : uint8_t {
  atlas = 0,
#ifndef CORRADE_TARGET_APPLE
  adjFaces = 1,
#endif
};
}  // namespace

PTexMeshShader::PTexMeshShader() {
  MAGNUM_ASSERT_GL_VERSION_SUPPORTED(Mn::GL::Version::GL410);

  if (!Corrade::Utility::Resource::hasGroup("default-shaders")) {
    importShaderResources();
  }

  // this is not the file name, but the group name in the config file
  const Corrade::Utility::Resource rs{"default-shaders"};

  Mn::GL::Shader vert{Mn::GL::Version::GL410, Mn::GL::Shader::Type::Vertex};
  Mn::GL::Shader geom{Mn::GL::Version::GL410, Mn::GL::Shader::Type::Geometry};
  Mn::GL::Shader frag{Mn::GL::Version::GL410, Mn::GL::Shader::Type::Fragment};

  vert.addSource(rs.get("ptex-default-gl410.vert"));
  geom.addSource(rs.get("ptex-default-gl410.geom"));
#ifdef CORRADE_TARGET_APPLE
  frag.addSource("#define CORRADE_TARGET_APPLE\n");
#endif
  frag.addSource(rs.get("ptex-default-gl410.frag"));

  CORRADE_INTERNAL_ASSERT_OUTPUT(Mn::GL::Shader::compile({vert, geom, frag}));

  attachShaders({vert, geom, frag});

  CORRADE_INTERNAL_ASSERT_OUTPUT(link());

  // set texture binding points in the shader;
  // see ptex fragment shader code for details
  setUniform(uniformLocation("atlasTex"), TextureBindingPointIndex::atlas);
#ifndef CORRADE_TARGET_APPLE
  setUniform(uniformLocation("meshAdjFaces"),
             TextureBindingPointIndex::adjFaces);
#endif

  // cache the uniform locations
  MVPMatrixUniform_ = uniformLocation("MVP");
  exposureUniform_ = uniformLocation("exposure");
  gammaUniform_ = uniformLocation("gamma");
  saturationUniform_ = uniformLocation("saturation");
  tileSizeUniform_ = uniformLocation("tileSize");
  widthInTilesUniform_ = uniformLocation("widthInTiles");
  objectIdUniform_ = uniformLocation("objectId");
}

// Note: the texture binding points are explicitly specified above.
// Cannot use "explicit uniform location" directly in shader since
// it requires GL4.3 (We stick to GL4.1 for MacOS).
PTexMeshShader& PTexMeshShader::bindAtlasTexture(Mn::GL::Texture2D& texture) {
  texture.bind(TextureBindingPointIndex::atlas);
  return *this;
}

PTexMeshShader& PTexMeshShader::bindAdjFacesBufferTexture(
    // NOLINTNEXTLINE(misc-unused-parameters)
    Mn::GL::BufferTexture& texture) {
#ifndef CORRADE_TARGET_APPLE
  texture.bind(TextureBindingPointIndex::adjFaces);
#endif
  return *this;
}

PTexMeshShader& PTexMeshShader::setMVPMatrix(const Mn::Matrix4& matrix) {
  setUniform(MVPMatrixUniform_, matrix);
  return *this;
}

PTexMeshShader& PTexMeshShader::setExposure(float exposure) {
  setUniform(exposureUniform_, exposure);
  return *this;
}
PTexMeshShader& PTexMeshShader::setGamma(float gamma) {
  setUniform(gammaUniform_, gamma);
  return *this;
}

PTexMeshShader& PTexMeshShader::setSaturation(float saturation) {
  setUniform(saturationUniform_, saturation);
  return *this;
}

PTexMeshShader& PTexMeshShader::setAtlasTextureSize(Mn::GL::Texture2D& texture,
                                                    uint32_t tileSize) {
  setUniform(tileSizeUniform_, static_cast<Mn::Int>(tileSize));

  // get image width in given mip level 0
  int mipLevel = 0;
  const auto width = texture.imageSize(mipLevel).x();
  setUniform(widthInTilesUniform_, int(width / tileSize));
  return *this;
}

PTexMeshShader& PTexMeshShader::setObjectId(unsigned int objectId) {
  setUniform(objectIdUniform_, objectId);
  return *this;
}

}  // namespace gfx
}  // namespace esp
