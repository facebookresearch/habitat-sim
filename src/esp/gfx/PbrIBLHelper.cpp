// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrIBLHelper.h"
#include <Magnum/GL/Mesh.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Angle.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/Copy.h>
#include <Magnum/MeshTools/FlipNormals.h>
#include <Magnum/Primitives/Cube.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshData.h>

#include "CubeMapCamera.h"

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

namespace {
// TODO: move to PbrShaderAttributes
constexpr unsigned int environmentMapSize = 1024;
constexpr unsigned int prefilteredMapSize = 1024;
constexpr unsigned int irradianceMapSize = 128;

Mn::Matrix4 buildDfltPerspectiveMatrix() {
  using Mn::Math::Literals::operator""_degf;
  return Mn::Matrix4::perspectiveProjection(
      90.0_degf,  // horizontal field of view angle
      1.0f,       // aspect ratio (width/height)
      0.01f,      // z-near plane
      1000.0f);   // z-far plane
}  // buildDfltPerspectiveMatrix()

}  // namespace

PbrIBLHelper::PbrIBLHelper(
    ShaderManager& shaderManager,
    const std::shared_ptr<Mn::GL::Texture2D>& brdfLUT,
    const std::shared_ptr<Mn::GL::Texture2D>& envMapTexture)
    : shaderManager_(shaderManager), brdfLUT_(brdfLUT) {
  // convert the loaded texture into a cubemap
  convertEquirectangularToCubeMap(envMapTexture);
}

void PbrIBLHelper::convertEquirectangularToCubeMap(
    const std::shared_ptr<Mn::GL::Texture2D>& envMapTexture) {
  // prepare a mesh to be displayed
  Mn::GL::Mesh mesh = Mn::GL::Mesh{};
  mesh.setCount(3);

  // Initialize the base environment map
  auto environmentMap =
      CubeMap(environmentMapSize,
              CubeMap::Flag::ColorTexture | CubeMap::Flag::ManuallyBuildMipmap);

  // prepare the shader
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrEquiRectangularToCubeMapShader>
      shader = shaderManager_.get<Mn::GL::AbstractShaderProgram,
                                  PbrEquiRectangularToCubeMapShader>(
          "equirectangularToCubeMap");
  // If not found create
  if (!shader) {
    shaderManager_.set<Mn::GL::AbstractShaderProgram>(
        shader.key(), new PbrEquiRectangularToCubeMapShader(),
        Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
  }
  CORRADE_INTERNAL_ASSERT(shader);

  // bind the equirectangular texture
  shader->bindEquirectangularTexture(*envMapTexture);

  // draw the 6 sides one by one
  for (unsigned int iSide = 0; iSide < 6; ++iSide) {
    // bind frambuffer, clear color and depth
    environmentMap.prepareToDraw(iSide);
    shader->setCubeSideIndex(iSide);
    shader->draw(mesh);
  }
  // do NOT forget to populate to all the mip levels!

  // environmentMap.getTexture(CubeMap::TextureType::Color).generateMipmap();
  auto& cubemapTexture = environmentMap.getTexture(CubeMap::TextureType::Color);
  cubemapTexture.generateMipmap();
  // compute the irradiance map for indirect diffuse part
  computeIrradianceMap(cubemapTexture);

  // compute the prefiltered environment map (indirect specular part)
  computePrefilteredEnvMap(cubemapTexture);

}  // convertEquirectangularToCubeMap

void PbrIBLHelper::computeIrradianceMap(Mn::GL::CubeMapTexture& envCubeMap) {
  // Prepare the shader for Irradiance Map

  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader> shader =
      shaderManager_
          .get<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader>(
              "irradianceMap");

  if (!shader) {
    shaderManager_.set<Mn::GL::AbstractShaderProgram>(
        shader.key(),
        new PbrPrecomputedMapShader(PbrPrecomputedMapShader::Flags{
            PbrPrecomputedMapShader::Flag::IrradianceMap}),
        Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
  }
  CORRADE_INTERNAL_ASSERT(shader);

  shader->bindEnvironmentMap(envCubeMap);
  shader->setProjectionMatrix(buildDfltPerspectiveMatrix());

  // prepare a cube
  Mn::Trade::MeshData cubeData =
      Mn::MeshTools::copy(Mn::Primitives::cubeSolid());
  // camera is now inside the cube, must flip the face winding, otherwise all
  // the faces are culled
  Mn::MeshTools::flipFaceWindingInPlace(cubeData.mutableIndices());
  Mn::GL::Mesh cube = Mn::MeshTools::compile(cubeData);

  irradianceMap_ = CubeMap(irradianceMapSize, CubeMap::Flag::ColorTexture);
  // compute irradiance map
  for (unsigned int iSide = 0; iSide < 6; ++iSide) {
    Mn::Matrix4 viewMatrix = CubeMapCamera::getCameraLocalTransform(
                                 CubeMapCamera::cubeMapCoordinate(iSide))
                                 .inverted();
    shader->setTransformationMatrix(viewMatrix);
    // bind framebuffer, clear color and depth
    irradianceMap_->prepareToDraw(iSide);
    shader->draw(cube);
  }

}  // computeIrradianceMap

void PbrIBLHelper::computePrefilteredEnvMap(
    Mn::GL::CubeMapTexture& envCubeMap) {
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader> shader =
      shaderManager_
          .get<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader>(
              "prefilteredMap");

  if (!shader) {
    shaderManager_.set<Mn::GL::AbstractShaderProgram>(
        shader.key(),
        new PbrPrecomputedMapShader(PbrPrecomputedMapShader::Flags{
            PbrPrecomputedMapShader::Flag::PrefilteredMap}),
        Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
  }

  shader->bindEnvironmentMap(envCubeMap);
  shader->setProjectionMatrix(buildDfltPerspectiveMatrix());

  // prepare a cube
  Mn::Trade::MeshData cubeData =
      Mn::MeshTools::copy(Mn::Primitives::cubeSolid());
  // camera is now inside the cube, must flip the face winding, otherwise all
  // the faces are culled
  Mn::MeshTools::flipFaceWindingInPlace(cubeData.mutableIndices());
  Mn::GL::Mesh cube = Mn::MeshTools::compile(cubeData);

  prefilteredMap_ =
      CubeMap(prefilteredMapSize,
              CubeMap::Flag::ColorTexture | CubeMap::Flag::ManuallyBuildMipmap);
  // compute prefiltered map
  unsigned int maxMipLevels = prefilteredMap_->getMipmapLevels();
  CORRADE_INTERNAL_ASSERT(maxMipLevels ==
                          Mn::Math::log2(prefilteredMapSize) + 1);
  for (unsigned int iMip = 0; iMip < maxMipLevels; ++iMip) {
    float roughness = float(iMip) / float(maxMipLevels - 1);

    shader->setRoughness(roughness);

    for (unsigned int iSide = 0; iSide < 6; ++iSide) {
      Mn::Matrix4 viewMatrix = CubeMapCamera::getCameraLocalTransform(
                                   CubeMapCamera::cubeMapCoordinate(iSide))
                                   .inverted();
      shader->setTransformationMatrix(viewMatrix);
      // bind framebuffer, clear color and depth
      prefilteredMap_->prepareToDraw(
          iSide,
          RenderCamera::Flag::ClearColor | RenderCamera::Flag::ClearDepth,
          iMip);
      shader->draw(cube);
    }  // for iSide
  }    // for iMip

}  // computePrefilteredEnvMap

CubeMap& PbrIBLHelper::getIrradianceMap() {
  CORRADE_ASSERT(
      irradianceMap_,
      "PbrIBLHelper::getIrradianceMap(): the irradiance map is empty. "
      "Did you forget to load or compute it (from environment map)?",
      *irradianceMap_);

  return *irradianceMap_;
}

CubeMap& PbrIBLHelper::getPrefilteredMap() {
  CORRADE_ASSERT(prefilteredMap_,
                 "PbrIBLHelper::getPrefilteredMap(): the pre-filtered "
                 "cube map is empty."
                 "Did you forget to load or compute it (from environment map)?",
                 *prefilteredMap_);
  return *prefilteredMap_;
}

Mn::GL::Texture2D& PbrIBLHelper::getBrdfLookupTable() {
  CORRADE_ASSERT(brdfLUT_,
                 "PbrIBLHelper::getBrdfLookupTable(): the brdf lookup "
                 "table (a texture) is empty"
                 "Did you forget to load or compute it (from environment map)?",
                 *brdfLUT_);

  return *brdfLUT_;
}

}  // namespace gfx
}  // namespace esp
