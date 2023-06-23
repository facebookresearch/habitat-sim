// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrImageBasedLighting.h"

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Resource.h>
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

// This is to import the "resources" at runtime. When the resource is
// compiled into static library, it must be explicitly initialized via this
// macro, and should be called *outside* of any namespace.
static void importPbrImageResources() {
  CORRADE_RESOURCE_INITIALIZE(PbrImageResources)
}

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

namespace {
constexpr unsigned int environmentMapSize = 1024;
constexpr unsigned int prefilteredMapSize = 1024;
constexpr unsigned int irradianceMapSize = 128;

Cr::Containers::Optional<Mn::Trade::ImageData2D> loadImageData(
    const std::string& imageFilename) {  // loadImageData
  // plugin manager used to instantiate importers which in turn are used
  // to load image data
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  std::string importerName{"AnyImageImporter"};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerName);
  CORRADE_INTERNAL_ASSERT(importer);
  // set importer flags if gfx logging is quieted
  if (!isLevelEnabled(logging::Subsystem::gfx,
                      logging::LoggingLevel::Warning)) {
    importer->addFlags(Mn::Trade::ImporterFlag::Quiet);
  } else if (isLevelEnabled(logging::Subsystem::gfx,
                            logging::LoggingLevel::VeryVerbose)) {
    // set verbose flags if necessary
    importer->addFlags(Mn::Trade::ImporterFlag::Verbose);
  }

  const Cr::Utility::Resource rs{"pbr-images"};
  importer->openData(rs.getRaw(imageFilename));
  Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
      importer->image2D(0);
  return imageData;
}  // loadImageData

Mn::Matrix4 buildDfltPerspectiveMatrix() {
  using namespace Mn::Math::Literals;
  return Mn::Matrix4::perspectiveProjection(
      90.0_degf,  // horizontal field of view angle
      1.0f,       // aspect ratio (width/height)
      0.01f,      // z-near plane
      1000.0f);   // z-far plane
}  // buildDfltPerspectiveMatrix()

}  // namespace

PbrImageBasedLighting::PbrImageBasedLighting(
    Flags flags,
    ShaderManager& shaderManager,
    const std::string& envmapImageFilename)
    : flags_(flags), shaderManager_(shaderManager) {
  // import the resources (URDF lookup texture, HDRi environment etc.)
  if (!Cr::Utility::Resource::hasGroup("pbr-images")) {
    importPbrImageResources();
  }
  // load the BRDF lookup table (indirect specular part)
  // TODO: should have the capability to compute it by the simulator

  // using the brdflut from here:
  // https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/screenshots/tex_brdflut.png

  Cr::Containers::Optional<Mn::Trade::ImageData2D> blutImageData =
      loadImageData("brdflut_ldr_512x512.png");
  // sanity checks
  CORRADE_INTERNAL_ASSERT(blutImageData);

  loadBrdfLookUpTable(blutImageData);

  // ==== load the equirectangular texture ====
  Cr::Containers::Optional<Mn::Trade::ImageData2D> envMapImageData =
      loadImageData(envmapImageFilename);
  // sanity checks
  CORRADE_INTERNAL_ASSERT(envMapImageData);

  // the image filename must be specified in the Resource
  convertEquirectangularToCubeMap(envMapImageData);

  auto& cubemapTexture =
      environmentMap_->getTexture(CubeMap::TextureType::Color);

  // compute the irradiance map for indirect diffuse part
  computeIrradianceMap(cubemapTexture);

  // compute the prefiltered environment map (indirect specular part)
  computePrefilteredEnvMap(cubemapTexture);
}

template <typename T>
Mn::Resource<Mn::GL::AbstractShaderProgram, T> PbrImageBasedLighting::getShader(
    PbrIblShaderType type) {
  Mn::ResourceKey key;
  switch (type) {
    case PbrIblShaderType::IrradianceMap:
      key = Mn::ResourceKey{"irradianceMap"};
      break;

    case PbrIblShaderType::PrefilteredMap:
      key = Mn::ResourceKey{"prefilteredMap"};
      break;

    case PbrIblShaderType::EquirectangularToCubeMap:
      key = Mn::ResourceKey{"equirectangularToCubeMap"};
      break;

    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
  Mn::Resource<Mn::GL::AbstractShaderProgram, T> shader =
      shaderManager_.get<Mn::GL::AbstractShaderProgram, T>(key);

  if (!shader) {
    if (type == PbrIblShaderType::IrradianceMap) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader.key(),
          new PbrPrecomputedMapShader(PbrPrecomputedMapShader::Flags{
              PbrPrecomputedMapShader::Flag::IrradianceMap}),
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    } else if (type == PbrIblShaderType::EquirectangularToCubeMap) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader.key(), new PbrEquiRectangularToCubeMapShader(),
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    } else if (type == PbrIblShaderType::PrefilteredMap) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader.key(),
          new PbrPrecomputedMapShader(PbrPrecomputedMapShader::Flags{
              PbrPrecomputedMapShader::Flag::PrefilteredMap}),
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }
  }
  CORRADE_INTERNAL_ASSERT(shader);

  return shader;
}

void PbrImageBasedLighting::loadBrdfLookUpTable(
    const Cr::Containers::Optional<Mn::Trade::ImageData2D>& imageData) {
  brdfLUT_ = Mn::GL::Texture2D{};
  (*brdfLUT_)
      .setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::RGBA8, imageData->size());

  if (!imageData->isCompressed()) {
    brdfLUT_->setSubImage(0, {}, *imageData);
  } else {
    brdfLUT_->setCompressedSubImage(0, {}, *imageData);
  }
}  // loadBrdfLookUpTable

void PbrImageBasedLighting::convertEquirectangularToCubeMap(
    const Cr::Containers::Optional<Mn::Trade::ImageData2D>& imageData) {
  Mn::GL::Texture2D tex = Mn::GL::Texture2D{};
  tex.setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::textureFormat(imageData->format()),
                  imageData->size());

  if (!imageData->isCompressed()) {
    tex.setSubImage(0, {}, *imageData);
  } else {
    tex.setCompressedSubImage(0, {}, *imageData);
  }
  // prepare a mesh to be displayed
  Mn::GL::Mesh mesh = Mn::GL::Mesh{};
  mesh.setCount(3);

  // Initialize the base environment map
  environmentMap_ =
      CubeMap(environmentMapSize,
              CubeMap::Flag::ColorTexture | CubeMap::Flag::ManuallyBuildMipmap);

  // prepare the shader
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrEquiRectangularToCubeMapShader>
      shader = getShader<PbrEquiRectangularToCubeMapShader>(
          PbrIblShaderType::EquirectangularToCubeMap);

  // bind the equirectangular texture
  shader->bindEquirectangularTexture(tex);

  // draw the 6 sides one by one
  for (unsigned int iSide = 0; iSide < 6; ++iSide) {
    // bind frambuffer, clear color and depth
    environmentMap_->prepareToDraw(iSide);
    shader->setCubeSideIndex(iSide);
    shader->draw(mesh);
  }
  // do NOT forget to populate to all the mip levels!
  environmentMap_->getTexture(CubeMap::TextureType::Color).generateMipmap();

}  // convertEquirectangularToCubeMap

void PbrImageBasedLighting::computeIrradianceMap(
    Mn::GL::CubeMapTexture& envCubeMap) {
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader> shader =
      getShader<PbrPrecomputedMapShader>(PbrIblShaderType::IrradianceMap);

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

void PbrImageBasedLighting::computePrefilteredEnvMap(
    Mn::GL::CubeMapTexture& envCubeMap) {
  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader> shader =
      getShader<PbrPrecomputedMapShader>(PbrIblShaderType::PrefilteredMap);

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

CubeMap& PbrImageBasedLighting::getIrradianceMap() {
  CORRADE_ASSERT(flags_ & Flag::IndirectDiffuse,
                 "PbrImageBasedLighting::getIrradianceMap(): the instance is "
                 "not created with indirect diffuse part enabled.",
                 *irradianceMap_);

  CORRADE_ASSERT(
      irradianceMap_,
      "PbrImageBasedLighting::getIrradianceMap(): the irradiance map is empty. "
      "Did you forget to load or compute it (from environment map)?",
      *irradianceMap_);

  return *irradianceMap_;
}

CubeMap& PbrImageBasedLighting::getPrefilteredMap() {
  CORRADE_ASSERT(flags_ & Flag::IndirectSpecular,
                 "PbrImageBasedLighting::getPrefilteredMap(): the instance is "
                 "not created with indirect specular part enabled.",
                 *prefilteredMap_);

  CORRADE_ASSERT(prefilteredMap_,
                 "PbrImageBasedLighting::getPrefilteredMap(): the pre-filtered "
                 "cube map is empty."
                 "Did you forget to load or compute it (from environment map)?",
                 *prefilteredMap_);
  return *prefilteredMap_;
}

Mn::GL::Texture2D& PbrImageBasedLighting::getBrdfLookupTable() {
  CORRADE_ASSERT(flags_ & Flag::IndirectSpecular,
                 "PbrImageBasedLighting::getBrdfLookupTable(): the instance is "
                 "not created with indirect specular part enabled.",
                 *brdfLUT_);

  CORRADE_ASSERT(brdfLUT_,
                 "PbrImageBasedLighting::getBrdfLookupTable(): the brdf lookup "
                 "table (a texture) is empty"
                 "Did you forget to load or compute it (from environment map)?",
                 *brdfLUT_);

  return *brdfLUT_;
}

}  // namespace gfx
}  // namespace esp
