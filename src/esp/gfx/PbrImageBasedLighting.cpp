// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrImageBasedLighting.h"

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Resource.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Angle.h>
#include <Magnum/MeshTools/Compile.h>
#include <Magnum/MeshTools/FlipNormals.h>
#include <Magnum/MeshTools/Reference.h>
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
constexpr unsigned int brdfLUTSize = 512;
};  // namespace

PbrImageBasedLighting::PbrImageBasedLighting(
    Flags flags,
    ShaderManager& shaderManager,
    const std::string& hdriImageFilename)
    : flags_(flags), shaderManager_(shaderManager) {
  recreateTextures();

  // import the resources (URDF lookup texture, HDRi environment etc.)
  if (!Cr::Utility::Resource::hasGroup("pbr-images")) {
    importPbrImageResources();
  }
  // the image filename must be specified in the Resource
  convertEquirectangularToCubeMap(hdriImageFilename);

  // compute the irradiance map for indirect diffuse part
  computePrecomputedMap(PrecomputedMapType::IrradianceMap);

  // load the BRDF lookup table (indirect specular part)
  // TODO: should have the capability to compute it by the simulator
  loadBrdfLookUpTable();

  // compute the prefiltered environment map (indirect specular part)
  computePrecomputedMap(PrecomputedMapType::PrefilteredMap);
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

void PbrImageBasedLighting::convertEquirectangularToCubeMap(
    const std::string& hdriImageFilename) {
  // ==== load the equirectangular texture ====
  // TODO: HDR!!
  // plugin manager used to instantiate importers which in turn are used
  // to load image data
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  std::string importerName{"AnyImageImporter"};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerName);
  CORRADE_INTERNAL_ASSERT(importer);

  const Cr::Utility::Resource rs{"pbr-images"};
  importer->openData(rs.getRaw(hdriImageFilename));
  Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
      importer->image2D(0);
  // sanity checks
  CORRADE_INTERNAL_ASSERT(imageData);

  Mn::Vector2i size = imageData->size();
  // TODO: HDR!!
  Mn::GL::Texture2D tex = Mn::GL::Texture2D{};
  tex.setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::textureFormat(imageData->format()),
                  size);  // TODO: HDR

  if (!imageData->isCompressed()) {
    tex.setSubImage(0, {}, *imageData);
  } else {
    tex.setCompressedSubImage(0, {}, *imageData);
  }
  // prepare a mesh to be displayed
  Mn::GL::Mesh mesh = Mn::GL::Mesh{};
  mesh.setCount(3);

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
}

void PbrImageBasedLighting::recreateTextures() {
  // TODO: HDR!!
  Mn::Vector2i size{brdfLUTSize, brdfLUTSize};
  brdfLUT_ = Mn::GL::Texture2D{};
  (*brdfLUT_)
      .setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::RGBA8, size);  // TODO: HDR

  // TODO: HDR!!
  // we do not use build-in function `renderToTexture` in the CubeMap class. So
  // we will have to populate the mipmaps by ourselves in this class.
  environmentMap_ =
      CubeMap(environmentMapSize,
              CubeMap::Flag::ColorTexture | CubeMap::Flag::ManuallyBuildMipmap);
  irradianceMap_ = CubeMap(irradianceMapSize, CubeMap::Flag::ColorTexture);
  prefilteredMap_ =
      CubeMap(prefilteredMapSize,
              CubeMap::Flag::ColorTexture | CubeMap::Flag::ManuallyBuildMipmap);
}

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

void PbrImageBasedLighting::loadBrdfLookUpTable() {
  // plugin manager used to instantiate importers which in turn are used
  // to load image data
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  std::string importerName{"AnyImageImporter"};
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerName);
  CORRADE_INTERNAL_ASSERT(importer);

  // TODO: HDR, No LDR in the future!
  // temporarily using the brdflut from here:
  // https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/screenshots/tex_brdflut.png
  const std::string brdflutFilename = "brdflut_ldr_512x512.png";

  // this is not the file name, but the group name in the config file
  // see PbrImages.conf in the shaders folder
  const Cr::Utility::Resource rs{"pbr-images"};
  importer->openData(rs.getRaw(brdflutFilename));

  Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
      importer->image2D(0);
  // sanity checks
  CORRADE_INTERNAL_ASSERT(imageData);

  brdfLUT_->setSubImage(0, {}, *imageData);
}

void PbrImageBasedLighting::computePrecomputedMap(PrecomputedMapType type) {
  CORRADE_ASSERT(
      environmentMap_,
      "PbrImageBasedLighting::computePrecomputedMap(): the environment "
      "map cannot be found. Have you loaded it? ", );

  if (type == PrecomputedMapType::IrradianceMap) {
    CORRADE_ASSERT(
        irradianceMap_,
        "PbrImageBasedLighting::computePrecomputedMap(): the irradiance map "
        "is empty (not initialized).", );
  } else {
    CORRADE_ASSERT(
        prefilteredMap_,
        "PbrImageBasedLighting::computePrecomputedMap(): the prefiltered map "
        "is empty (not initialized).", );
  }

  Mn::Resource<Mn::GL::AbstractShaderProgram, PbrPrecomputedMapShader> shader =
      type == PrecomputedMapType::IrradianceMap
          ? getShader<PbrPrecomputedMapShader>(PbrIblShaderType::IrradianceMap)
          : getShader<PbrPrecomputedMapShader>(
                PbrIblShaderType::PrefilteredMap);

  // TODO: HDR!!
  shader->bindEnvironmentMap(
      environmentMap_->getTexture(CubeMap::TextureType::Color));

  // NOLINTNEXTLINE(google-build-using-namespace)
  using namespace Mn::Math::Literals;
  shader->setProjectionMatrix(Mn::Matrix4::perspectiveProjection(
      90.0_degf,  // horizontal field of view angle
      1.0f,       // aspect ratio (width/height)
      0.01f,      // z-near plane
      1000.0f));  // z-far plane

  // prepare a cube
  Mn::Trade::MeshData cubeData =
      Mn::MeshTools::owned(Mn::Primitives::cubeSolid());
  // camera is now inside the cube, must flip the face winding, otherwise all
  // the faces are culled
  Mn::MeshTools::flipFaceWindingInPlace(cubeData.mutableIndices());
  Magnum::GL::Mesh cube = Magnum::MeshTools::compile(cubeData);

  if (type == PrecomputedMapType::IrradianceMap) {
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
  } else {
    // compute prefiltered map
    unsigned int maxMipLevels = prefilteredMap_->getMipmapLevels();
    CORRADE_INTERNAL_ASSERT(maxMipLevels ==
                            Mn::Math::log2(prefilteredMapSize) + 1);
    for (unsigned int iMip = 0; iMip < maxMipLevels; ++iMip) {
      float roughness = float(iMip) / float(maxMipLevels - 1);

      shader->setRoughness(roughness);

      for (unsigned int jSide = 0; jSide < 6; ++jSide) {
        Mn::Matrix4 viewMatrix = CubeMapCamera::getCameraLocalTransform(
                                     CubeMapCamera::cubeMapCoordinate(jSide))
                                     .inverted();
        shader->setTransformationMatrix(viewMatrix);
        // bind framebuffer, clear color and depth
        prefilteredMap_->prepareToDraw(
            jSide,
            RenderCamera::Flag::ClearColor | RenderCamera::Flag::ClearDepth,
            iMip);
        shader->draw(cube);
      }  // for jSide
    }    // for iMip
  }      // else
}

}  // namespace gfx
}  // namespace esp
