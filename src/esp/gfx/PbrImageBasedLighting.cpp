// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrImageBasedLighting.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Resource.h>

#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/ImageView.h>
#include <Magnum/Magnum.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>

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
PbrImageBasedLighting::PbrImageBasedLighting(Flags flags,
                                             ShaderManager& shaderManager)
    : flags_(flags), shaderManager_(shaderManager) {
  recreateTextures();
  loadBrdfLookUpTable();

  // load environment map
  environmentMap_->loadTexture(CubeMap::TextureType::Color, "skybox", "png");
}

void PbrImageBasedLighting::recreateTextures() {
  // TODO: HDR!!
  Mn::Vector2i size{512, 512};
  brdfLUT_ = Mn::GL::Texture2D{};
  (*brdfLUT_)
      .setMinificationFilter(Mn::GL::SamplerFilter::Linear)
      .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
      .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
      .setStorage(1, Mn::GL::TextureFormat::RGBA8, size);  // TODO: HDR

  // TODO: HDR!!
  environmentMap_ = CubeMap(1024, {CubeMap::Flag::ColorTexture});
  irradianceMap_ = CubeMap(128, {CubeMap::Flag::ColorTexture});
  prefilteredMap_ = CubeMap(1024, {CubeMap::Flag::ColorTexture});
}

CubeMap& PbrImageBasedLighting::getIrradianceMap() {
  CORRADE_ASSERT(flags_ & Flag::IndirectDiffuse,
                 "PbrImageBasedLighting::getIrradianceMap(): the instance is "
                 "not created with indirect diffuse part enabled.",
                 *irradianceMap_);

  CORRADE_ASSERT(
      irradianceMap_ != Cr::Containers::NullOpt,
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

  CORRADE_ASSERT(prefilteredMap_ != Cr::Containers::NullOpt,
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

  CORRADE_ASSERT(brdfLUT_ != Cr::Containers::NullOpt,
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
  std::string filename = "/data/pbr/brdflut_ldr_512x512.png";

  importer->openFile(filename);
  Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
      importer->image2D(0);
  // sanity checks
  CORRADE_INTERNAL_ASSERT(imageData);

  brdfLUT_->setSubImage(0, {}, *imageData);
}
}  // namespace gfx
}  // namespace esp
