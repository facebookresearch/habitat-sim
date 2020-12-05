// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree
#include "CubeMap.h"
#include <Corrade/Containers/Optional.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Assert.h>
#include <Magnum/DebugTools/TextureImage.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Shaders/Generic.h>
#include <Magnum/Trade/AbstractImageConverter.h>
#include <Magnum/Trade/ImageData.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

const Mn::GL::Framebuffer::ColorAttachment colorAttachment =
    Mn::GL::Framebuffer::ColorAttachment{0};

// TODO:
// const Mn::GL::Framebuffer::ColorAttachment objectIdAttachment =
//    Mn::GL::Framebuffer::ColorAttachment{1};

CubeMap::CubeMap(int imageSize, Flags flags) : flags_(flags) {
  reset(imageSize);
}

void CubeMap::reset(int imageSize) {
  if (imageSize_ == imageSize) {
    return;
  }

  imageSize_ = imageSize;
  CORRADE_ASSERT(
      imageSize_ > 0,
      "CubeMap::reset(): image size" << imageSize << "is illegal.", );
  // create an empty cubemap texture
  recreateTexture();

  // prepare frame buffer and render buffer
  recreateFramebuffer();
}

void CubeMap::recreateTexture() {
  Mn::Vector2i size{imageSize_, imageSize_};

  // color texture
  if (flags_ & Flag::ColorTexture) {
    auto& colorTexture = textures_[TextureType::Color];
    colorTexture = std::make_unique<Mn::GL::CubeMapTexture>();
    (*colorTexture)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Linear,
                               Mn::GL::SamplerMipmap::Linear)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
        .setStorage(Mn::Math::log2(imageSize_) + 1,
                    Mn::GL::TextureFormat::RGBA8, size);
  }

  // depth texture
  if (flags_ & Flag::DepthTexture) {
    auto& depthTexture = textures_[TextureType::Depth];
    depthTexture = std::make_unique<Mn::GL::CubeMapTexture>();
    (*depthTexture)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, size);
  }
}

void CubeMap::recreateFramebuffer() {
  Mn::Vector2i viewportSize{imageSize_, imageSize_};
  frameBuffer_ = Mn::GL::Framebuffer{{{}, viewportSize}};
  optionalDepthBuffer_.setStorage(Mn::GL::RenderbufferFormat::DepthComponent24,
                                  viewportSize);
}

Magnum::GL::CubeMapCoordinate CubeMap::convertFaceIndexToCubeMapCoordinate(
    int faceIndex) {
  CORRADE_ASSERT(faceIndex >= 0 && faceIndex < 6,
                 "CubeMap::convertFaceIndexToCubeMapCoordinate(): the index of "
                 "the cube side"
                     << faceIndex << "is illegal.",
                 Mn::GL::CubeMapCoordinate::PositiveX);
  switch (faceIndex) {
    case 0:
      return Mn::GL::CubeMapCoordinate::PositiveX;
      break;
    case 1:
      return Mn::GL::CubeMapCoordinate::NegativeX;
      break;
    case 2:
      return Mn::GL::CubeMapCoordinate::PositiveY;
      break;
    case 3:
      return Mn::GL::CubeMapCoordinate::NegativeY;
      break;
    case 4:
      return Mn::GL::CubeMapCoordinate::PositiveZ;
      break;
    case 5:
      return Mn::GL::CubeMapCoordinate::NegativeZ;
      break;
    default:  // never reach, just avoid compiler warning
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
}

void CubeMap::prepareToDraw(int cubeSideIndex) {
  Magnum::GL::CubeMapCoordinate cubeMapCoord =
      convertFaceIndexToCubeMapCoordinate(cubeSideIndex);

  if (flags_ & Flag::ColorTexture) {
    frameBuffer_.attachCubeMapTexture(
        colorAttachment, *textures_[TextureType::Color], cubeMapCoord, 0);
  }

  if (flags_ & Flag::DepthTexture) {
    frameBuffer_.attachCubeMapTexture(
        Mn::GL::Framebuffer::BufferAttachment::Depth,
        *textures_[TextureType::Depth], cubeMapCoord, 0);
  } else {
    frameBuffer_.attachRenderbuffer(
        Mn::GL::Framebuffer::BufferAttachment::Depth, optionalDepthBuffer_);
  }

  mapForDraw();

  frameBuffer_.clearDepth(1.0f).clearColor(1, Mn::Vector4ui{0});

  CORRADE_INTERNAL_ASSERT(
      frameBuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
      Mn::GL::Framebuffer::Status::Complete);
}

void CubeMap::mapForDraw() {
  frameBuffer_.mapForDraw({
      {Mn::Shaders::Generic3D::ColorOutput, colorAttachment},
      // TODO:
      //{Mn::Shaders::Generic3D::ObjectIdOutput, objectIdAttachment}
  });
}
void CubeMap::textureTypeSanityCheck(TextureType type,
                                     const std::string& functionNameStr) {
  // sanity check
  switch (type) {
    case TextureType::Color:
      CORRADE_ASSERT(flags_ & Flag::ColorTexture,
                     functionNameStr.c_str()
                         << "instance was not created with color "
                            "texture output enabled.", );
      break;
    case TextureType::Depth:
      CORRADE_ASSERT(flags_ & Flag::DepthTexture,
                     functionNameStr.c_str()
                         << "instance was not created with depth "
                            "texture output enabled.", );
      break;
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
}

Mn::GL::CubeMapTexture& CubeMap::getTexture(TextureType type) {
  textureTypeSanityCheck(type, "CubeMap::getTexture():");
  return *textures_[type];
}

std::string CubeMap::getTextureTypeFilenameString(TextureType type) {
  switch (type) {
    case TextureType::Color:
      return std::string(".rgba");
      break;
    case TextureType::Depth:
      return std::string(".depth");
      break;
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
}
Mn::PixelFormat CubeMap::getPixelFormat(TextureType type) {
  switch (type) {
    case TextureType::Color:
      return Mn::PixelFormat::RGBA8Unorm;
      break;
    case TextureType::Depth:
      return Mn::PixelFormat::R32F;
      /*
      case TextureType::ObjectId:
      return Mn::PixelFormat::R32UI;
      */
    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }
}

bool CubeMap::saveTexture(TextureType type,
                          const std::string& imageFilePrefix) {
  textureTypeSanityCheck(type, "CubeMap::saveTexture():");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> converter;
  if (!(converter = manager.loadAndInstantiate("AnyImageConverter"))) {
    return false;
  }

  std::string coordStrings[6] = {".+X", ".-X", ".+Y", ".-Y", ".+Z", ".-Z"};
  for (int iFace = 0; iFace < 6; ++iFace) {
    Mn::Image2D image = Mn::DebugTools::textureSubImage(
        *textures_[type], convertFaceIndexToCubeMapCoordinate(iFace), 0,
        frameBuffer_.viewport(), {getPixelFormat(type)});

    std::string filename = imageFilePrefix +
                           getTextureTypeFilenameString(type) +
                           coordStrings[iFace] + std::string{".png"};

    if (!converter->exportToFile(image, filename)) {
      return false;
    } else {
      LOG(INFO) << "Saved image " << iFace << " to " << filename;
    }
  }

  return true;
}

void CubeMap::renderToTexture(CubeMapCamera& camera,
                              scene::SceneGraph& sceneGraph,
                              RenderCamera::Flags flags) {
  CORRADE_ASSERT(camera.isInSceneGraph(sceneGraph),
                 "CubeMap::renderToTexture: camera is NOT attached to the "
                 "current scene graph.", );
  camera.setProjectionMatrix(imageSize_,
                             0.001f,    // z-near
                             1000.0f);  // z-far
  // In case user set the relative transformation of the camera node before
  // calling this function, original viewing matrix of the camera MUST be
  // updated as well.
  camera.updateOriginalViewingMatrix();
  frameBuffer_.bind();
  for (int iFace = 0; iFace < 6; ++iFace) {
    camera.switchToFace(iFace);
    prepareToDraw(iFace);

    // TODO:
    // camera should have flags so that it can do "low quality" rendering,
    // e.g., no normal maps, no specular lighting, low-poly meshes,
    // low-quality textures.

    for (auto& it : sceneGraph.getDrawableGroups()) {
      // TODO: remove || true
      if (it.second.prepareForDraw(camera) || true) {
        camera.draw(it.second, flags);
      }
    }
  }  // iFace
}

void CubeMap::loadTexture(TextureType type,
                          const std::string& imageFilePrefix,
                          const std::string& imageFileExtension) {
  auto loadImporter = [&](const std::string& plugin) {
    Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
    Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
        manager.loadAndInstantiate(plugin);
    CORRADE_ASSERT(importer,
                   "CubeMap::loadTexture(): cannot initialize" << plugin, );

    imageImporterManger_.set<Mn::Trade::AbstractImporter>(
        plugin, importer.release(), Mn::ResourceDataState::Final,
        Mn::ResourcePolicy::Manual);
  };

  std::string plugin;
  if (imageFileExtension == "png") {
    plugin = "PngImporter";
  } else if (imageFileExtension == "jpg") {
    plugin = "JpegImporter";
  } else {
    LOG(INFO) << "CubeMap::loadTexture(): cannot support image type "
              << imageFileExtension;
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  }

  Mn::Resource<Mn::Trade::AbstractImporter> importer =
      imageImporterManger_.get<Mn::Trade::AbstractImporter>(plugin);
  // create one if there is no such importer
  if (!importer) {
    loadImporter(plugin);
  }
  CORRADE_INTERNAL_ASSERT(importer);

  std::string coordStrings[6] = {".+X", ".-X", ".+Y", ".-Y", ".+Z", ".-Z"};
  int imageSize = 0;
  for (int iFace = 0; iFace < 6; ++iFace) {
    // open image file
    std::string filename =
        imageFilePrefix + getTextureTypeFilenameString(type) +
        coordStrings[iFace] + "." + std::string{imageFileExtension};
    importer->openFile(filename);
    Cr::Containers::Optional<Mn::Trade::ImageData2D> image =
        importer->image2D(0);

    // sanity checks
    CORRADE_INTERNAL_ASSERT(image);
    Mn::Vector2i size = image->size();
    CORRADE_ASSERT(
        size.x() == size.y(),
        " CubeMap::loadTexture(): each texture image must be a square.", );
    if (iFace == 0) {
      imageSize = size.x();
      reset(imageSize);
    } else {
      CORRADE_ASSERT(
          size.x() == imageSize,
          " CubeMap::loadTexture(): texture images must have the same size.", );
    }

    // set images
    Mn::GL::CubeMapTexture* texture;
    switch (type) {
      case TextureType::Color:
        texture = textures_[TextureType::Color].get();
        break;

      case TextureType::Depth:
        texture = textures_[TextureType::Depth].get();
        break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
    texture->setSubImage(convertFaceIndexToCubeMapCoordinate(iFace), 0, {},
                         *image);
  }

  // We don't need the importer anymore
  imageImporterManger_.free<Mn::Trade::AbstractImporter>();
}

}  // namespace gfx
}  // namespace esp
