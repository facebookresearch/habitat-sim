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
#include <Magnum/GL/Renderer.h>
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

void CubeMap::enableSeamlessCubeMapTexture() {
#ifndef MAGNUM_TARGET_WEBGL
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::SeamlessCubeMapTexture);
#endif
}

CubeMap::CubeMap(int imageSize, Flags flags) : flags_(flags) {
  reset(imageSize);
}

bool CubeMap::reset(int imageSize) {
  if (imageSize_ == imageSize) {
    return false;
  }

  imageSize_ = imageSize;
  CORRADE_ASSERT(imageSize_ > 0,
                 "CubeMap::reset(): image size" << imageSize << "is illegal.",
                 false);
  // create an empty cubemap texture
  recreateTexture();

  // prepare frame buffer and render buffer
  recreateFramebuffer();

  return true;
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
        .setMagnificationFilter(Mn::GL::SamplerFilter::Linear);

    if (flags_ & Flag::BuildMipMap) {
      (*colorTexture)
          .setStorage(Mn::Math::log2(imageSize_) + 1,
                      Mn::GL::TextureFormat::RGBA8, size);
    } else {
      (*colorTexture).setStorage(1, Mn::GL::TextureFormat::RGBA8, size);
    }
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

Mn::GL::CubeMapCoordinate CubeMap::convertFaceIndexToCubeMapCoordinate(
    int faceIndex) {
  CORRADE_ASSERT(faceIndex >= 0 && faceIndex < 6,
                 "CubeMap::convertFaceIndexToCubeMapCoordinate(): the index of "
                 "the cube side"
                     << faceIndex << "is illegal.",
                 Mn::GL::CubeMapCoordinate::PositiveX);
  return Mn::GL::CubeMapCoordinate(int(Mn::GL::CubeMapCoordinate::PositiveX) +
                                   faceIndex);
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
      break;
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
#ifdef MAGNUM_TARGET_WEBGL
  // because Mn::Image2D image = textures_[type]->image(...)
  // requires desktop OpenGL
  return false;
#else
  textureTypeSanityCheck(type, "CubeMap::saveTexture():");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> converter;
  if (!(converter = manager.loadAndInstantiate("AnyImageConverter"))) {
    return false;
  }

  std::string coordStrings[6] = {".+X", ".-X", ".+Y", ".-Y", ".+Z", ".-Z"};
  for (int iFace = 0; iFace < 6; ++iFace) {
    Mn::Image2D image = textures_[type]->image(
        convertFaceIndexToCubeMapCoordinate(iFace), 0, {getPixelFormat(type)});

    std::string filename;
    switch (type) {
      case TextureType::Color: {
        filename = imageFilePrefix + getTextureTypeFilenameString(type) +
                   coordStrings[iFace] + std::string{".png"};
      } break;

      case TextureType::Depth: {
        filename = imageFilePrefix + getTextureTypeFilenameString(type) +
                   coordStrings[iFace] + std::string{".hdr"};
      } break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }

    if (!converter->exportToFile(image, filename)) {
      return false;
    } else {
      LOG(INFO) << "Saved image " << iFace << " to " << filename;
    }
  }

  return true;
#endif
}

void CubeMap::renderToTexture(CubeMapCamera& camera,
                              scene::SceneGraph& sceneGraph,
                              RenderCamera::Flags flags) {
  CORRADE_ASSERT(camera.isInSceneGraph(sceneGraph),
                 "CubeMap::renderToTexture(): camera is NOT attached to the "
                 "current scene graph.", );
  // ==== projection matrix ====
  // CAREFUL! In this function the projection matrix of the camera is assumed to
  // be set already outside of this function by the user.
  // we simply do sanity check here.
  {
    Mn::Vector2i vp = camera.viewport();
    CORRADE_ASSERT(vp.x() == vp.y() && vp.x() == imageSize_,
                   "CubeMap::renderToTexture(): the image size with in the "
                   "CubeMapCamera, which is"
                       << vp.x() << "by" << vp.y() << "compared to"
                       << imageSize_ << "is not correct.", );
  }

  // ==== camera matrix ====
  // In case user set the relative transformation of
  // the camera node before calling this function, original viewing matrix of
  // the camera MUST be updated as well.
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

  // CAREFUL!!!
  // switchToFace() will change the local transformation of this camera node!
  // If you do not do anything, in next rendering cycle, since
  // camera.updateOriginalViewingMatrix() is called, the original viewing matrix
  // will be updated by mistake!!! To prevent such mistakes, local
  // transformation of this camera node must be reset.
  camera.restoreTransformation();

  // Color texture ONLY, NOT for depth
  if (flags_ & (Flag::BuildMipMap | Flag::ColorTexture)) {
    textures_[TextureType::Color]->generateMipmap();
  }
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
  } else if (imageFileExtension == "hdr") {
    plugin = "HdrImporter";
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
      CORRADE_ASSERT(size.x() == imageSize,
                     " CubeMap::loadTexture(): texture images must have the "
                     "same size.", );
    }

    texture->setSubImage(convertFaceIndexToCubeMapCoordinate(iFace), 0, {},
                         *image);
  }
  // Color texture ONLY, NOT for depth
  if (flags_ & (Flag::BuildMipMap | Flag::ColorTexture)) {
    texture->generateMipmap();
  }
  // We don't need the importer anymore
  imageImporterManger_.free<Mn::Trade::AbstractImporter>();
}

}  // namespace gfx
}  // namespace esp
