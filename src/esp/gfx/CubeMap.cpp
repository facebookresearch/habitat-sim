// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree
#include "CubeMap.h"
#include <Corrade/Containers/ArrayView.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Corrade/Containers/StringStl.h>
#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/Algorithms.h>
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/DebugTools/TextureImage.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Shaders/GenericGL.h>
#include <Magnum/Trade/AbstractImageConverter.h>
#include <Magnum/Trade/ImageData.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

const Mn::GL::Framebuffer::ColorAttachment rgbaAttachment =
    Mn::GL::Framebuffer::ColorAttachment{0};
const Mn::GL::Framebuffer::ColorAttachment objectIdAttachment =
    Mn::GL::Framebuffer::ColorAttachment{1};

/**
 * @brief check if the class instance is created with corresponding texture
 * enabled
 */
void textureTypeSanityCheck(CubeMap::Flags& flag,
                            CubeMap::TextureType type,
                            const std::string& functionNameStr) {
  switch (type) {
    case CubeMap::TextureType::Color:
      CORRADE_ASSERT(flag & CubeMap::Flag::ColorTexture,
                     functionNameStr.c_str()
                         << "instance was not created with color "
                            "texture output enabled.", );
      return;
      break;
    case CubeMap::TextureType::Depth:
      CORRADE_ASSERT(flag & CubeMap::Flag::DepthTexture,
                     functionNameStr.c_str()
                         << "instance was not created with depth "
                            "texture output enabled.", );
      return;
    case CubeMap::TextureType::ObjectId:
      CORRADE_ASSERT(flag & CubeMap::Flag::ObjectIdTexture,
                     functionNameStr.c_str()
                         << "instance was not created with object id"
                            "texture output enabled.", );
      return;
      break;

    case CubeMap::TextureType::Count:
      break;
  }
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

/**
 * @brief convert cube face index to Magnum::GL::CubeMapCoordinate
 */
Magnum::GL::CubeMapCoordinate convertFaceIndexToCubeMapCoordinate(
    unsigned int faceIndex) {
  CORRADE_ASSERT(
      faceIndex < 6,
      "In CubeMap: ConvertFaceIndexToCubeMapCoordinate(): the index of "
      "the cube side"
          << faceIndex << "is illegal.",
      Mn::GL::CubeMapCoordinate::PositiveX);
  return Mn::GL::CubeMapCoordinate(int(Mn::GL::CubeMapCoordinate::PositiveX) +
                                   faceIndex);
}

/**
 * @brief get texture type string for texture filename
 */
const char* getTextureTypeFilenameString(CubeMap::TextureType type) {
  switch (type) {
    case CubeMap::TextureType::Color:
      return "rgba";
      break;
    case CubeMap::TextureType::Depth:
      return "depth";
      break;
    case CubeMap::TextureType::ObjectId:
      return "objectId";
      break;
    case CubeMap::TextureType::Count:
      break;
  }
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

/**
 * @brief get the pixel format based on texture type (color, depth objectId
 * etc.)
 */
Mn::PixelFormat getPixelFormat(CubeMap::TextureType type) {
  switch (type) {
    case CubeMap::TextureType::Color:
      return Mn::PixelFormat::RGBA8Unorm;
      break;
    case CubeMap::TextureType::Depth:
      return Mn::PixelFormat::R32F;
      break;
    case CubeMap::TextureType::ObjectId:
      return Mn::PixelFormat::R32UI;
      break;
    case CubeMap::TextureType::Count:
      break;
  }
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

Magnum::GL::CubeMapTexture& CubeMap::texture(TextureType type) {
  CORRADE_INTERNAL_ASSERT(uint8_t(type) < Cr::Containers::arraySize(textures_));
  return textures_[uint8_t(type)];
}

CubeMap::CubeMap(int imageSize, Flags flags) : flags_(flags) {
#ifndef MAGNUM_TARGET_WEBGL
  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::SeamlessCubeMapTexture);
#endif
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

  // attach render buffer to frame buffer
  attachFramebufferRenderbuffer();

  return true;
}

void CubeMap::recreateTexture() {
  Mn::Vector2i size{imageSize_, imageSize_};

  // color texture
  if (flags_ & Flag::ColorTexture) {
    auto& colorTexture = texture(TextureType::Color);
    colorTexture = Mn::GL::CubeMapTexture{};
    colorTexture.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Linear,
                               Mn::GL::SamplerMipmap::Linear)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Linear);

    if (flags_ & Flag::BuildMipmap) {
      // RGBA8 is for the LDR. Use RGBA16F for the HDR (TODO)
      colorTexture.setStorage(Mn::Math::log2(imageSize_) + 1,
                              Mn::GL::TextureFormat::RGBA8, size);
    } else {
      colorTexture.setStorage(1, Mn::GL::TextureFormat::RGBA8, size);
    }
  }

  // depth texture
  if (flags_ & Flag::DepthTexture) {
    auto& depthTexture = texture(TextureType::Depth);
    depthTexture = Mn::GL::CubeMapTexture{};
    depthTexture.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, size);
  }

  // object id texture
  if (flags_ & Flag::ObjectIdTexture) {
    auto& objectIdTexture = texture(TextureType::ObjectId);
    objectIdTexture = Mn::GL::CubeMapTexture{};
    objectIdTexture.setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setStorage(1, Mn::GL::TextureFormat::R32UI, size);
  }
}

void CubeMap::recreateFramebuffer() {
  Mn::Vector2i viewportSize{imageSize_, imageSize_};
  for (int iFbo = 0; iFbo < 6; ++iFbo) {
    frameBuffer_[iFbo] = Mn::GL::Framebuffer{{{}, viewportSize}};
  }

  // optional depth buffer is 24-bit integer pixel, which is different from the
  // depth texture (32-bit float)
  if (!(flags_ & CubeMap::Flag::DepthTexture)) {
    for (int index = 0; index < 6; ++index) {
      optionalDepthBuffer_[index] = Mn::GL::Renderbuffer{};
      optionalDepthBuffer_[index].setStorage(
          Mn::GL::RenderbufferFormat::DepthComponent24, viewportSize);
    }
  }
}

void CubeMap::attachFramebufferRenderbuffer() {
  for (unsigned int index = 0; index < 6; ++index) {
    Magnum::GL::CubeMapCoordinate cubeMapCoord =
        convertFaceIndexToCubeMapCoordinate(index);

    if (flags_ & Flag::ColorTexture) {
      frameBuffer_[index].attachCubeMapTexture(
          rgbaAttachment, texture(TextureType::Color), cubeMapCoord, 0);
    }

    if (flags_ & Flag::DepthTexture) {
      frameBuffer_[index].attachCubeMapTexture(
          Mn::GL::Framebuffer::BufferAttachment::Depth,
          texture(TextureType::Depth), cubeMapCoord, 0);
    } else {
      frameBuffer_[index].attachRenderbuffer(
          Mn::GL::Framebuffer::BufferAttachment::Depth,
          optionalDepthBuffer_[index]);
    }

    if (flags_ & Flag::ObjectIdTexture) {
      frameBuffer_[index].attachCubeMapTexture(
          objectIdAttachment, texture(TextureType::ObjectId), cubeMapCoord, 0);
    }
  }
}

void CubeMap::prepareToDraw(unsigned int cubeSideIndex,
                            RenderCamera::Flags renderCameraFlags) {
  // Note: we ONLY need to map shader output to color attachment when necessary,
  // which means in depth texture mode, we do NOT need to do this
  if (flags_ & CubeMap::Flag::ColorTexture ||
      flags_ & CubeMap::Flag::ObjectIdTexture) {
    mapForDraw(cubeSideIndex);
  }

  if (flags_ & CubeMap::Flag::ColorTexture &&
      renderCameraFlags & RenderCamera::Flag::ClearColor) {
    frameBuffer_[cubeSideIndex].clearColor(
        0,                      // color attachment
        Mn::Color4{0, 0, 0, 1}  // clear color
    );
  }

  if (renderCameraFlags & RenderCamera::Flag::ClearDepth) {
    frameBuffer_[cubeSideIndex].clearDepth(1.0f);
  }

  if (flags_ & CubeMap::Flag::ObjectIdTexture &&
      renderCameraFlags & RenderCamera::Flag::ClearObjectId) {
    frameBuffer_[cubeSideIndex].clearColor(1, Mn::Vector4ui{});
  }

  CORRADE_INTERNAL_ASSERT(frameBuffer_[cubeSideIndex].checkStatus(
                              Mn::GL::FramebufferTarget::Draw) ==
                          Mn::GL::Framebuffer::Status::Complete);
}

void CubeMap::mapForDraw(unsigned int index) {
  frameBuffer_[index].mapForDraw(
      {{Mn::Shaders::GenericGL3D::ColorOutput,
        (flags_ & CubeMap::Flag::ColorTexture
             ? rgbaAttachment
             : Mn::GL::Framebuffer::DrawAttachment::None)},
       {Mn::Shaders::GenericGL3D::ObjectIdOutput,
        (flags_ & CubeMap::Flag::ObjectIdTexture
             ? objectIdAttachment
             : Mn::GL::Framebuffer::DrawAttachment::None)}});
}

Mn::GL::CubeMapTexture& CubeMap::getTexture(TextureType type) {
  textureTypeSanityCheck(flags_, type, "CubeMap::getTexture():");
  return textures_[static_cast<uint8_t>(type)];
}

#ifndef MAGNUM_TARGET_WEBGL
// because Mn::Image2D image = textures_[type]->image(...)
// requires desktop OpenGL
bool CubeMap::saveTexture(TextureType type,
                          const std::string& imageFilePrefix) {
  textureTypeSanityCheck(flags_, type, "CubeMap::saveTexture():");

  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> converter;
  if (!(converter = manager.loadAndInstantiate("AnyImageConverter"))) {
    return false;
  }

  const char* coordStrings[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  for (int iFace = 0; iFace < 6; ++iFace) {
    std::string filename = "";
    auto& tex = texture(type);
    switch (type) {
      case TextureType::Color: {
        Mn::Image2D image =
            tex.image(convertFaceIndexToCubeMapCoordinate(iFace), 0,
                      {getPixelFormat(type)});
        filename = Cr::Utility::formatString("{}.{}.{}.png", imageFilePrefix,
                                             getTextureTypeFilenameString(type),
                                             coordStrings[iFace]);
        if (!converter->convertToFile(image, filename)) {
          return false;
        }
      } break;

      case TextureType::Depth: {
        filename = Cr::Utility::formatString("{}.{}.{}.hdr", imageFilePrefix,
                                             getTextureTypeFilenameString(type),
                                             coordStrings[iFace]);
        Mn::Image2D image = tex.image(
            convertFaceIndexToCubeMapCoordinate(iFace), 0,
            {Mn::GL::PixelFormat::DepthComponent, Mn::GL::PixelType::Float});
        Mn::ImageView2D depthAsRedChannel{
            image.storage(), Mn::PixelFormat::R32F, image.size(), image.data()};
        if (!converter->convertToFile(depthAsRedChannel, filename)) {
          return false;
        }
      } break;
      /*
      case CubeMap::TextureType::ObjectId:
        // TODO: save object Id texture
        break;
        */
      case CubeMap::TextureType::Count:
        break;

      default:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }
    CORRADE_ASSERT(!filename.empty(),
                   "CubeMap::saveTexture(): Unknown texture type.", false);

    LOG(INFO) << "Saved image " << iFace << " to " << filename;
  }  // for

  return true;
}
#endif

void CubeMap::loadTexture(TextureType type,
                          const std::string& imageFilePrefix,
                          const std::string& imageFileExtension) {
  textureTypeSanityCheck(flags_, type, "CubeMap::loadTexture():");

  // set the alias of the texture
  Mn::GL::CubeMapTexture& tex = texture(type);

  // plugin manager used to instantiate importers which in turn are used
  // to load image data
  Cr::PluginManager::Manager<Mn::Trade::AbstractImporter> manager;
  std::string importerName{"AnyImageImporter"};
  if (type == TextureType::Depth) {
    importerName = "StbImageImporter";
  }
  Cr::Containers::Pointer<Mn::Trade::AbstractImporter> importer =
      manager.loadAndInstantiate(importerName);
  CORRADE_INTERNAL_ASSERT(importer);
  if (type == TextureType::Depth) {
    // Override image channel count. Because depth info should be R32F
    importer->configuration().setValue("forceChannelCount", 1);
  }

  const char* coordStrings[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  int imageSize = 0;

  for (int iFace = 0; iFace < 6; ++iFace) {
    // open image file

    std::string filename = Cr::Utility::formatString(
        "{}.{}.{}.{}", imageFilePrefix, getTextureTypeFilenameString(type),
        coordStrings[iFace], imageFileExtension);

    importer->openFile(filename);
    Cr::Containers::Optional<Mn::Trade::ImageData2D> imageData =
        importer->image2D(0);

    // sanity checks
    CORRADE_INTERNAL_ASSERT(imageData);
    Mn::Vector2i size = imageData->size();
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

    switch (type) {
      case TextureType::Color:
        tex.setSubImage(convertFaceIndexToCubeMapCoordinate(iFace), 0, {},
                        *imageData);
        break;

      case TextureType::Depth: {
        CORRADE_INTERNAL_ASSERT(imageData->format() == Mn::PixelFormat::R32F);
        Mn::ImageView2D imageView(Mn::GL::PixelFormat::DepthComponent,
                                  Mn::GL::PixelType::Float, imageData->size(),
                                  imageData->data());
        tex.setSubImage(convertFaceIndexToCubeMapCoordinate(iFace), 0, {},
                        imageView);
      } break;

      case TextureType::ObjectId:
        // TODO: object Id texture
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;

      case TextureType::Count:
        CORRADE_INTERNAL_ASSERT_UNREACHABLE();
        break;
    }  // switch
    LOG(INFO) << "Loaded image " << iFace << " from " << filename;
  }
  // Color texture ONLY, NOT for depth
  if ((flags_ & Flag::BuildMipmap) && (flags_ & Flag::ColorTexture)) {
    tex.generateMipmap();
  }
}

void CubeMap::renderToTexture(CubeMapCamera& camera,
                              scene::SceneGraph& sceneGraph,
                              RenderCamera::Flags renderCameraFlags) {
  CORRADE_ASSERT(camera.isInSceneGraph(sceneGraph),
                 "CubeMap::renderToTexture(): camera is NOT attached to the "
                 "current scene graph.", );
  // ==== projection matrix ====
  // CAREFUL! In this function the projection matrix of the camera is assumed to
  // be set already outside of this function by the user.
  // we simply do sanity check here.
  {
    Mn::Vector2i vp = camera.viewport();
    CORRADE_ASSERT(vp == Mn::Vector2i{imageSize_},
                   "CubeMap::renderToTexture(): the image size with in the "
                   "CubeMapCamera, which is"
                       << vp << "compared to" << imageSize_
                       << "is not correct.", );
  }

  // ==== camera matrix ====
  // In case user set the relative transformation of
  // the camera node before calling this function, original viewing matrix of
  // the camera MUST be updated as well.
  camera.updateOriginalViewingMatrix();
  for (int iFace = 0; iFace < 6; ++iFace) {
    frameBuffer_[iFace].bind();
    camera.switchToFace(iFace);
    prepareToDraw(iFace, renderCameraFlags);

    // TODO:
    // camera should have renderCameraFlags so that it can do "low quality"
    // rendering, e.g., no normal maps, no specular lighting, low-poly meshes,
    // low-quality textures.

    for (auto& it : sceneGraph.getDrawableGroups()) {
      if (it.second.prepareForDraw(camera)) {
        camera.draw(it.second, renderCameraFlags);
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
  if ((flags_ & Flag::BuildMipmap) && (flags_ & Flag::ColorTexture)) {
    texture(TextureType::Color).generateMipmap();
  }
}

}  // namespace gfx
}  // namespace esp
