// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
#include <Magnum/GL/Texture.h>
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

namespace {
const Mn::GL::Framebuffer::ColorAttachment rgbaAttachment =
    Mn::GL::Framebuffer::ColorAttachment{0};
const Mn::GL::Framebuffer::ColorAttachment objectIdAttachment =
    Mn::GL::Framebuffer::ColorAttachment{1};

/**
 * @brief check if the class instance is created with corresponding texture
 * enabled
 */
void textureTypeSanityCheck(const char* functionNameStr,
                            CubeMap::Flags& flag,
                            CubeMap::TextureType type) {
  switch (type) {
    case CubeMap::TextureType::Color:
      CORRADE_ASSERT(flag & CubeMap::Flag::ColorTexture,
                     functionNameStr << "instance was not created with color "
                                        "texture output enabled.", );
      return;
      break;
    case CubeMap::TextureType::Depth:
      CORRADE_ASSERT(flag & CubeMap::Flag::DepthTexture,
                     functionNameStr << "instance was not created with depth "
                                        "texture output enabled.", );
      return;
    case CubeMap::TextureType::ObjectId:
      CORRADE_ASSERT(flag & CubeMap::Flag::ObjectIdTexture,
                     functionNameStr
                         << "instance was not created with object id"
                            "texture output enabled.", );
      return;
      break;

    case CubeMap::TextureType::Count:
      break;
  }
  CORRADE_INTERNAL_ASSERT_UNREACHABLE();
}

/** @brief do a couple of sanity checks based on mipLevel value */
void mipLevelSanityCheck(const char* msgPrefix,
                         CubeMap::Flags& flags,
                         unsigned int mipLevel,
                         unsigned int mipmapLevels) {
  if (mipLevel > 0) {
    // sanity check
    CORRADE_ASSERT(flags & CubeMap::Flag::ManuallyBuildMipmap,
                   msgPrefix << "CubeMap is not created with "
                                "Flag::ManuallyBuildMipmap specified. ", );
    // TODO: HDR!!
    CORRADE_ASSERT(
        (flags & CubeMap::Flag::ColorTexture),
        msgPrefix
            << "CubeMap is not created with Flag::ColorTexture specified.", );

    CORRADE_ASSERT(mipLevel < mipmapLevels,
                   msgPrefix << "mip level" << mipLevel
                             << "is illegal. It must smaller than"
                             << mipmapLevels, );
  }
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
}  // namespace

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
  for (unsigned int iSide = 0; iSide < 6; ++iSide) {
    recreateFramebuffer(iSide, imageSize_);
    // attach render buffer to frame buffer
    attachFramebufferRenderbuffer(iSide, 0);
  }

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

    if ((flags_ & Flag::AutoBuildMipmap) ||
        (flags_ & Flag::ManuallyBuildMipmap)) {
      // RGBA8 is for the LDR. Use RGBA16F for the HDR (TODO)
      mipmapLevels_ = Mn::Math::log2(imageSize_) + 1;
      colorTexture.setStorage(mipmapLevels_, Mn::GL::TextureFormat::RGBA8,
                              size);  // TODO: HDR!!
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

void CubeMap::recreateFramebuffer(unsigned int cubeSideIndex,
                                  int framebufferSize) {
  Mn::Vector2i viewportSize{framebufferSize};  // at mip level 0
  frameBuffer_[cubeSideIndex] = Mn::GL::Framebuffer{{{}, viewportSize}};

  // optional depth buffer is 24-bit integer pixel, which is different from the
  // depth texture (32-bit float)
  if (!(flags_ & CubeMap::Flag::DepthTexture)) {
    optionalDepthBuffer_[cubeSideIndex] = Mn::GL::Renderbuffer{};
    optionalDepthBuffer_[cubeSideIndex].setStorage(
        Mn::GL::RenderbufferFormat::DepthComponent24, viewportSize);
  }
}

void CubeMap::attachFramebufferRenderbuffer(unsigned int cubeSideIndex,
                                            unsigned int mipLevel) {
  Magnum::GL::CubeMapCoordinate cubeMapCoord =
      convertFaceIndexToCubeMapCoordinate(cubeSideIndex);

  if (flags_ & Flag::ColorTexture) {
    frameBuffer_[cubeSideIndex].attachCubeMapTexture(
        rgbaAttachment, texture(TextureType::Color), cubeMapCoord,
        int(mipLevel));
  }

  // does NOT make any sense to talk about mip level for depth or object id
  // texture. so the mipLevel is always 0 for both.
  if (flags_ & Flag::DepthTexture) {
    frameBuffer_[cubeSideIndex].attachCubeMapTexture(
        Mn::GL::Framebuffer::BufferAttachment::Depth,
        texture(TextureType::Depth), cubeMapCoord, 0);
  } else {
    frameBuffer_[cubeSideIndex].attachRenderbuffer(
        Mn::GL::Framebuffer::BufferAttachment::Depth,
        optionalDepthBuffer_[cubeSideIndex]);
  }

  if (flags_ & Flag::ObjectIdTexture) {
    frameBuffer_[cubeSideIndex].attachCubeMapTexture(
        objectIdAttachment, texture(TextureType::ObjectId), cubeMapCoord, 0);
  }
}

void CubeMap::prepareToDraw(unsigned int cubeSideIndex,
                            RenderCamera::Flags renderCameraFlags,
                            unsigned int mipLevel) {
  mipLevelSanityCheck("CubeMap::prepareToDraw():", flags_, mipLevel,
                      mipmapLevels_);

  if ((flags_ & Flag::ManuallyBuildMipmap) && (flags_ & Flag::ColorTexture)) {
    int size = imageSize_ / (1 << mipLevel);
    frameBuffer_[cubeSideIndex].setViewport({{}, {size, size}});
    recreateFramebuffer(cubeSideIndex, size);
    attachFramebufferRenderbuffer(cubeSideIndex, mipLevel);
  }

  bindFramebuffer(cubeSideIndex);

  // Note: we ONLY need to map shader output to color attachment when necessary,
  // which means in depth texture mode, we do NOT need to do this
  if (flags_ & CubeMap::Flag::ColorTexture ||
      flags_ & CubeMap::Flag::ObjectIdTexture) {
    mapForDraw(cubeSideIndex);
  }

  if (flags_ & CubeMap::Flag::ColorTexture &&
      renderCameraFlags & RenderCamera::Flag::ClearColor) {
    frameBuffer_[cubeSideIndex].clearColor(
        0,                                  // color attachment
        Mn::Color4{0.0f, 0.0f, 0.0f, 1.0f}  // clear color
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

unsigned int CubeMap::getMipmapLevels() const {
  return mipmapLevels_;
}

void CubeMap::bindFramebuffer(unsigned int cubeSideIndex) {
  CORRADE_INTERNAL_ASSERT(cubeSideIndex < 6);
  frameBuffer_[cubeSideIndex].bind();
}

void CubeMap::mapForDraw(unsigned int index) {
  frameBuffer_[index].mapForDraw(
      {{ColorOutput, (flags_ & CubeMap::Flag::ColorTexture
                          ? rgbaAttachment
                          : Mn::GL::Framebuffer::DrawAttachment::None)},
       {ObjectIdOutput, (flags_ & CubeMap::Flag::ObjectIdTexture
                             ? objectIdAttachment
                             : Mn::GL::Framebuffer::DrawAttachment::None)}});
}

Mn::GL::CubeMapTexture& CubeMap::getTexture(TextureType type) {
  textureTypeSanityCheck("CubeMap::getTexture():", flags_, type);
  return textures_[static_cast<uint8_t>(type)];
}

#ifndef MAGNUM_TARGET_WEBGL
// because Mn::Image2D image = textures_[type]->image(...)
// requires desktop OpenGL
bool CubeMap::saveTexture(TextureType type,
                          const std::string& imageFilePrefix,
                          unsigned int mipLevel) {
  textureTypeSanityCheck("CubeMap::saveTexture():", flags_, type);
  mipLevelSanityCheck("CubeMap::saveTexture():", flags_, mipLevel,
                      mipmapLevels_);

  Cr::PluginManager::Manager<Mn::Trade::AbstractImageConverter> manager;
  Cr::Containers::Pointer<Mn::Trade::AbstractImageConverter> converter;
  if (!(converter = manager.loadAndInstantiate("AnyImageConverter"))) {
    return false;
  }
  // set image converter flags if gfx logging is quieted
  if (!isLevelEnabled(logging::Subsystem::gfx,
                      logging::LoggingLevel::Warning)) {
    converter->addFlags(Mn::Trade::ImageConverterFlag::Quiet);
  } else if (isLevelEnabled(logging::Subsystem::gfx,
                            logging::LoggingLevel::VeryVerbose)) {
    // set verbose flags if necessary
    converter->addFlags(Mn::Trade::ImageConverterFlag::Verbose);
  }

  const char* coordStrings[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};
  for (int iFace = 0; iFace < 6; ++iFace) {
    std::string filename = "";
    auto& tex = texture(type);
    switch (type) {
      case TextureType::Color: {
        Mn::Image2D image =
            tex.image(convertFaceIndexToCubeMapCoordinate(iFace), mipLevel,
                      {getPixelFormat(type)});
        filename = Cr::Utility::formatString(
            "{}.{}.mip_{}.{}.png", imageFilePrefix,
            getTextureTypeFilenameString(type), mipLevel, coordStrings[iFace]);
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

    ESP_DEBUG() << "Saved image" << iFace << "to" << filename;
  }  // for

  return true;
}
#endif

void CubeMap::loadTexture(TextureType type,
                          const std::string& imageFilePrefix,
                          const std::string& imageFileExtension) {
  textureTypeSanityCheck("CubeMap::loadTexture():", flags_, type);

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
    ESP_DEBUG() << "Loaded image" << iFace << "from" << filename;
  }
  // sanity check is within the function
  generateMipmap(type);
}

void CubeMap::generateMipmap(TextureType type) {
  CORRADE_INTERNAL_ASSERT(type == TextureType::Color);
  CORRADE_INTERNAL_ASSERT(flags_ & Flag::ColorTexture);
  if ((flags_ & Flag::AutoBuildMipmap) ||
      (flags_ & Flag::ManuallyBuildMipmap)) {
    Mn::GL::CubeMapTexture& tex = texture(type);
    tex.generateMipmap();
  }
}

void CubeMap::copySubImage(unsigned int cubeSideIndex,
                           TextureType type,
                           Magnum::GL::Texture2D& texture,
                           unsigned int mipLevel) {
  CORRADE_INTERNAL_ASSERT(cubeSideIndex < 6);

  if (mipLevel > 0) {
    mipLevelSanityCheck("CubeMap::CopyToTexture2D():", flags_, mipLevel,
                        mipmapLevels_);
  }

  int size = imageSize_ / (1 << mipLevel);

#ifndef MAGNUM_TARGET_WEBGL
  CORRADE_ASSERT(texture.imageSize(0) == Mn::Vector2i(size),
                 "CubeMap::CopyToTexture2D(): the texture size does not match "
                 "the cubemap size.", );
#endif
  // map for read
  switch (type) {
    case TextureType::Color:
      frameBuffer_[cubeSideIndex].mapForRead(rgbaAttachment);
      break;

    case TextureType::ObjectId:
      frameBuffer_[cubeSideIndex].mapForRead(objectIdAttachment);
      break;

    default:
      CORRADE_INTERNAL_ASSERT_UNREACHABLE();
      break;
  }

  frameBuffer_[cubeSideIndex].copySubImage(
      Mn::Range2Di{{0, 0}, {size, size}},  // rectangle area
      texture,                             // destination
      mipLevel,                            // mip level
      {0, 0});                             // offset
}

void CubeMap::renderToTexture(CubeMapCamera& camera,
                              scene::SceneGraph& sceneGraph,
                              const char* drawableGroupName,
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
    camera.switchToFace(iFace);
    prepareToDraw(iFace, renderCameraFlags);

    // TODO:
    // should have different drawable groups that can do "low quality"
    // rendering, e.g., no normal maps, no specular lighting, low-poly meshes,
    // low-quality textures.
    DrawableGroup& group = sceneGraph.getDrawables(drawableGroupName);
    group.prepareForDraw(camera);
    camera.draw(group, renderCameraFlags);
  }  // iFace

  // CAREFUL!!!
  // switchToFace() will change the local transformation of this camera node!
  // If you do not do anything, in next rendering cycle, since
  // camera.updateOriginalViewingMatrix() is called, the original viewing matrix
  // will be updated by mistake!!! To prevent such mistakes, local
  // transformation of this camera node must be reset.
  camera.restoreTransformation();

  // NOT for depth
  if (flags_ & Flag::AutoBuildMipmap) {
    if (flags_ & Flag::ColorTexture) {
      texture(TextureType::Color).generateMipmap();
    }
  }
}

}  // namespace gfx
}  // namespace esp
