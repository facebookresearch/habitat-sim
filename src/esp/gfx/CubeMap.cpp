// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree
#include "CubeMap.h"
#include <Corrade/Utility/Assert.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Shaders/Generic.h>

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
  CORRADE_ASSERT(imageSize_ > 0,
                 "CubeMap::reset: image size" << imageSize << "is illegal.", );
  // create an empty cubemap texture
  recreateTexture();

  // prepare frame buffer and render buffer
  recreateFramebuffer();
}

void CubeMap::recreateTexture() {
  Mn::Vector2i size{imageSize_, imageSize_};

  // color texture
  if (flags_ | Flag::ColorTexture) {
    auto& colorTexture = textures_[TextureType::Color];
    colorTexture = std::make_unique<Mn::GL::CubeMapTexture>();
    (*colorTexture)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setMinificationFilter(Mn::GL::SamplerFilter::Linear,
                               Mn::GL::SamplerMipmap::Linear)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Linear)
        .setStorage(Mn::Math::log2(imageSize_) + 1, Mn::GL::TextureFormat::RGB8,
                    size);
  }

  // depth texture
  if (flags_ | Flag::DepthTexture) {
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

void CubeMap::prepareToDraw(int cubeSideIndex) {
  CORRADE_ASSERT(cubeSideIndex >= 0 && cubeSideIndex < 6,
                 "CubeMap::prepareToDraw: the index of the cube side"
                     << cubeSideIndex << "is illegal.", );
  Mn::GL::CubeMapCoordinate cubeMapCoord = Mn::GL::CubeMapCoordinate::PositiveX;
  switch (cubeSideIndex) {
    case 0:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::PositiveX;
      break;
    case 1:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::NegativeX;
      break;
    case 2:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::PositiveY;
      break;
    case 3:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::NegativeY;
      break;
    case 4:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::PositiveZ;
      break;
    case 5:
      cubeMapCoord = Mn::GL::CubeMapCoordinate::NegativeZ;
      break;
    default:  // never reach, just avoid compiler warning
      break;
  }
  if (flags_ | Flag::ColorTexture) {
    frameBuffer_.attachCubeMapTexture(
        colorAttachment, *textures_[TextureType::Color], cubeMapCoord, 0);
  }

  if (flags_ | Flag::DepthTexture) {
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

void CubeMap::renderToTexture(CubeMapCamera& camera,
                              scene::SceneGraph& sceneGraph,
                              RenderCamera::Flags flags) {
  CORRADE_ASSERT(camera.isInSceneGraph(sceneGraph),
                 "CubeMap::renderToTexture: camera is NOT attached to the "
                 "current scene graph.", );
  camera.setProjectionMatrix(imageSize_,
                             0.001f,    // z-near
                             1000.0f);  // z-far
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

void CubeMap::loadColorTexture(Mn::Trade::AbstractImporter& importer,
                               const std::string& imageFilePrefix,
                               const std::string& imageFileExtension) {
  // TODO
}

}  // namespace gfx
}  // namespace esp
