// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include <Corrade/Containers/StridedArrayView.h>
#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>

#include "esp/gfx/DepthUnprojection.h"
#include "esp/gfx/magnum.h"

using namespace Magnum;

namespace esp {
namespace gfx {

struct Renderer::Impl {
  Impl(int width, int height)
      : framebufferSize_(width, height),
        colorBuffer_(),
        objectIdBuffer_(),
        depthRenderbuffer_(),
        framebuffer_({{}, framebufferSize_}) {
    GL::Renderer::enable(GL::Renderer::Feature::DepthTest);
    GL::Renderer::enable(GL::Renderer::Feature::FaceCulling);
    setSize(width, height);
  }
  ~Impl() { LOG(INFO) << "Deconstructing Renderer"; }

  void setSize(int width, int height) {
    framebufferSize_[0] = width;
    framebufferSize_[1] = height;
    colorBuffer_.setStorage(GL::RenderbufferFormat::SRGB8Alpha8,
                            framebufferSize_);
    objectIdBuffer_.setStorage(GL::RenderbufferFormat::R32UI, framebufferSize_);
    depthRenderbuffer_.setStorage(GL::RenderbufferFormat::DepthComponent32F,
                                  framebufferSize_);
    framebuffer_ = GL::Framebuffer{{{}, framebufferSize_}};
    framebuffer_
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer_)
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1},
                            objectIdBuffer_)
        .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth,
                            depthRenderbuffer_)
        .mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}},
                     {1, GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(
        framebuffer_.checkStatus(GL::FramebufferTarget::Draw) ==
        GL::Framebuffer::Status::Complete);
  }

  inline void renderEnter() {
    framebuffer_.clearDepth(1.0);
    framebuffer_.clearColor(0, Color4{});
    framebuffer_.clearColor(1, Vector4ui{});
    framebuffer_.bind();
  }

  inline void renderExit() {}

  void draw(RenderCamera& camera, MagnumDrawableGroup& drawables) {
    renderEnter();
    camera.getMagnumCamera().setViewport(framebufferSize_);

    depthUnprojection_ =
        calculateDepthUnprojection(camera.getMagnumCamera().projectionMatrix());

    camera.draw(drawables);
    renderExit();
  }

  void draw(sensor::Sensor& visualSensor, scene::SceneGraph& sceneGraph) {
    ASSERT(visualSensor.isVisualSensor());

    // set the modelview matrix, projection matrix of the render camera;
    sceneGraph.setDefaultRenderCamera(visualSensor);

    draw(sceneGraph.getDefaultRenderCamera(), sceneGraph.getDrawables());
  }

  void readFrameRgba(uint8_t* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{0});
    Image2D rgbaImage =
        framebuffer_.read(Range2Di::fromSize({0, 0}, framebufferSize_),
                          {PixelFormat::RGBA8Unorm});
    std::memcpy(ptr, rgbaImage.data(), rgbaImage.data().size());
  }

  void readFrameDepth(float* ptr) {
    Image2D depthImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    /* Unproject the Z */
    unprojectDepth(depthUnprojection_,
                   Containers::arrayCast<Float>(depthImage.data()));
    std::memcpy(ptr, depthImage.data(), depthImage.data().size());
  }

  void readFrameObjectId(uint32_t* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D objectImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32UI});
    std::memcpy(ptr, objectImage.data(), objectImage.data().size());
  }

  Magnum::Vector2i framebufferSize_;
  GL::Renderbuffer colorBuffer_;
  GL::Renderbuffer objectIdBuffer_;
  GL::Renderbuffer depthRenderbuffer_;
  GL::Framebuffer framebuffer_;

  Matrix2x2 depthUnprojection_;
};

Renderer::Renderer(int width, int height)
    : pimpl_(spimpl::make_unique_impl<Impl>(width, height)) {}

void Renderer::draw(RenderCamera& camera, scene::SceneGraph& sceneGraph) {
  pimpl_->draw(camera, sceneGraph.getDrawables());
}

void Renderer::draw(sensor::Sensor& visualSensor,
                    scene::SceneGraph& sceneGraph) {
  pimpl_->draw(visualSensor, sceneGraph);
}

void Renderer::setSize(int width, int height) {
  pimpl_->setSize(width, height);
}

void Renderer::readFrameRgba(uint8_t* ptr) {
  pimpl_->readFrameRgba(ptr);
}

void Renderer::readFrameDepth(float* ptr) {
  pimpl_->readFrameDepth(ptr);
}

void Renderer::readFrameObjectId(uint32_t* ptr) {
  pimpl_->readFrameObjectId(ptr);
}

vec3i Renderer::getSize() {
  return vec3i(pimpl_->framebufferSize_[0], pimpl_->framebufferSize_[1], 4);
}

}  // namespace gfx
}  // namespace esp
