// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Renderer.h"

#include "magnum.h"

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>

using namespace Magnum;

namespace esp {
namespace gfx {

struct Renderer::Impl {
  Impl(int width, int height)
      : framebufferSize_(width, height),
        colorBuffer_(),
        depthBuffer_(),
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
    depthBuffer_.setStorage(GL::RenderbufferFormat::R32F, framebufferSize_);
    objectIdBuffer_.setStorage(GL::RenderbufferFormat::R32UI, framebufferSize_);
    depthRenderbuffer_.setStorage(GL::RenderbufferFormat::Depth24Stencil8,
                                  framebufferSize_);
    framebuffer_ = GL::Framebuffer{{{}, framebufferSize_}};
    framebuffer_
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{0}, colorBuffer_)
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{1}, depthBuffer_)
        .attachRenderbuffer(GL::Framebuffer::ColorAttachment{2},
                            objectIdBuffer_)
        .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth,
                            depthRenderbuffer_)
        .mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}},
                     {1, GL::Framebuffer::ColorAttachment{1}},
                     {2, GL::Framebuffer::ColorAttachment{2}}});
    CORRADE_INTERNAL_ASSERT(
        framebuffer_.checkStatus(GL::FramebufferTarget::Draw) ==
        GL::Framebuffer::Status::Complete);
  }

  inline void renderEnter() {
    framebuffer_.clear(GL::FramebufferClear::Color |
                       GL::FramebufferClear::Depth);
    framebuffer_.clearColor(1, Vector4{});
    framebuffer_.clearColor(2, Vector4ui{});
    framebuffer_.bind();
  }

  inline void renderExit() {}

  void draw(RenderCamera& camera, MagnumDrawableGroup& drawables) {
    renderEnter();
    camera.getMagnumCamera().setViewport(framebufferSize_);
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
    uint8_t* src_ptr = rgbaImage.data<uint8_t>();
    std::memcpy(
        ptr, src_ptr,
        framebufferSize_[0] * framebufferSize_[1] * 4 * sizeof(uint8_t));
  }

  void readFrameDepth(float* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{1});
    Image2D depthImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32F});
    float* src_ptr = depthImage.data<float>();
    std::memcpy(ptr, src_ptr,
                framebufferSize_[0] * framebufferSize_[1] * sizeof(float));
  }

  void readFrameObjectId(uint32_t* ptr) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{2});
    Image2D objectImage = framebuffer_.read(
        Range2Di::fromSize({0, 0}, framebufferSize_), {PixelFormat::R32UI});
    uint32_t* src_ptr = objectImage.data<uint32_t>();
    std::memcpy(ptr, src_ptr,
                framebufferSize_[0] * framebufferSize_[1] * sizeof(uint32_t));
  }

  Magnum::Vector2i framebufferSize_;
  GL::Renderbuffer colorBuffer_;
  GL::Renderbuffer depthBuffer_;
  GL::Renderbuffer objectIdBuffer_;
  GL::Renderbuffer depthRenderbuffer_;
  GL::Framebuffer framebuffer_;
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
