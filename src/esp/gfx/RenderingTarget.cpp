// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
//

#include <Magnum/GL/Buffer.h>
#include <Magnum/GL/BufferImage.h>
#include <Magnum/GL/DefaultFramebuffer.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/PixelFormat.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/GL/Texture.h>
#include <Magnum/GL/TextureFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#include "RenderingTarget.h"
#include "magnum.h"

using namespace Magnum;

namespace esp {
namespace gfx {
struct RenderingTarget::Impl {
  Impl(WindowlessContext::ptr context, const Magnum::Vector2i& size)
      : context_{context},
        framebufferSize_(size),
        colorBuffer_(),
        depthBuffer_(),
        objectIdBuffer_(),
        depthRenderbuffer_(),
        framebuffer_(Magnum::NoCreate) {
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

    GL::Renderer::setClearColor(Magnum::Color4{0, 0, 0, 1});
    GL::Renderer::setClearDepth(1.0);
  }

  void renderEnter() {
    framebuffer_.bind();

    framebuffer_.clear(GL::FramebufferClear::Color |
                       GL::FramebufferClear::Depth);
  }

  void renderExit() {}

  gltensor::GLTensorParam::ptr glTensorParam() const {
    auto param = std::make_shared<gltensor::GLTensorParam>();

    param->height_ = framebufferSize_.y();
    param->width_ = framebufferSize_.x();

    param->target_ = GL_RENDERBUFFER;
    param->device_id_ = context_->gpuDevice();

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamRgba() const {
    auto param = this->glTensorParam();

    param->format_ = GL_RGBA;
    param->image_ = colorBuffer_.id();
    param->channels_ = 4;

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamDepth() const {
    auto param = this->glTensorParam();

    param->format_ = GL_R32F;
    param->image_ = depthBuffer_.id();
    param->channels_ = 1;

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamId() const {
    auto param = this->glTensorParam();

    param->format_ = GL_R32UI;
    param->image_ = objectIdBuffer_.id();
    param->channels_ = 1;

    return param;
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

  Magnum::Vector2i framebufferSize() const { return framebufferSize_; }

 private:
  WindowlessContext::ptr context_ = nullptr;

  Magnum::Vector2i framebufferSize_;
  Magnum::GL::Renderbuffer colorBuffer_;
  Magnum::GL::Renderbuffer depthBuffer_;
  Magnum::GL::Renderbuffer objectIdBuffer_;
  Magnum::GL::Renderbuffer depthRenderbuffer_;
  Magnum::GL::Framebuffer framebuffer_;
};  // namespace gfx

RenderingTarget::RenderingTarget(WindowlessContext::ptr context,
                                 const Magnum::Vector2i& size)
    : pimpl_(spimpl::make_unique_impl<Impl>(context, size)) {}

void RenderingTarget::renderEnter() {
  pimpl_->renderEnter();
}

void RenderingTarget::renderExit() {
  pimpl_->renderExit();
}

gltensor::GLTensorParam::ptr RenderingTarget::glTensorParamRgba() const {
  return pimpl_->glTensorParamRgba();
}
gltensor::GLTensorParam::ptr RenderingTarget::glTensorParamDepth() const {
  return pimpl_->glTensorParamDepth();
}
gltensor::GLTensorParam::ptr RenderingTarget::glTensorParamId() const {
  return pimpl_->glTensorParamId();
}

void RenderingTarget::readFrameRgba(uint8_t* ptr) {
  pimpl_->readFrameRgba(ptr);
}

void RenderingTarget::readFrameDepth(float* ptr) {
  pimpl_->readFrameDepth(ptr);
}

void RenderingTarget::readFrameObjectId(uint32_t* ptr) {
  pimpl_->readFrameObjectId(ptr);
}

Magnum::Vector2i RenderingTarget::framebufferSize() const {
  return pimpl_->framebufferSize();
}

}  // namespace gfx
}  // namespace esp
