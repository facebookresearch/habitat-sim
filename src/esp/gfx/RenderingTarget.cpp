// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
//
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

#include "RenderingTarget.h"

using namespace Magnum;

namespace esp {
namespace gfx {
struct RenderingTarget::Impl {
  Impl(WindowlessContext::ptr context, int height, int width)
      : context_{context},
        framebufferSize_(width, height),
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
  }

  void renderEnter() { framebuffer_.bind(); }

  void renderExit() { glBindFramebuffer(GL_FRAMEBUFFER, 0); }

  gltensor::GLTensorParam::ptr glTensorParam() const {
    auto param = std::make_shared<gltensor::GLTensorParam>();

    param->height_ = framebufferSize_[0];
    param->width_ = framebufferSize_[1];

    param->target_ = GL_TEXTURE_2D;

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamRGBA() const {
    auto param = this->glTensorParam();

    param->format_ = GL_RGBA;
    param->image_ = colorBuffer_.id();

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamDepth() const {
    auto param = this->glTensorParam();

    param->format_ = GL_R32F;
    param->image_ = depthBuffer_.id();

    return param;
  }

  gltensor::GLTensorParam::ptr glTensorParamID() const {
    auto param = this->glTensorParam();

    param->format_ = GL_R32UI;
    param->image_ = objectIdBuffer_.id();

    return param;
  }

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
                                 int height,
                                 int width)
    : pimpl_(spimpl::make_unique_impl<Impl>(context, height, width)) {}

void RenderingTarget::renderEnter() {
  pimpl_->renderEnter();
}
void renderExit();
gltensor::GLTensorParam::ptr glTensorParam() const;

void readFrameRgba(uint8_t* ptr);

void readFrameDepth(float* ptr);

void readFrameObjectId(uint32_t* ptr);

const Magnum::Vector2i framebufferSize() const;

}  // namespace gfx
}  // namespace esp
