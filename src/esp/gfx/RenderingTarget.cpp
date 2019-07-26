// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

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

#ifdef ESP_WITH_GPU_GPU
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include "helper_cuda.h"
#endif

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

  void readFrameRgba(const MutableImageView2D& view) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{0})
        .read({{}, framebufferSize_}, view);
  }

  void readFrameDepth(const MutableImageView2D& view) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{1})
        .read({{}, framebufferSize_}, view);
  }

  void readFrameObjectId(const MutableImageView2D& view) {
    framebuffer_.mapForRead(GL::Framebuffer::ColorAttachment{2})
        .read({{}, framebufferSize_}, view);
  }

  Magnum::Vector2i framebufferSize() const { return framebufferSize_; }

  int gpuDeviceId() const { return this->context_->gpuDevice(); }

#ifdef ESP_WITH_GPU_GPU
  void readFrameRgbaGPU(uint8_t* devPtr) {
    if (colorBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &colorBufferCugl_, colorBuffer_.id(), GL_RENDERBUFFER,
          cudaGraphicsRegisterFlagsReadOnly));

    checkCudaErrors(cudaGraphicsMapResources(1, &colorBufferCugl_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(
        cudaGraphicsSubResourceGetMappedArray(&array, colorBufferCugl_, 0, 0));
    checkCudaErrors(cudaMemcpyFromArray(
        devPtr, array, 0, 0,
        framebufferSize_.x() * framebufferSize_.y() * 4 * sizeof(uint8_t),
        cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &colorBufferCugl_, 0));
  }

  void readFrameDepthGPU(float* devPtr) {
    if (depthBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &depthBufferCugl_, depthBuffer_.id(), GL_RENDERBUFFER,
          cudaGraphicsRegisterFlagsReadOnly));

    checkCudaErrors(cudaGraphicsMapResources(1, &depthBufferCugl_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(
        cudaGraphicsSubResourceGetMappedArray(&array, depthBufferCugl_, 0, 0));
    checkCudaErrors(cudaMemcpyFromArray(
        devPtr, array, 0, 0,
        framebufferSize_.x() * framebufferSize_.y() * 1 * sizeof(float),
        cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &depthBufferCugl_, 0));
  }

  void readFrameObjectIdGPU(int32_t* devPtr) {
    if (objecIdBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &objecIdBufferCugl_, objectIdBuffer_.id(), GL_RENDERBUFFER,
          cudaGraphicsRegisterFlagsReadOnly));

    checkCudaErrors(cudaGraphicsMapResources(1, &objecIdBufferCugl_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(
        &array, objecIdBufferCugl_, 0, 0));
    checkCudaErrors(cudaMemcpyFromArray(
        devPtr, array, 0, 0,
        framebufferSize_.x() * framebufferSize_.y() * 1 * sizeof(int32_t),
        cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &objecIdBufferCugl_, 0));
  }
#endif

  ~Impl() {
#ifdef ESP_WITH_GPU_GPU
    if (colorBufferCugl_ != nullptr)
      checkCudaErrors(cudaGraphicsUnregisterResource(colorBufferCugl_));
    if (depthBufferCugl_ != nullptr)
      checkCudaErrors(cudaGraphicsUnregisterResource(depthBufferCugl_));
    if (objecIdBufferCugl_ != nullptr)
      checkCudaErrors(cudaGraphicsUnregisterResource(objecIdBufferCugl_));
#endif
  }

 private:
  WindowlessContext::ptr context_ = nullptr;

  Magnum::Vector2i framebufferSize_;
  Magnum::GL::Renderbuffer colorBuffer_;
  Magnum::GL::Renderbuffer depthBuffer_;
  Magnum::GL::Renderbuffer objectIdBuffer_;
  Magnum::GL::Renderbuffer depthRenderbuffer_;
  Magnum::GL::Framebuffer framebuffer_;

#ifdef ESP_WITH_GPU_GPU
  cudaGraphicsResource_t colorBufferCugl_ = nullptr;
  cudaGraphicsResource_t depthBufferCugl_ = nullptr;
  cudaGraphicsResource_t objecIdBufferCugl_ = nullptr;
#endif
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

#ifdef ESP_WITH_GPU_GPU
void RenderingTarget::readFrameRgbaGPU(uint8_t* devPtr) {
  pimpl_->readFrameRgbaGPU(devPtr);
}

void RenderingTarget::readFrameDepthGPU(float* devPtr) {
  pimpl_->readFrameDepthGPU(devPtr);
}

void RenderingTarget::readFrameObjectIdGPU(int32_t* devPtr) {
  pimpl_->readFrameObjectIdGPU(devPtr);
}
#endif

void RenderingTarget::readFrameRgba(const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameRgba(view);
}

void RenderingTarget::readFrameDepth(const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameDepth(view);
}

void RenderingTarget::readFrameObjectId(
    const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameObjectId(view);
}

int RenderingTarget::gpuDeviceId() const {
  return pimpl_->gpuDeviceId();
}

Magnum::Vector2i RenderingTarget::framebufferSize() const {
  return pimpl_->framebufferSize();
}

}  // namespace gfx
}  // namespace esp
