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

#include "RenderTarget.h"
#include "magnum.h"

#include "esp/sensor/Sensor.h"

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include "helper_cuda.h"
#endif

using namespace Magnum;

namespace esp {
namespace gfx {

const GL::Framebuffer::ColorAttachment RgbaBuffer =
    GL::Framebuffer::ColorAttachment{0};
const GL::Framebuffer::ColorAttachment ObjectIdBuffer =
    GL::Framebuffer::ColorAttachment{1};

struct RenderTarget::Impl {
  Impl(WindowlessContext::ptr context,
       const Magnum::Vector2i& size,
       const Magnum::Matrix2x2& depthUnprojection)
      : context_{context},
        colorBuffer_{},
        objectIdBuffer_{},
        depthRenderbuffer_{},
        framebuffer_{Magnum::NoCreate},
        depthUnprojection_{depthUnprojection} {
    colorBuffer_.setStorage(GL::RenderbufferFormat::SRGB8Alpha8, size);
    objectIdBuffer_.setStorage(GL::RenderbufferFormat::R32UI, size);
    depthRenderbuffer_.setStorage(GL::RenderbufferFormat::DepthComponent32F,
                                  size);
    framebuffer_ = GL::Framebuffer{{{}, size}};
    framebuffer_.attachRenderbuffer(RgbaBuffer, colorBuffer_)
        .attachRenderbuffer(ObjectIdBuffer, objectIdBuffer_)
        .attachRenderbuffer(GL::Framebuffer::BufferAttachment::Depth,
                            depthRenderbuffer_)
        .mapForDraw({{0, GL::Framebuffer::ColorAttachment{0}},
                     {1, GL::Framebuffer::ColorAttachment{1}}});
    CORRADE_INTERNAL_ASSERT(
        framebuffer_.checkStatus(GL::FramebufferTarget::Draw) ==
        GL::Framebuffer::Status::Complete);

#ifndef CORRADE_TARGET_EMSCRIPTEN
    GL::Renderer::setClearColor(Magnum::Color4{0, 0, 0, 1});
    GL::Renderer::setClearDepth(1.0);
#endif
  }

  void renderEnter() {
#ifdef CORRADE_TARGET_EMSCRIPTEN
    framebuffer_.clearDepth(1.0);
    framebuffer_.clearColor(0, Magnum::Color4{});
    framebuffer_.clearColor(1, Magnum::Vector4ui{});
#else
    framebuffer_.clear(GL::FramebufferClear::Color |
                       GL::FramebufferClear::Depth);
    framebuffer_.clearColor(1, Magnum::Vector4ui{});
#endif
    framebuffer_.bind();
  }

  void renderExit() {}

  void readFrameRgba(const MutableImageView2D& view) {
    framebuffer_.mapForRead(RgbaBuffer).read(framebuffer_.viewport(), view);
  }

  void readFrameDepth(const MutableImageView2D& view) {
    Image2D depthImage = framebuffer_.read(
        framebuffer_.viewport(),
        {GL::PixelFormat::DepthComponent, GL::PixelType::Float});

    /* Unproject the Z */
    Containers::ArrayView<const Float> data =
        Containers::arrayCast<const Float>(depthImage.data());
    Containers::ArrayView<Float> ptr =
        Containers::arrayCast<Float>(view.data());
    for (std::size_t i = 0; i != data.size(); ++i) {
      const Float z = data[i];

      /* If a fragment has a depth of 1, it's due to a hole in the mesh. The
         consumers expect 0 for things that are too far, so be nice to them.
         We can afford using == for comparison as 1.0f has an exact
         representation and the depth is cleared to exactly this value. */
      if (z == 1.0f) {
        ptr[i] = 0.0f;
        continue;
      }

      /* The following is

          (az + b) / (cz + d)

         See the comment in PinholeCamera::depthUnprojection() for details. */
      ptr[i] =
          Math::fma(depthUnprojection_[0][0], z, depthUnprojection_[1][0]) /
          Math::fma(depthUnprojection_[0][1], z, depthUnprojection_[1][1]);
    }
  }

  void readFrameObjectId(const MutableImageView2D& view) {
    framebuffer_.mapForRead(ObjectIdBuffer).read(framebuffer_.viewport(), view);
  }

  Magnum::Vector2i framebufferSize() const {
    return framebuffer_.viewport().size();
  }

  int gpuDeviceId() const { return this->context_->gpuDevice(); }

#ifdef ESP_BUILD_WITH_CUDA
  void readFrameRgbaGPU(uint8_t* devPtr) {
    if (colorBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &colorBufferCugl_, colorBuffer_.id(), GL_RENDERBUFFER,
          cudaGraphicsRegisterFlagsReadOnly));

    checkCudaErrors(cudaGraphicsMapResources(1, &colorBufferCugl_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(
        cudaGraphicsSubResourceGetMappedArray(&array, colorBufferCugl_, 0, 0));
    const int widthInBytes = framebufferSize().x() * 4 * sizeof(uint8_t);
    checkCudaErrors(cudaMemcpy2DFromArray(devPtr, widthInBytes, array, 0, 0,
                                          widthInBytes, framebufferSize().y(),
                                          cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &colorBufferCugl_, 0));
  }

  void readFrameDepthGPU(float* devPtr) {
    if (depthBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &depthBufferCugl_, depthRenderbuffer_.id(), GL_RENDERBUFFER,
          cudaGraphicsRegisterFlagsReadOnly));

    checkCudaErrors(cudaGraphicsMapResources(1, &depthBufferCugl_, 0));

    cudaArray* array = nullptr;
    checkCudaErrors(
        cudaGraphicsSubResourceGetMappedArray(&array, depthBufferCugl_, 0, 0));
    const int widthInBytes = framebufferSize().x() * 1 * sizeof(float);
    checkCudaErrors(cudaMemcpy2DFromArray(devPtr, widthInBytes, array, 0, 0,
                                          widthInBytes, framebufferSize().y(),
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
    const int widthInBytes = framebufferSize().x() * 1 * sizeof(int32_t);
    checkCudaErrors(cudaMemcpy2DFromArray(devPtr, widthInBytes, array, 0, 0,
                                          widthInBytes, framebufferSize().y(),
                                          cudaMemcpyDeviceToDevice));

    checkCudaErrors(cudaGraphicsUnmapResources(1, &objecIdBufferCugl_, 0));
  }
#endif

  ~Impl() {
#ifdef ESP_BUILD_WITH_CUDA
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

  Magnum::GL::Renderbuffer colorBuffer_;
  Magnum::GL::Renderbuffer objectIdBuffer_;
  Magnum::GL::Renderbuffer depthRenderbuffer_;
  Magnum::GL::Framebuffer framebuffer_;

  Matrix2x2 depthUnprojection_;

#ifdef ESP_BUILD_WITH_CUDA
  cudaGraphicsResource_t colorBufferCugl_ = nullptr;
  cudaGraphicsResource_t objecIdBufferCugl_ = nullptr;
  cudaGraphicsResource_t depthBufferCugl_ = nullptr;
#endif
};  // namespace gfx

RenderTarget::RenderTarget(WindowlessContext::ptr context,
                           const Magnum::Vector2i& size,
                           const Magnum::Matrix2x2& depthUnprojection)
    : pimpl_(spimpl::make_unique_impl<Impl>(context, size, depthUnprojection)) {
}

void RenderTarget::renderEnter() {
  pimpl_->renderEnter();
}

void RenderTarget::renderExit() {
  pimpl_->renderExit();
}

void RenderTarget::readFrameRgba(const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameRgba(view);
}

void RenderTarget::readFrameDepth(const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameDepth(view);
}

void RenderTarget::readFrameObjectId(const Magnum::MutableImageView2D& view) {
  pimpl_->readFrameObjectId(view);
}

int RenderTarget::gpuDeviceId() const {
  return pimpl_->gpuDeviceId();
}

Magnum::Vector2i RenderTarget::framebufferSize() const {
  return pimpl_->framebufferSize();
}

#ifdef ESP_BUILD_WITH_CUDA
void RenderTarget::readFrameRgbaGPU(uint8_t* devPtr) {
  pimpl_->readFrameRgbaGPU(devPtr);
}

void RenderTarget::readFrameDepthGPU(float* devPtr) {
  pimpl_->readFrameDepthGPU(devPtr);
}

void RenderTarget::readFrameObjectIdGPU(int32_t* devPtr) {
  pimpl_->readFrameObjectIdGPU(devPtr);
}
#endif

}  // namespace gfx
}  // namespace esp
