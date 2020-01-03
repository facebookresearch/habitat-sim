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

#include "esp/gfx/DepthUnprojection.h"

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>
#include <cuda_runtime.h>
#include "helper_cuda.h"
#endif

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

const Mn::GL::Framebuffer::ColorAttachment RgbaBuffer =
    Mn::GL::Framebuffer::ColorAttachment{0};
const Mn::GL::Framebuffer::ColorAttachment ObjectIdBuffer =
    Mn::GL::Framebuffer::ColorAttachment{1};
const Mn::GL::Framebuffer::ColorAttachment UnprojectedDepthBuffer =
    Mn::GL::Framebuffer::ColorAttachment{0};

struct RenderTarget::Impl {
  Impl(const Mn::Vector2i& size,
       const Mn::Vector2& depthUnprojection,
       DepthShader* depthShader)
      : colorBuffer_{},
        objectIdBuffer_{},
        depthRenderTexture_{},
        framebuffer_{Mn::NoCreate},
        depthUnprojection_{depthUnprojection},
        depthShader_{depthShader},
        unprojectedDepth_{Mn::NoCreate},
        depthUnprojectionMesh_{Mn::NoCreate},
        depthUnprojectionFrameBuffer_{Mn::NoCreate} {
    if (depthShader_) {
      CORRADE_INTERNAL_ASSERT(depthShader_->flags() &
                              DepthShader::Flag::UnprojectExistingDepth);
    }

    colorBuffer_.setStorage(Mn::GL::RenderbufferFormat::SRGB8Alpha8, size);
    objectIdBuffer_.setStorage(Mn::GL::RenderbufferFormat::R32UI, size);
    depthRenderTexture_.setMinificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setMagnificationFilter(Mn::GL::SamplerFilter::Nearest)
        .setWrapping(Mn::GL::SamplerWrapping::ClampToEdge)
        .setStorage(1, Mn::GL::TextureFormat::DepthComponent32F, size);

    framebuffer_ = Mn::GL::Framebuffer{{{}, size}};
    framebuffer_.attachRenderbuffer(RgbaBuffer, colorBuffer_)
        .attachRenderbuffer(ObjectIdBuffer, objectIdBuffer_)
        .attachTexture(Mn::GL::Framebuffer::BufferAttachment::Depth,
                       depthRenderTexture_, 0)
        .mapForDraw({{0, RgbaBuffer}, {1, ObjectIdBuffer}});
    CORRADE_INTERNAL_ASSERT(
        framebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
        Mn::GL::Framebuffer::Status::Complete);
  }

  void initDepthUnprojector() {
    if (depthUnprojectionMesh_.id() == 0) {
      unprojectedDepth_ = Mn::GL::Renderbuffer{};
      unprojectedDepth_.setStorage(Mn::GL::RenderbufferFormat::R32F,
                                   framebufferSize());

      depthUnprojectionFrameBuffer_ =
          Mn::GL::Framebuffer{{{}, framebufferSize()}};
      depthUnprojectionFrameBuffer_
          .attachRenderbuffer(UnprojectedDepthBuffer, unprojectedDepth_)
          .mapForDraw({{0, UnprojectedDepthBuffer}});
      CORRADE_INTERNAL_ASSERT(
          framebuffer_.checkStatus(Mn::GL::FramebufferTarget::Draw) ==
          Mn::GL::Framebuffer::Status::Complete);

      depthUnprojectionMesh_ = Mn::GL::Mesh{};
      depthUnprojectionMesh_.setCount(3);
    }
  }

  void unprojectDepthGPU() {
    CORRADE_INTERNAL_ASSERT(depthShader_ != nullptr);
    initDepthUnprojector();

    depthUnprojectionFrameBuffer_.bind();
    depthShader_->bindDepthTexture(depthRenderTexture_)
        .setDepthUnprojection(depthUnprojection_);

    depthUnprojectionMesh_.draw(*depthShader_);
  }

  void renderEnter() {
    framebuffer_.clearDepth(1.0);
    framebuffer_.clearColor(0, Mn::Color4{0, 0, 0, 1});
    framebuffer_.clearColor(1, Mn::Vector4ui{});
    framebuffer_.bind();
  }

  void renderExit() {}

  void blitRgbaToDefault() {
    framebuffer_.mapForRead(RgbaBuffer);
    ASSERT(framebuffer_.viewport() == Mn::GL::defaultFramebuffer.viewport());

    Mn::GL::AbstractFramebuffer::blit(
        framebuffer_, Mn::GL::defaultFramebuffer, framebuffer_.viewport(),
        Mn::GL::defaultFramebuffer.viewport(), Mn::GL::FramebufferBlit::Color,
        Mn::GL::FramebufferBlitFilter::Nearest);
  }

  void readFrameRgba(const Mn::MutableImageView2D& view) {
    framebuffer_.mapForRead(RgbaBuffer).read(framebuffer_.viewport(), view);
  }

  void readFrameDepth(const Mn::MutableImageView2D& view) {
    if (depthShader_) {
      unprojectDepthGPU();
      depthUnprojectionFrameBuffer_.mapForRead(UnprojectedDepthBuffer)
          .read(framebuffer_.viewport(), view);
    } else {
      Mn::MutableImageView2D depthBufferView{
          Mn::GL::PixelFormat::DepthComponent, Mn::GL::PixelType::Float,
          view.size(), view.data()};
      framebuffer_.read(framebuffer_.viewport(), depthBufferView);
      unprojectDepth(depthUnprojection_,
                     Cr::Containers::arrayCast<Mn::Float>(view.data()));
    }
  }

  void readFrameObjectId(const Mn::MutableImageView2D& view) {
    framebuffer_.mapForRead(ObjectIdBuffer).read(framebuffer_.viewport(), view);
  }

  Mn::Vector2i framebufferSize() const {
    return framebuffer_.viewport().size();
  }

#ifdef ESP_BUILD_WITH_CUDA
  void readFrameRgbaGPU(uint8_t* devPtr) {
    // TODO: Consider implementing the GPU read functions with EGLImage
    // See discussion here:
    // https://github.com/facebookresearch/habitat-sim/pull/114#discussion_r312718502

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
    unprojectDepthGPU();

    if (depthBufferCugl_ == nullptr)
      checkCudaErrors(cudaGraphicsGLRegisterImage(
          &depthBufferCugl_, unprojectedDepth_.id(), GL_RENDERBUFFER,
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
  Mn::GL::Renderbuffer colorBuffer_;
  Mn::GL::Renderbuffer objectIdBuffer_;
  Mn::GL::Texture2D depthRenderTexture_;
  Mn::GL::Framebuffer framebuffer_;

  Mn::Vector2 depthUnprojection_;
  DepthShader* depthShader_;
  Mn::GL::Renderbuffer unprojectedDepth_;
  Mn::GL::Mesh depthUnprojectionMesh_;
  Mn::GL::Framebuffer depthUnprojectionFrameBuffer_;

#ifdef ESP_BUILD_WITH_CUDA
  cudaGraphicsResource_t colorBufferCugl_ = nullptr;
  cudaGraphicsResource_t objecIdBufferCugl_ = nullptr;
  cudaGraphicsResource_t depthBufferCugl_ = nullptr;
#endif
};  // namespace gfx

RenderTarget::RenderTarget(const Mn::Vector2i& size,
                           const Mn::Vector2& depthUnprojection,
                           DepthShader* depthShader)
    : pimpl_(spimpl::make_unique_impl<Impl>(size,
                                            depthUnprojection,
                                            depthShader)) {}

void RenderTarget::renderEnter() {
  pimpl_->renderEnter();
}

void RenderTarget::renderExit() {
  pimpl_->renderExit();
}

void RenderTarget::readFrameRgba(const Mn::MutableImageView2D& view) {
  pimpl_->readFrameRgba(view);
}

void RenderTarget::readFrameDepth(const Mn::MutableImageView2D& view) {
  pimpl_->readFrameDepth(view);
}

void RenderTarget::readFrameObjectId(const Mn::MutableImageView2D& view) {
  pimpl_->readFrameObjectId(view);
}

void RenderTarget::blitRgbaToDefault() {
  pimpl_->blitRgbaToDefault();
}

Mn::Vector2i RenderTarget::framebufferSize() const {
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
