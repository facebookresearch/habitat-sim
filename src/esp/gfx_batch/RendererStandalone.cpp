// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RendererStandalone.h"

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Magnum/GL/BufferImage.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>
#include <Magnum/PixelFormat.h>

#ifdef MAGNUM_TARGET_EGL
#include <Magnum/Platform/WindowlessEglApplication.h>
#elif defined(CORRADE_TARGET_APPLE)
#include <Magnum/Platform/WindowlessCglApplication.h>
#elif defined(CORRADE_TARGET_UNIX)
/* Mainly for builds with external Magnum that might not have TARGET_EGL
   enabled. */
#include <Magnum/Platform/WindowlessGlxApplication.h>
#elif defined(CORRADE_TARGET_WINDOWS)
#include <Magnum/Platform/WindowlessWglApplication.h>
#else
#error unsupported platform
#endif

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>

#include "HelperCuda.h"
#endif

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx_batch {

struct RendererStandaloneConfiguration::State {
  /* Not picking any CUDA device by default */
  Magnum::UnsignedInt cudaDevice = ~Magnum::UnsignedInt{};
  RendererStandaloneFlags flags;
};

RendererStandaloneConfiguration::RendererStandaloneConfiguration()
    : state{Cr::InPlaceInit} {}
RendererStandaloneConfiguration::~RendererStandaloneConfiguration() = default;

RendererStandaloneConfiguration& RendererStandaloneConfiguration::setCudaDevice(
    Magnum::UnsignedInt id) {
  state->cudaDevice = id;
  return *this;
}

RendererStandaloneConfiguration& RendererStandaloneConfiguration::setFlags(
    RendererStandaloneFlags flags) {
  state->flags = flags;
  return *this;
}

struct RendererStandalone::State {
  RendererStandaloneFlags flags;
  Mn::Platform::WindowlessGLContext context;
  Mn::Platform::GLContext magnumContext{Mn::NoCreate};
  Mn::GL::Renderbuffer color{Mn::NoCreate}, depth{Mn::NoCreate};
  Mn::GL::Framebuffer framebuffer{Mn::NoCreate};
  Mn::GL::BufferImage2D colorBuffer{Mn::NoCreate};
  Mn::GL::BufferImage2D depthBuffer{Mn::NoCreate};
#ifdef ESP_BUILD_WITH_CUDA
  cudaGraphicsResource* cudaColorBuffer{};
  cudaGraphicsResource* cudaDepthBuffer{};
#endif

  explicit State(const RendererStandaloneConfiguration& configuration)
      : flags{configuration.state->flags}, context {
    Mn::Platform::WindowlessGLContext::Configuration {}
#if defined(MAGNUM_TARGET_EGL) && !defined(CORRADE_TARGET_EMSCRIPTEN)
    .setCudaDevice(configuration.state->cudaDevice)
#endif
        .addFlags(
            configuration.state->flags & RendererStandaloneFlag::QuietLog
                ? Mn::Platform::WindowlessGLContext::Configuration::Flag::
                      QuietLog
                : Mn::Platform::WindowlessGLContext::Configuration::Flags{})
  }
  {
    context.makeCurrent();
    magnumContext.create(Mn::GL::Context::Configuration{}.addFlags(
        configuration.state->flags & RendererStandaloneFlag::QuietLog
            ? Mn::GL::Context::Configuration::Flag::QuietLog
            : Mn::GL::Context::Configuration::Flags{}));
    color = Mn::GL::Renderbuffer{};
    depth = Mn::GL::Renderbuffer{};
  }

#ifdef ESP_BUILD_WITH_CUDA
  ~State() {
    /* Should be unmapped before the GL object gets destroyed, I guess? */
    if (cudaColorBuffer) {
      checkCudaErrors(cudaGraphicsUnmapResources(1, &cudaColorBuffer, 0));
      checkCudaErrors(cudaGraphicsUnregisterResource(cudaColorBuffer));
    }
    if (cudaDepthBuffer) {
      checkCudaErrors(cudaGraphicsUnmapResources(1, &cudaDepthBuffer, 0));
      checkCudaErrors(cudaGraphicsUnregisterResource(cudaDepthBuffer));
    }
  }
#endif
};

RendererStandalone::RendererStandalone(
    const RendererConfiguration& configuration,
    const RendererStandaloneConfiguration& standaloneConfiguration)
    : Renderer{Mn::NoCreate}, state_{Cr::InPlaceInit, standaloneConfiguration} {
  /* Create the renderer only once the GL context is ready */
  create(configuration);

  const Mn::Vector2i size = tileSize() * tileCount();
  state_->color.setStorage(Mn::GL::RenderbufferFormat::RGBA8, size);
  state_->depth.setStorage(Mn::GL::RenderbufferFormat::DepthComponent32F, size);
  state_->framebuffer = Mn::GL::Framebuffer{Mn::Range2Di{{}, size}};
  state_->framebuffer
      .attachRenderbuffer(Mn::GL::Framebuffer::ColorAttachment{0},
                          state_->color)
      .attachRenderbuffer(Mn::GL::Framebuffer::BufferAttachment::Depth,
                          state_->depth);
  /* Defer the buffer initialization to the point when it's actually read
     into */
  state_->colorBuffer = Mn::GL::BufferImage2D{colorFramebufferFormat()};
  state_->depthBuffer = Mn::GL::BufferImage2D{depthFramebufferFormat()};
}

RendererStandalone::~RendererStandalone() {
  /* As we hold the GL context, GL resources have to be destructed before this
   * destructor. */
  Renderer::destroy();
}

RendererStandaloneFlags RendererStandalone::standaloneFlags() const {
  return state_->flags;
}

Mn::PixelFormat RendererStandalone::colorFramebufferFormat() const {
  return Mn::PixelFormat::RGBA8Unorm;
}

Mn::PixelFormat RendererStandalone::depthFramebufferFormat() const {
  return Mn::PixelFormat::Depth32F;
}

void RendererStandalone::draw() {
  state_->framebuffer.clear(Mn::GL::FramebufferClear::Color |
                            Mn::GL::FramebufferClear::Depth);
  Renderer::draw(state_->framebuffer);
}

Mn::Image2D RendererStandalone::colorImage() {
  /* Not using state_->framebuffer.viewport() as it's left pointing to whatever
     tile was rendered last */
  return state_->framebuffer.read({{}, tileCount() * tileSize()},
                                  colorFramebufferFormat());
}

void RendererStandalone::colorImageInto(const Magnum::Range2Di& rectangle,
                                        const Mn::MutableImageView2D& image) {
  /* Deliberately not checking that image.format() == colorFramebufferFormat()
     in order to allow for pixel format by the driver (such as RGBA to RGB) */
  CORRADE_ASSERT(rectangle.max() <= tileCount() * tileSize(),
                 "RendererStandalone::colorImageInto():"
                     << rectangle << "doesn't fit in a size of"
                     << tileCount() * tileSize(), );
  CORRADE_ASSERT(image.size() == rectangle.size(),
                 "RendererStandalone::colorImageInto(): expected image size of"
                     << rectangle.size() << "pixels but got" << image.size(), );
  return state_->framebuffer.read(rectangle, image);
}

Mn::Image2D RendererStandalone::depthImage() {
  /* Not using state_->framebuffer.viewport() as it's left pointing to whatever
     tile was rendered last */
  return state_->framebuffer.read({{}, tileCount() * tileSize()},
                                  depthFramebufferFormat());
}

void RendererStandalone::depthImageInto(const Magnum::Range2Di& rectangle,
                                        const Mn::MutableImageView2D& image) {
  /* Deliberately not checking that image.format() == depthFramebufferFormat()
     in order to allow for pixel format by the driver (such as 24-bit to 32-bit
     float) */
  CORRADE_ASSERT(rectangle.max() <= tileCount() * tileSize(),
                 "RendererStandalone::depthImageInto():"
                     << rectangle << "doesn't fit in a size of"
                     << tileCount() * tileSize(), );
  CORRADE_ASSERT(image.size() == rectangle.size(),
                 "RendererStandalone::depthImageInto(): expected image size of"
                     << rectangle.size() << "pixels but got" << image.size(), );
  return state_->framebuffer.read(rectangle, image);
}

#ifdef ESP_BUILD_WITH_CUDA
const void* RendererStandalone::colorCudaBufferDevicePointer() {
  /* If the CUDA buffer exists already, it's mapped from the previous call.
     Unmap it first so we can read into it from GL. */
  if (state_->cudaColorBuffer)
    checkCudaErrors(cudaGraphicsUnmapResources(1, &state_->cudaColorBuffer, 0));

  /* Read to the buffer image, allocating it if it's not already. Can't really
     return a pointer directly to the renderbuffer because the returned device
     pointer is expected to be linearized. */
  state_->framebuffer.read({{}, tileCount() * tileSize()}, state_->colorBuffer,
                           Mn::GL::BufferUsage::DynamicRead);

  /* Initialize the CUDA buffer from the GL buffer image if it's not already */
  if (!state_->cudaColorBuffer) {
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(
        &state_->cudaColorBuffer, state_->colorBuffer.buffer().id(),
        cudaGraphicsRegisterFlagsReadOnly));
  }

  /* Map the buffer and return the device pointer */
  checkCudaErrors(cudaGraphicsMapResources(1, &state_->cudaColorBuffer, 0));
  void* pointer;
  std::size_t size;
  checkCudaErrors(cudaGraphicsResourceGetMappedPointer(
      &pointer, &size, state_->cudaColorBuffer));
  CORRADE_INTERNAL_ASSERT(size == state_->colorBuffer.size().product() *
                                      state_->colorBuffer.pixelSize());
  return pointer;
}

const void* RendererStandalone::depthCudaBufferDevicePointer() {
  /* If the CUDA buffer exists already, it's mapped from the previous call.
     Unmap it first so we can read into it from GL. */
  if (state_->cudaDepthBuffer)
    checkCudaErrors(cudaGraphicsUnmapResources(1, &state_->cudaDepthBuffer, 0));

  /* Read to the buffer image, allocating it if it's not already. Can't really
     return a pointer directly to the renderbuffer because the returned device
     pointer is expected to be linearized. */
  state_->framebuffer.read({{}, tileCount() * tileSize()}, state_->depthBuffer,
                           Mn::GL::BufferUsage::DynamicRead);

  /* Initialize the CUDA buffer from the GL buffer image if it's not already */
  if (!state_->cudaDepthBuffer) {
    checkCudaErrors(cudaGraphicsGLRegisterBuffer(
        &state_->cudaDepthBuffer, state_->depthBuffer.buffer().id(),
        cudaGraphicsRegisterFlagsReadOnly));
  }

  /* Map the buffer and return the device pointer */
  checkCudaErrors(cudaGraphicsMapResources(1, &state_->cudaDepthBuffer, 0));
  void* pointer;
  std::size_t size;
  checkCudaErrors(cudaGraphicsResourceGetMappedPointer(
      &pointer, &size, state_->cudaDepthBuffer));
  CORRADE_INTERNAL_ASSERT(size == state_->depthBuffer.size().product() *
                                      state_->depthBuffer.pixelSize());
  return pointer;
}
#endif

}  // namespace gfx_batch
}  // namespace esp
