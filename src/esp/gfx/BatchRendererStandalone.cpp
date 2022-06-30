// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchRendererStandalone.h"

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/StridedArrayView.h>
#include <Magnum/GL/BufferImage.h>
#include <Magnum/GL/Framebuffer.h>
#include <Magnum/GL/Renderbuffer.h>
#include <Magnum/GL/RenderbufferFormat.h>
#include <Magnum/Image.h>
#include <Magnum/PixelFormat.h>

#if defined(CORRADE_TARGET_APPLE)
#include <Magnum/Platform/WindowlessCglApplication.h>
#elif defined(CORRADE_TARGET_EMSCRIPTEN)
#include <Magnum/Platform/WindowlessEglApplication.h>
#elif defined(CORRADE_TARGET_UNIX)
#ifdef ESP_BUILD_EGL_SUPPORT
#include <Magnum/Platform/WindowlessEglApplication.h>
#else
#include <Magnum/Platform/WindowlessGlxApplication.h>
#endif
#elif defined(CORRADE_TARGET_WINDOWS)
#include <Magnum/Platform/WindowlessWglApplication.h>
#endif

#ifdef ESP_BUILD_WITH_CUDA
#include <cuda_gl_interop.h>

#include "HelperCuda.h"
#endif

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

struct BatchRendererStandaloneConfiguration::State {
  /* Not picking any CUDA device by default */
  Magnum::UnsignedInt cudaDevice = ~Magnum::UnsignedInt{};
  BatchRendererStandaloneFlags flags;
};

BatchRendererStandaloneConfiguration::BatchRendererStandaloneConfiguration()
    : state{Cr::InPlaceInit} {}
BatchRendererStandaloneConfiguration::~BatchRendererStandaloneConfiguration() =
    default;

BatchRendererStandaloneConfiguration&
BatchRendererStandaloneConfiguration::setCudaDevice(Magnum::UnsignedInt id) {
  state->cudaDevice = id;
  return *this;
}

BatchRendererStandaloneConfiguration&
BatchRendererStandaloneConfiguration::setFlags(
    BatchRendererStandaloneFlags flags) {
  state->flags = flags;
  return *this;
}

struct BatchRendererStandalone::State {
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

  explicit State(const BatchRendererStandaloneConfiguration& configuration)
      : context{Mn::Platform::WindowlessGLContext::Configuration{}
#ifdef ESP_BUILD_EGL_SUPPORT
                    .setCudaDevice(configuration.state->cudaDevice)
#endif
                    .addFlags(configuration.state->flags &
                                      BatchRendererStandaloneFlag::QuietLog
                                  ? Mn::Platform::WindowlessGLContext::
                                        Configuration::Flag::QuietLog
                                  : Mn::Platform::WindowlessGLContext::
                                        Configuration::Flags{})} {
    context.makeCurrent();
    magnumContext.create(Mn::GL::Context::Configuration{}.addFlags(
        configuration.state->flags & BatchRendererStandaloneFlag::QuietLog
            ? Mn::GL::Context::Configuration::Flag::QuietLog
            : Mn::GL::Context::Configuration::Flags{}));
    color = Mn::GL::Renderbuffer{};
    depth = Mn::GL::Renderbuffer{};
  }

#ifdef ESP_BUILD_WITH_CUDA
  ~State() {
    /* Should be unmapped before the GL object gets destroyed, I guess? */
    if (cudaColorBuffer)
      checkCudaErrors(cudaGraphicsUnmapResources(1, &cudaColorBuffer, 0));
    if (cudaDepthBuffer)
      checkCudaErrors(cudaGraphicsUnmapResources(1, &cudaDepthBuffer, 0));
  }
#endif
};

BatchRendererStandalone::BatchRendererStandalone(
    const BatchRendererConfiguration& configuration,
    const BatchRendererStandaloneConfiguration& standaloneConfiguration)
    : BatchRenderer{Mn::NoCreate},
      state_{Cr::InPlaceInit, standaloneConfiguration} {
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

BatchRendererStandalone::~BatchRendererStandalone() {
  /* Yup, shitty, but as we hold the GL context we can't let any GL resources
     to be destructed after our destructor. Better ideas? */
  BatchRenderer::destroy();
}

Mn::PixelFormat BatchRendererStandalone::colorFramebufferFormat() const {
  return Mn::PixelFormat::RGBA8Unorm;
}

Mn::PixelFormat BatchRendererStandalone::depthFramebufferFormat() const {
  return Mn::PixelFormat::Depth32F;
}

void BatchRendererStandalone::draw() {
  state_->framebuffer.clear(Mn::GL::FramebufferClear::Color |
                            Mn::GL::FramebufferClear::Depth);
  BatchRenderer::draw(state_->framebuffer);
}

Mn::Image2D BatchRendererStandalone::colorImage() {
  /* Not using state_->framebuffer.viewport() as it's left pointing to whatever
     tile was rendered last */
  return state_->framebuffer.read({{}, tileCount() * tileSize()},
                                  colorFramebufferFormat());
}

Mn::Image2D BatchRendererStandalone::depthImage() {
  /* Not using state_->framebuffer.viewport() as it's left pointing to whatever
     tile was rendered last */
  return state_->framebuffer.read({{}, tileCount() * tileSize()},
                                  depthFramebufferFormat());
}

#ifdef ESP_BUILD_WITH_CUDA
const void* BatchRendererStandalone::colorCudaBufferDevicePointer() {
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

const void* BatchRendererStandalone::depthCudaBufferDevicePointer() {
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

}  // namespace gfx
}  // namespace esp
