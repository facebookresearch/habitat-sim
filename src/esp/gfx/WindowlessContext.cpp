// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "WindowlessContext.h"

#include <Corrade/configure.h>
#include "esp/core/configure.h"

#ifdef MAGNUM_TARGET_EGL
#include <Magnum/Platform/WindowlessEglApplication.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
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

#include <Magnum/Platform/GLContext.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace gfx {

struct WindowlessContext::Impl {
  explicit Impl(int device)
      : device_{device},
        magnumGLContext_{Mn::NoCreate},
        windowlessGLContext_{Mn::NoCreate} {
    Mn::Platform::WindowlessGLContext::Configuration config;

#if defined(CORRADE_TARGET_UNIX) && !defined(CORRADE_TARGET_APPLE)
#ifdef MAGNUM_TARGET_EGL
#ifdef ESP_BUILD_WITH_CUDA
    // With CUDA, map the gpu device ID to a CUDA EGL device so that EGL
    // rendering and CUDA compute target the same GPU.  device == -1 means
    // "use the default EGL device without CUDA mapping".
    if (device != -1) {
      config.setCudaDevice(device);
    }
#else   // NO ESP_BUILD_WITH_CUDA
    // Without CUDA, select an EGL device directly by index.  This avoids
    // Magnum's CUDA device search path which fails when no CUDA-capable GPU
    // is present (e.g. Mesa software rendering on aarch64).  device == 0 is
    // the default EGL device and requires no explicit selection.
    if (device > 0) {
      config.setDevice(device);
    }
#endif  // ESP_BUILD_WITH_CUDA
#else   // NO MAGNUM_TARGET_EGL
    if (device != 0)
      Mn::Fatal{} << "GLX context does not support multiple GPUs. Please "
                     "compile with --headless for multi-gpu support via EGL";

    if (std::getenv("DISPLAY") == nullptr)
      Mn::Fatal{} << "DISPLAY not detected. For headless systems, compile with "
                     "--headless for EGL support";
#endif
#endif

    windowlessGLContext_ =
        Mn::Platform::WindowlessGLContext{config, &magnumGLContext_};

    if (!windowlessGLContext_.isCreated())
      Mn::Fatal{} << "WindowlessContext: Unable to create windowless context";

    makeCurrentPlatform();

    if (!magnumGLContext_.tryCreate())
      Mn::Fatal{} << "WindowlessContext: Failed to create OpenGL context";
  }

  void makeCurrent() {
    makeCurrentPlatform();
    Mn::GL::Context::makeCurrent(&magnumGLContext_);
  }

  bool makeCurrentPlatform() {
    if (!windowlessGLContext_.makeCurrent())
      Mn::Fatal{} << "Failed to make platform current";

    return true;
  }

  void release() {
    Mn::GL::Context::makeCurrent(nullptr);
    releasePlatform();
  }

  void releasePlatform() { windowlessGLContext_.release(); }

  int gpuDevice() const { return device_; }

 private:
  int device_;
  Mn::Platform::GLContext magnumGLContext_;
  Mn::Platform::WindowlessGLContext windowlessGLContext_;
};

WindowlessContext::WindowlessContext(int device /* = 0 */)
    : pimpl_(spimpl::make_unique_impl<Impl>(device)) {}

void WindowlessContext::makeCurrent() {
  pimpl_->makeCurrent();
}

void WindowlessContext::makeCurrentPlatform() {
  pimpl_->makeCurrentPlatform();
}

void WindowlessContext::release() {
  pimpl_->release();
}

void WindowlessContext::releasePlatform() {
  pimpl_->releasePlatform();
}

int WindowlessContext::gpuDevice() const {
  return pimpl_->gpuDevice();
}

}  // namespace gfx
}  // namespace esp
