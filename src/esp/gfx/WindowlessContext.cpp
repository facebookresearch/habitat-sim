// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "WindowlessContext.h"

#include <Corrade/configure.h>

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

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#elif defined(CORRADE_TARGET_WINDOWS)
#include <Magnum/Platform/WindowlessWglApplication.h>
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
#ifdef ESP_BUILD_EGL_SUPPORT
    config.setCudaDevice(device);
#else  // NO ESP_BUILD_EGL_SUPPORT
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

    makeCurrent();

    if (!magnumGLContext_.tryCreate())
      Mn::Fatal{} << "WindowlessContext: Failed to create OpenGL context";
  }

  void makeCurrent() { windowlessGLContext_.makeCurrent(); }

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

int WindowlessContext::gpuDevice() const {
  return pimpl_->gpuDevice();
}

}  // namespace gfx
}  // namespace esp
