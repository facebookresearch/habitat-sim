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
#include <Magnum/Platform/GLContext.h>

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

// Based on code from:
// https://devblogs.nvidia.com/parallelforall/egl-eye-opengl-visualization-without-x-server/
// https://github.com/facebookresearch/House3D/blob/master/renderer/gl/glContext.cc

namespace esp {
namespace gfx {

#if defined(CORRADE_TARGET_UNIX) && !defined(CORRADE_TARGET_APPLE)

namespace {

struct ESPContext {
  virtual void makeCurrent() = 0;
  virtual bool isValid() = 0;
  virtual int gpuDevice() const = 0;

  virtual ~ESPContext(){};

  ESP_SMART_POINTERS(ESPContext);
};

#ifdef ESP_BUILD_EGL_SUPPORT
const int MAX_DEVICES = 128;

#define CHECK_EGL_ERROR()                             \
  do {                                                \
    EGLint err = eglGetError();                       \
    CHECK(err == EGL_SUCCESS) << "EGL error:" << err; \
  } while (0)

struct ESPEGLContext : ESPContext {
  explicit ESPEGLContext(int device)
      : magnumGlContext_{Mn::NoCreate},
        eglContext_{
            Mn::Platform::WindowlessEglContext::Configuration{}.setCudaDevice(
                device),
            &magnumGlContext_},
        gpuDevice_{device} {
    CHECK(eglContext_.isCreated())
        << "[EGL] Failed to create headless EGL context";

    makeCurrent();

    CHECK(magnumGlContext_.tryCreate())
        << "[EGL] Failed to create OpenGL Context";
    isValid_ = true;
  }

  void makeCurrent() { eglContext_.makeCurrent(); };

  bool isValid() { return isValid_; };

  int gpuDevice() const { return gpuDevice_; }

  ~ESPEGLContext() {}

 private:
  Mn::Platform::GLContext magnumGlContext_;
  Mn::Platform::WindowlessEglContext eglContext_;
  bool isValid_ = false;
  int gpuDevice_;

  ESP_SMART_POINTERS(ESPEGLContext);
};

#else  // ESP_BUILD_EGL_SUPPORT not defined

struct ESPGLXContext : ESPContext {
  ESPGLXContext()
      : magnumGlContext_{Mn::NoCreate},
        glxCtx_{Mn::Platform::WindowlessGlxContext::Configuration{},
                &magnumGlContext_} {
    CHECK(glxCtx_.isCreated())
        << "[GLX] Failed to created headless glX context";

    makeCurrent();

    CHECK(magnumGlContext_.tryCreate())
        << "[GLX] Failed to create OpenGL Context";
    isValid_ = true;
  };

  void makeCurrent() { glxCtx_.makeCurrent(); };
  bool isValid() { return isValid_; };
  int gpuDevice() const { return 0; }

 private:
  Mn::Platform::GLContext magnumGlContext_;
  Mn::Platform::WindowlessGlxContext glxCtx_;
  bool isValid_ = false;

  ESP_SMART_POINTERS(ESPGLXContext);
};

#endif

};  // namespace

struct WindowlessContext::Impl {
  explicit Impl(int device) {
#ifdef ESP_BUILD_EGL_SUPPORT
    glContext_ = ESPEGLContext::create_unique(device);
#else
    CHECK_EQ(device, 0)
        << "glX context does not support multiple GPUs. Please compile with "
           "BUILD_GUI_VIEWERS=0 for multi-gpu support via EGL";
    CHECK(std::getenv("DISPLAY") != nullptr)
        << "DISPLAY not detected. For headless systems, compile with "
           "--headless for EGL support";

    glContext_ = ESPGLXContext::create_unique();
#endif

    makeCurrent();
  }

  ~Impl() { LOG(INFO) << "Deconstructing GL context"; }

  void makeCurrent() { glContext_->makeCurrent(); }

  int gpuDevice() const { return glContext_->gpuDevice(); }

  ESPContext::uptr glContext_ = nullptr;
};

#else  // not defined(CORRADE_TARGET_UNIX) && !defined(CORRADE_TARGET_APPLE)

struct WindowlessContext::Impl {
  explicit Impl(int) : glContext_({}), magnumGlContext_(Mn::NoCreate) {
    glContext_.makeCurrent();
    if (!magnumGlContext_.tryCreate()) {
      LOG(ERROR) << "Failed to create GL context";
    }
  }

  ~Impl() { LOG(INFO) << "Deconstructing GL context"; }

  void makeCurrent() { glContext_.makeCurrent(); }

  int gpuDevice() const { return 0; }

  Mn::Platform::WindowlessGLContext glContext_;
  Mn::Platform::GLContext magnumGlContext_;
};

#endif

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
