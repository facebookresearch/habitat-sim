// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/GL/Framebuffer.h>

#include "esp/core/esp.h"

#include "WindowlessContext.h"

namespace esp {
namespace gfx {

class RenderingTarget {
 public:
  static const Magnum::GL::Framebuffer::ColorAttachment RgbaBuffer;
  static const Magnum::GL::Framebuffer::ColorAttachment DepthBuffer;
  static const Magnum::GL::Framebuffer::ColorAttachment ObjectIdBuffer;

  RenderingTarget(WindowlessContext::ptr context, const Magnum::Vector2i& size);

  ~RenderingTarget() { LOG(INFO) << "Deconstructing RenderingTarget"; }

  void renderEnter();
  void renderExit();

  Magnum::Vector2i framebufferSize() const;

  Magnum::GL::Framebuffer& framebuffer();

  int gpuDeviceId() const;

#ifdef ESP_WITH_GPU_GPU
  void readFrameRgbaGPU(uint8_t* devPtr);
  void readFrameDepthGPU(float* devPtr);
  void readFrameObjectIdGPU(int32_t* devPtr);
#endif

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(RenderingTarget);
};

}  // namespace gfx
}  // namespace esp
