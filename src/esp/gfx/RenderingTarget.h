// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Magnum.h>

#include "esp/core/esp.h"

#include "WindowlessContext.h"

namespace esp {
namespace gfx {

class RenderingTarget {
 public:
  RenderingTarget(WindowlessContext::ptr context, const Magnum::Vector2i& size);

  ~RenderingTarget() { LOG(INFO) << "Deconstructing RenderingTarget"; }

  void renderEnter();
  void renderExit();

  Magnum::Vector2i framebufferSize() const;

  void readFrameRgba(uint8_t* ptr);
  void readFrameDepth(float* ptr);
  void readFrameObjectId(uint32_t* ptr);

  int gpuDeviceId() const;

#ifdef ESP_WITH_GPU_GPU
  void readFrameRgbaGPU(uint8_t* ptr);
  void readFrameDepthGPU(float* ptr);
  void readFrameObjectIdGPU(int32_t* ptr);
#endif

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(RenderingTarget);
};

}  // namespace gfx
}  // namespace esp
