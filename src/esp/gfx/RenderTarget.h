// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/Magnum.h>

#include "esp/core/esp.h"

#include "esp/gfx/DepthUnprojection.h"

namespace esp {
namespace gfx {

class RenderTarget {
 public:
  RenderTarget(const Magnum::Vector2i& size,
               const Magnum::Vector2& depthUnprojection,
               DepthShader* depthShader);

  RenderTarget(const Magnum::Vector2i& size,
               const Magnum::Vector2& depthUnprojection)
      : RenderTarget{size, depthUnprojection, nullptr} {};

  ~RenderTarget() { LOG(INFO) << "Deconstructing RenderTarget"; }

  void renderEnter();
  void renderExit();

  Magnum::Vector2i framebufferSize() const;

  void readFrameRgba(const Magnum::MutableImageView2D& view);
  void readFrameDepth(const Magnum::MutableImageView2D& view);
  void readFrameObjectId(const Magnum::MutableImageView2D& view);

  // Delete copy
  RenderTarget(const RenderTarget&) = delete;
  RenderTarget& operator=(const RenderTarget&) = delete;

#ifdef ESP_BUILD_WITH_CUDA
  /**
   * @brief Reads the RGBA frame-buffer directly into CUDA memory specified by
   * devPtr
   */
  void readFrameRgbaGPU(uint8_t* devPtr);

  /**
   * @brief Reads the Depth frame-buffer directly into CUDA memory specified by
   * devPtr
   */
  void readFrameDepthGPU(float* devPtr);

  /**
   * @brief Reads the ObjectID frame-buffer directly into CUDA memory specified
   * by devPtr
   */
  void readFrameObjectIdGPU(int32_t* devPtr);
#endif

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(RenderTarget);
};

}  // namespace gfx
}  // namespace esp
