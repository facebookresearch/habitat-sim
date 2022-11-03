// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_WINDOWLESSCONTEXT_H_
#define ESP_GFX_WINDOWLESSCONTEXT_H_

#include "esp/core/Esp.h"

namespace esp {
namespace gfx {

class WindowlessContext {
 public:
  explicit WindowlessContext(int gpuDevice = 0);

  ~WindowlessContext() { ESP_DEBUG() << "Deconstructing WindowlessContext"; }

  void makeCurrent();
  void makeCurrentPlatform();

  void release();
  void releasePlatform();

  int gpuDevice() const;

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(WindowlessContext)
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_WINDOWLESSCONTEXT_H_
