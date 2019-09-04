// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class WindowlessContext {
 public:
  explicit WindowlessContext(int gpuDevice = 0);

  ~WindowlessContext() { LOG(INFO) << "Deconstructing WindowlessContext"; }

  void makeCurrent();

  int gpuDevice() const;

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(WindowlessContext)
};

}  // namespace gfx
}  // namespace esp
