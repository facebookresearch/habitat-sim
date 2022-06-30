// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BATCH_RENDERER_STANDALONE_H_
#define ESP_GFX_BATCH_RENDERER_STANDALONE_H_

#include "BatchRenderer.h"

#include "esp/core/configure.h"

namespace esp {
namespace gfx {

enum class BatchRendererStandaloneFlag { QuietLog = 1 << 0 };
typedef Corrade::Containers::EnumSet<BatchRendererStandaloneFlag>
    BatchRendererStandaloneFlags;
CORRADE_ENUMSET_OPERATORS(BatchRendererStandaloneFlags)

struct BatchRendererStandaloneConfiguration {
  explicit BatchRendererStandaloneConfiguration();
  ~BatchRendererStandaloneConfiguration();

  BatchRendererStandaloneConfiguration& setCudaDevice(Magnum::UnsignedInt id);
  BatchRendererStandaloneConfiguration& setFlags(
      BatchRendererStandaloneFlags flags);

  struct State;
  Corrade::Containers::Pointer<State> state;
};

class BatchRendererStandalone : public BatchRenderer {
 public:
  explicit BatchRendererStandalone(
      const BatchRendererConfiguration& configuration,
      const BatchRendererStandaloneConfiguration& standaloneConfiguration);
  ~BatchRendererStandalone();

  Magnum::PixelFormat colorFramebufferFormat() const;
  Magnum::PixelFormat depthFramebufferFormat() const;

  void draw();

  Magnum::Image2D colorImage();
  Magnum::Image2D depthImage();

#ifdef ESP_BUILD_WITH_CUDA
  const void* colorCudaBufferDevicePointer();
  const void* depthCudaBufferDevicePointer();
#endif

 private:
  struct State;
  Corrade::Containers::Pointer<State> state_;
};

}  // namespace gfx
}  // namespace esp

#endif
