// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_BATCH_RENDERER_STANDALONE_H_
#define ESP_GFX_BATCH_RENDERER_STANDALONE_H_

#include "Renderer.h"

// TODO: ideally this would go directly to esp/, so we don't depend on core
#include "esp/core/configure.h"

namespace esp {
namespace gfx_batch {

enum class RendererStandaloneFlag { QuietLog = 1 << 0 };
typedef Corrade::Containers::EnumSet<RendererStandaloneFlag>
    RendererStandaloneFlags;
CORRADE_ENUMSET_OPERATORS(RendererStandaloneFlags)

struct RendererStandaloneConfiguration {
  explicit RendererStandaloneConfiguration();
  ~RendererStandaloneConfiguration();

  RendererStandaloneConfiguration& setCudaDevice(Magnum::UnsignedInt id);
  RendererStandaloneConfiguration& setFlags(RendererStandaloneFlags flags);

  struct State;
  Corrade::Containers::Pointer<State> state;
};

class RendererStandalone : public Renderer {
 public:
  explicit RendererStandalone(
      const RendererConfiguration& configuration,
      const RendererStandaloneConfiguration& standaloneConfiguration);
  ~RendererStandalone();

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

}  // namespace gfx_batch
}  // namespace esp

#endif
