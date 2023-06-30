// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_BATCHREPLAYRENDERER_H_
#define ESP_SIM_BATCHREPLAYRENDERER_H_

#include "esp/gfx/replay/Player.h"
#include "esp/gfx_batch/RendererStandalone.h"
#include "esp/sim/AbstractReplayRenderer.h"

namespace esp {
namespace sim {

class BatchReplayRenderer : public AbstractReplayRenderer {
 public:
  // TODO figure out a better way how to abstract this so i don't need to
  //  propagate each and every batch-renderer-specific option through all these
  //  layers like an animal
  explicit BatchReplayRenderer(
      const ReplayRendererConfiguration& cfg,
      gfx_batch::RendererConfiguration&& batchRendererConfiguration =
          gfx_batch::RendererConfiguration{});

  ~BatchReplayRenderer() override;

  const void* getCudaColorBufferDevicePointer() override;

  const void* getCudaDepthBufferDevicePointer() override;

 private:
  void doClose() override;

  void doCloseImpl();

  void doPreloadFile(Corrade::Containers::StringView filename) override;

  unsigned doEnvironmentCount() const override;

  Magnum::Vector2i doSensorSize(unsigned envIndex) override;

  esp::gfx::replay::Player& doPlayerFor(unsigned envIndex) override;

  void doSetSensorTransform(unsigned envIndex,
                            const std::string& sensorName,
                            const Mn::Matrix4& transform) override;

  void doSetSensorTransformsFromKeyframe(unsigned envIndex,
                                         const std::string& prefix) override;

  void doRender(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                    colorImageViews,
                Corrade::Containers::ArrayView<const Magnum::MutableImageView2D>
                    depthImageViews) override;

  void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) override;

  esp::geo::Ray doUnproject(unsigned envIndex,
                            const Mn::Vector2i& viewportPosition) override;

  /* If standalone_ is true, renderer_ contains a RendererStandalone. Has to be
     before the EnvironmentRecord array because Player calls
     gfx_batch::Renderer::clear() on destruction. */
  bool standalone_;
  Corrade::Containers::Pointer<esp::gfx_batch::Renderer> renderer_;

  // TODO pimpl all this?
  struct EnvironmentRecord {
    std::shared_ptr<gfx::replay::AbstractPlayerImplementation>
        playerImplementation_;
    gfx::replay::Player player_{playerImplementation_};
  };
  Corrade::Containers::Array<EnvironmentRecord> envs_;

  Corrade::Containers::String theOnlySensorName_;
  Mn::Matrix4 theOnlySensorProjection_;

  ESP_SMART_POINTERS(BatchReplayRenderer)
};

}  // namespace sim
}  // namespace esp

#endif
