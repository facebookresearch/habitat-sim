// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_REPLAYBATCHRENDERER_H_
#define ESP_SIM_REPLAYBATCHRENDERER_H_

#include "esp/gfx/replay/Player.h"
#include "esp/gfx_batch/RendererStandalone.h"
#include "esp/sim/AbstractReplayRenderer.h"

namespace esp {
namespace sim {

class BatchReplayRenderer : public AbstractReplayRenderer {
 public:
  explicit BatchReplayRenderer(const ReplayRendererConfiguration& cfg);

  ~BatchReplayRenderer() override;

 private:
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
                    imageViews) override;

  void doRender(Magnum::GL::AbstractFramebuffer& framebuffer) override;

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
