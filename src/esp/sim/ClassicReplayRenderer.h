// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_CLASSICBATCHRENDERER_H_
#define ESP_SIM_CLASSICBATCHRENDERER_H_

#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx/replay/Player.h"
#include "esp/scene/SceneManager.h"
#include "esp/sim/AbstractReplayRenderer.h"

namespace esp {
namespace assets {
class ResourceManager;
}  // namespace assets
namespace gfx {
class Renderer;
}
}  // namespace esp

namespace esp {
namespace sim {

class ClassicReplayRenderer : public AbstractReplayRenderer {
 public:
  struct EnvironmentRecord {
    explicit EnvironmentRecord(
        std::shared_ptr<gfx::replay::AbstractSceneGraphPlayerImplementation>
            playerImplementation)
        : playerImplementation_{std::move(playerImplementation)},
          player_{playerImplementation_} {}
    std::shared_ptr<gfx::replay::AbstractSceneGraphPlayerImplementation>
        playerImplementation_;
    esp::gfx::replay::Player player_;
    int sceneID_ = ID_UNDEFINED;
    int semanticSceneID_ = ID_UNDEFINED;
    esp::scene::SceneNode* sensorParentNode_ = nullptr;
    std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>
        sensorMap_;
  };

  explicit ClassicReplayRenderer(const ReplayRendererConfiguration& cfg);

  ~ClassicReplayRenderer() override;

  // TODO those are used by bindings at the moment, but should eventually
  //  become private as well
  std::shared_ptr<gfx::Renderer> getRenderer() { return renderer_; }

  esp::scene::SceneGraph& getSceneGraph(unsigned envIndex);
  esp::scene::SceneGraph& getSemanticSceneGraph(unsigned envIndex);

  esp::scene::SceneNode* getEnvironmentSensorParentNode(
      unsigned envIndex) const;
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getEnvironmentSensors(unsigned envIndex);

  esp::geo::Ray doUnproject(unsigned envIndex,
                            const Mn::Vector2i& viewportPosition) override;

 private:
  void doClose() override;

  void doCloseImpl();

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

  gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
      unsigned envIndex,
      const assets::AssetInfo& assetInfo,
      const assets::RenderAssetInstanceCreationInfo& creation);

  std::vector<EnvironmentRecord> envs_;

  std::unique_ptr<assets::ResourceManager> resourceManager_;

  scene::SceneManager::uptr sceneManager_ = nullptr;

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;

  ReplayRendererConfiguration config_;

  ESP_SMART_POINTERS(ClassicReplayRenderer)
};

}  // namespace sim
}  // namespace esp

#endif
