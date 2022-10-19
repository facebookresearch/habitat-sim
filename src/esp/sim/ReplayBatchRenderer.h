// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_REPLAYBATCHRENDERER_H_
#define ESP_SIM_REPLAYBATCHRENDERER_H_

#include "esp/assets/Asset.h"
#include "esp/assets/RenderAssetInstanceCreationInfo.h"
#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx/replay/Player.h"
#include "esp/scene/SceneManager.h"

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

class ReplayBatchRendererConfiguration {
 public:
  int numEnvironments = 1;
  //! The system GPU device to use for rendering.
  int gpuDeviceId = 0;
  bool forceSeparateSemanticSceneGraph = false;
  /**
   * @brief Leave the context with the background thread after finishing draw
   * jobs. This will improve performance as transfering the OpenGL context back
   * and forth takes time but will require the user to manually transfer the
   * context back to the main thread before adding or removing objects.
   */
  bool leaveContextWithBackgroundRenderer = false;

  std::vector<sensor::SensorSpec::ptr> sensorSpecifications;

  ESP_SMART_POINTERS(ReplayBatchRendererConfiguration)
};

class ReplayBatchRenderer {
 public:
  class EnvironmentRecord {
   public:
    esp::gfx::replay::Player player_;
    int sceneID_ = ID_UNDEFINED;
    int semanticSceneID_ = ID_UNDEFINED;
    esp::scene::SceneNode* sensorParentNode_ = nullptr;
    std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>
        sensorMap_;
  };

  explicit ReplayBatchRenderer(const ReplayBatchRendererConfiguration& cfg);
  ~ReplayBatchRenderer();

  std::shared_ptr<gfx::Renderer> getRenderer() { return renderer_; }

  esp::scene::SceneGraph& getSceneGraph(int envIndex);
  esp::scene::SceneGraph& getSemanticSceneGraph(int envIndex);

  esp::scene::SceneNode* getEnvironmentSensorParentNode(int envIndex) const;
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getEnvironmentSensors(int envIndex);

  void setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe);

  // You must have done Recorder::addUserTransformToKeyframe(prefix +
  // sensorName, ...) for every sensor in
  // ReplayBatchRendererConfiguration::sensorSpecifications, for the specified
  // environment's keyframe. See also setEnvironmentKeyframe.
  void setSensorTransformsFromKeyframe(int envIndex, const std::string& prefix);

 private:
  scene::SceneNode* loadAndCreateRenderAssetInstance(
      int envIndex,
      const assets::AssetInfo& assetInfo,
      const assets::RenderAssetInstanceCreationInfo& creation);

  std::vector<EnvironmentRecord> envs_;

  std::unique_ptr<assets::ResourceManager> resourceManager_ = nullptr;

  scene::SceneManager::uptr sceneManager_ = nullptr;

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;

  ReplayBatchRendererConfiguration config_;

  ESP_SMART_POINTERS(ReplayBatchRenderer)
};

}  // namespace sim
}  // namespace esp

#endif
