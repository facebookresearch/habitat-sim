// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_REPLAYBATCHRENDERER_H_
#define ESP_SIM_REPLAYBATCHRENDERER_H_

#include "esp/gfx/WindowlessContext.h"
#include "esp/gfx_batch/RendererStandalone.h"
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

class ReplayRendererConfiguration {
 public:
  int numEnvironments = 1;
  //! The system GPU device to use for rendering.
  int gpuDeviceId = 0;
  bool forceSeparateSemanticSceneGraph = false;
  /**
   * @brief Leave the context with the background thread after finishing draw
   * jobs. This will improve performance as transferring the OpenGL context back
   * and forth takes time but will require the user to manually transfer the
   * context back to the main thread before adding or removing objects.
   */
  bool leaveContextWithBackgroundRenderer = false;

  std::vector<sensor::SensorSpec::ptr> sensorSpecifications;

  ESP_SMART_POINTERS(ReplayRendererConfiguration)
};

class AbstractReplayRenderer {
 public:
  virtual ~AbstractReplayRenderer();

  // Assumes there's just one sensor per env
  virtual Magnum::Vector2i sensorSize(int envIndex) = 0;

  virtual void setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe) = 0;

  // You must have done Recorder::addUserTransformToKeyframe(prefix +
  // sensorName, ...) for every sensor in
  // ReplayRendererConfiguration::sensorSpecifications, for the specified
  // environment's keyframe. See also setEnvironmentKeyframe.
  virtual void setSensorTransformsFromKeyframe(int envIndex, const std::string& prefix) = 0;

  // Renders and waits for the render to finish
  virtual void render(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D> imageViews) = 0;
};

class ReplayRenderer: public AbstractReplayRenderer {
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

  explicit ReplayRenderer(const ReplayRendererConfiguration& cfg);

  ~ReplayRenderer();

  Magnum::Vector2i sensorSize(int envIndex) override;

  void setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe) override;

  void setSensorTransformsFromKeyframe(int envIndex, const std::string& prefix) override;

  void render(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D> imageViews) override;

  // TODO those are used by bindings at the moment, but should eventually
  //  become private as well
  std::shared_ptr<gfx::Renderer> getRenderer() { return renderer_; }

  esp::scene::SceneGraph& getSceneGraph(int envIndex);
  esp::scene::SceneGraph& getSemanticSceneGraph(int envIndex);

  esp::scene::SceneNode* getEnvironmentSensorParentNode(int envIndex) const;
  std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
  getEnvironmentSensors(int envIndex);

 private:
  gfx::replay::GfxReplayNode* loadAndCreateRenderAssetInstance(
      int envIndex,
      const assets::AssetInfo& assetInfo,
      const assets::RenderAssetInstanceCreationInfo& creation);

  std::vector<EnvironmentRecord> envs_;

  std::unique_ptr<assets::ResourceManager> resourceManager_ = nullptr;

  scene::SceneManager::uptr sceneManager_ = nullptr;

  gfx::WindowlessContext::uptr context_ = nullptr;
  std::shared_ptr<gfx::Renderer> renderer_ = nullptr;

  ReplayRendererConfiguration config_;

  ESP_SMART_POINTERS(ReplayRenderer)
};

class ReplayBatchRenderer: public AbstractReplayRenderer {
 public:
  explicit ReplayBatchRenderer(const ReplayRendererConfiguration& cfg);

  ~ReplayBatchRenderer() override;

  Magnum::Vector2i sensorSize(int envIndex) override;

  void setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe) override;

  void setSensorTransformsFromKeyframe(int envIndex, const std::string& prefix) override;

  void render(Corrade::Containers::ArrayView<const Magnum::MutableImageView2D> imageViews) override;
 private:
   // TODO pimpl all this?
  struct EnvironmentRecord {
    esp::gfx::replay::Player player_;
  };
  Corrade::Containers::Array<EnvironmentRecord> envs_;

  esp::gfx_batch::RendererStandalone renderer_;
  Corrade::Containers::String theOnlySensorName_;
  Mn::Matrix4 theOnlySensorProjection_;
};

}  // namespace sim
}  // namespace esp

#endif
