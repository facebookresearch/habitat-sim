// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayBatchRenderer.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/ReplayManager.h"
#include "esp/sensor/SensorFactory.h"

#include <Magnum/GL/Context.h>

namespace esp {
namespace sim {

ReplayBatchRenderer::ReplayBatchRenderer(
    const ReplayBatchRendererConfiguration& cfg) {
  config_ = cfg;
  SimulatorConfiguration simConfig;
  simConfig.createRenderer = true;
  auto metadataMediator = metadata::MetadataMediator::create(simConfig);
  assets::ResourceManager::Flags flags{};
  resourceManager_ =
      std::make_unique<assets::ResourceManager>(metadataMediator, flags);

  // after we get GfxReplay to sync lightSetups, we shouldn't need this
  resourceManager_->setLightSetup(gfx::getDefaultLights());

  sceneManager_ = scene::SceneManager::create_unique();

  for (int b = 0; b < config_.numEnvironments; b++) {
    auto cb = [this, b](const assets::AssetInfo& assetInfo,
                        const assets::RenderAssetInstanceCreationInfo& creation)
        -> scene::SceneNode* {
      return loadAndCreateRenderAssetInstance(b, assetInfo, creation);
    };
    auto sceneID = sceneManager_->initSceneGraph();
    auto semanticSceneID = cfg.forceSeparateSemanticSceneGraph
                               ? sceneManager_->initSceneGraph()
                               : sceneID;

    auto& sceneGraph = sceneManager_->getSceneGraph(sceneID);
    auto& parentNode = sceneGraph.getRootNode().createChild();
    auto sensorMap = esp::sensor::SensorFactory::createSensors(
        parentNode, cfg.sensorSpecifications);

    // not needed; python will do bind_render_target
    // for (const auto& kv : sensorMap) {
    //   sensor::VisualSensor& sensor =
    //       static_cast<sensor::VisualSensor&>(kv.second.get());
    //   renderer_->bindRenderTarget(sensor);
    // }

    envs_.emplace_back(EnvironmentRecord{.player_ = gfx::replay::Player(cb),
                                         .sceneID_ = sceneID,
                                         .semanticSceneID_ = semanticSceneID,
                                         .sensorParentNode_ = &parentNode,
                                         .sensorMap_ = std::move(sensorMap)});
  }

  // OpenGL context and renderer
  {
    // not sure what's going on here
    if (!context_ && !Magnum::GL::Context::hasCurrent()) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    gfx::Renderer::Flags flags;
#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER
    if (context_)
      flags |= gfx::Renderer::Flag::BackgroundRenderer;

    if (context_ && config_.leaveContextWithBackgroundRenderer)
      flags |= gfx::Renderer::Flag::LeaveContextWithBackgroundRenderer;
#else
#error unsupported
#endif
    renderer_ = gfx::Renderer::create(context_.get(), flags);

    renderer_->acquireGlContext();
  }
}

// ReplayBatchRenderer::~ReplayBatchRenderer() {
//   ESP_DEBUG() << "Deconstructing ReplayBatchRenderer";
//   close();
// }

void ReplayBatchRenderer::setSensorTransformsFromKeyframe(
    int envIndex,
    const std::string& prefix) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  ESP_CHECK(env.player_.getNumKeyframes() == 1,
            "setSensorTransformsFromKeyframe: for environment "
                << envIndex
                << ", you have not yet called setEnvironmentKeyframe.");
  for (const auto& kv : env.sensorMap_) {
    const auto sensorName = kv.first;
    sensor::VisualSensor& sensor =
        static_cast<sensor::VisualSensor&>(kv.second.get());

    std::string userName = prefix + sensorName;
    Mn::Vector3 translation;
    Mn::Quaternion rotation;
    bool found =
        env.player_.getUserTransform(userName, &translation, &rotation);
    ESP_CHECK(found,
              "setSensorTransformsFromKeyframe: couldn't find user transform \""
                  << userName << "\" for environment " << envIndex << ".");
    sensor.node().setRotation(rotation);
    sensor.node().setTranslation(translation);
  }
}

esp::scene::SceneNode* ReplayBatchRenderer::getEnvironmentSensorParentNode(
    int envIndex) const {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  return env.sensorParentNode_;
}

std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
ReplayBatchRenderer::getEnvironmentSensors(int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  return env.sensorMap_;
}

esp::scene::SceneGraph& ReplayBatchRenderer::getSceneGraph(int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.sceneID_);
}

esp::scene::SceneGraph& ReplayBatchRenderer::getSemanticSceneGraph(
    int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.semanticSceneID_ == ID_UNDEFINED
                                          ? env.sceneID_
                                          : env.semanticSceneID_);
}

void ReplayBatchRenderer::setEnvironmentKeyframe(
    int envIndex,
    const std::string& serKeyframe) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  env.player_.setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromString(serKeyframe));
}

scene::SceneNode* ReplayBatchRenderer::loadAndCreateRenderAssetInstance(
    int envIndex,
    const assets::AssetInfo& assetInfo,
    const assets::RenderAssetInstanceCreationInfo& creation) {
  // Note this pattern of passing the scene manager and two scene ids to
  // resource manager. This is similar to ResourceManager::loadStage.
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  // perf todo: avoid dynamic mem alloc
  std::vector<int> tempIDs{env.sceneID_, env.semanticSceneID_};
  return resourceManager_->loadAndCreateRenderAssetInstance(
      assetInfo, creation, sceneManager_.get(), tempIDs);
}

}  // namespace sim
}  // namespace esp
