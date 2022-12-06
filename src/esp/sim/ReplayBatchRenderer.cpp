// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayBatchRenderer.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/replay/PlayerCallbacks.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/sensor/SensorFactory.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sim/SimulatorConfiguration.h"

#include <Corrade/Utility/Algorithms.h>
#include <Magnum/GL/Context.h>
#include <Magnum/ImageView.h>
#include <Magnum/Image.h>

namespace esp {
namespace sim {

using namespace Mn::Math::Literals;

AbstractReplayRenderer::~AbstractReplayRenderer() = default;

ReplayRenderer::ReplayRenderer(
    const ReplayRendererConfiguration& cfg) {
  config_ = cfg;
  SimulatorConfiguration simConfig;
  simConfig.createRenderer = true;
  auto metadataMediator = metadata::MetadataMediator::create(simConfig);
  assets::ResourceManager::Flags flags{};
  resourceManager_ =
      std::make_unique<assets::ResourceManager>(metadataMediator, flags);

  sceneManager_ = scene::SceneManager::create_unique();

  for (int envIdx = 0; envIdx < config_.numEnvironments; ++envIdx) {
    auto callbacks = gfx::replay::createSceneGraphPlayerCallbacks();
    callbacks.loadAndCreateRenderInstance_ =
        [this, envIdx](const assets::AssetInfo& assetInfo,
                       const assets::RenderAssetInstanceCreationInfo& creation)
        -> gfx::replay::GfxReplayNode* {
      return loadAndCreateRenderAssetInstance(envIdx, assetInfo, creation);
    };
    callbacks.changeLightSetup_ =
        [this](const gfx::LightSetup& lights) -> void {
      resourceManager_->setLightSetup(lights);
    };

    auto sceneID = sceneManager_->initSceneGraph();
    auto semanticSceneID = cfg.forceSeparateSemanticSceneGraph
                               ? sceneManager_->initSceneGraph()
                               : sceneID;

    auto& sceneGraph = sceneManager_->getSceneGraph(sceneID);
    auto& parentNode = sceneGraph.getRootNode().createChild();
    auto sensorMap = esp::sensor::SensorFactory::createSensors(
        parentNode, cfg.sensorSpecifications);

    envs_.emplace_back(
        EnvironmentRecord{.player_ = gfx::replay::Player(std::move(callbacks)),
                          .sceneID_ = sceneID,
                          .semanticSceneID_ = semanticSceneID,
                          .sensorParentNode_ = &parentNode,
                          .sensorMap_ = std::move(sensorMap)});
  }

  // OpenGL context and renderer
  {
    if (!Magnum::GL::Context::hasCurrent()) {
      context_ = gfx::WindowlessContext::create_unique(config_.gpuDeviceId);
    }

    gfx::Renderer::Flags flags;
#ifdef ESP_BUILD_WITH_BACKGROUND_RENDERER
    if (context_)
      flags |= gfx::Renderer::Flag::BackgroundRenderer;

    if (context_ && config_.leaveContextWithBackgroundRenderer)
      flags |= gfx::Renderer::Flag::LeaveContextWithBackgroundRenderer;
#else
    if (config_.numEnvironments > 1)
      ESP_DEBUG()
          << "ReplayBatchRenderer created without a background renderer. "
             "Multiple environments require a background renderer.";
#endif
    renderer_ = gfx::Renderer::create(context_.get(), flags);

    renderer_->acquireGlContext();
  }

  // Stuff?
  for (int envIndex = 0; envIndex < config_.numEnvironments; envIndex++) {
    auto& sensorMap = getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
    for (auto& kv : sensorMap) {
      CORRADE_INTERNAL_ASSERT(kv.second.get().isVisualSensor());
      auto& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(kv.second.get());
      renderer_->bindRenderTarget(visualSensor);
    }
  }
}

ReplayRenderer::~ReplayRenderer() {
  ESP_DEBUG() << "Deconstructing ReplayBatchRenderer";
  for (int envIdx = 0; envIdx < config_.numEnvironments; ++envIdx) {
    envs_[envIdx].player_.close();
    auto& sensorMap = envs_[envIdx].sensorMap_;
    for (auto& sensorPair : sensorMap) {
      sensor::SensorFactory::deleteSensor(sensorPair.second);
    }
  }
  resourceManager_.reset();
}

Mn::Vector2i ReplayRenderer::sensorSize(int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];

  CORRADE_INTERNAL_ASSERT(env.sensorMap_.size() == 1);
  return static_cast<esp::sensor::VisualSensor&>(env.sensorMap_.begin()->second.get()).framebufferSize();
}

void ReplayRenderer::setSensorTransformsFromKeyframe(
    int envIndex,
    const std::string& prefix) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
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

void ReplayRenderer::render(Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  for (int envIndex = 0; envIndex < config_.numEnvironments; envIndex++) {
    auto& sensorMap = getEnvironmentSensors(envIndex);
    for (auto& kv : sensorMap) {
      CORRADE_INTERNAL_ASSERT(kv.second.get().isVisualSensor());
      auto& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(kv.second.get());

      auto& sceneGraph = getSceneGraph(envIndex);

      // todo: investigate flags (frustum culling?)
      renderer_->enqueueAsyncDrawJob(visualSensor, sceneGraph,
                                    imageViews[envIndex],
                                    esp::gfx::RenderCamera::Flags{});
    }
  }

  renderer_->startDrawJobs();
  renderer_->waitDrawJobs();
}

esp::scene::SceneNode* ReplayRenderer::getEnvironmentSensorParentNode(
    int envIndex) const {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return env.sensorParentNode_;
}

std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
ReplayRenderer::getEnvironmentSensors(int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  return env.sensorMap_;
}

esp::scene::SceneGraph& ReplayRenderer::getSceneGraph(int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.sceneID_);
}

esp::scene::SceneGraph& ReplayRenderer::getSemanticSceneGraph(
    int envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.semanticSceneID_ == ID_UNDEFINED
                                          ? env.sceneID_
                                          : env.semanticSceneID_);
}

void ReplayRenderer::setEnvironmentKeyframe(
    int envIndex,
    const std::string& serKeyframe) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];
  env.player_.setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromString(serKeyframe));
}

gfx::replay::GfxReplayNode*
ReplayRenderer::loadAndCreateRenderAssetInstance(
    int envIndex,
    const assets::AssetInfo& assetInfo,
    const assets::RenderAssetInstanceCreationInfo& creation) {
  // Note this pattern of passing the scene manager and two scene ids to
  // resource manager. This is similar to ResourceManager::loadStage.
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  // perf todo: avoid dynamic mem alloc
  std::vector<int> tempIDs{env.sceneID_, env.semanticSceneID_};
  auto* node = resourceManager_->loadAndCreateRenderAssetInstance(
      assetInfo, creation, sceneManager_.get(), tempIDs);
  return reinterpret_cast<gfx::replay::GfxReplayNode*>(node);
}

ReplayBatchRenderer::ReplayBatchRenderer(const ReplayRendererConfiguration& cfg): renderer_{
  gfx_batch::RendererConfiguration{}
      // TODO provide X/Y count in the config, this is silly; also will blow up
      //  if too high
      .setTileSizeCount(Mn::Vector2i{static_cast<esp::sensor::CameraSensorSpec&>(*cfg.sensorSpecifications.front()).resolution}.flipped(), {1, cfg.numEnvironments}),
  gfx_batch::RendererStandaloneConfiguration{}}
{
  // TODO
  CORRADE_ASSERT(cfg.sensorSpecifications.size() == 1,
    "ReplayBatchRenderer: expecting exactly one sensor", );
  const auto& sensor = static_cast<esp::sensor::CameraSensorSpec&>(*cfg.sensorSpecifications.front());
  theOnlySensorName_ = sensor.uuid;
  theOnlySensorProjection_ = sensor.projectionMatrix();

  envs_ = Cr::Containers::Array<EnvironmentRecord>{Cr::NoInit, std::size_t(cfg.numEnvironments)};
  for(std::size_t i = 0; i != cfg.numEnvironments; ++i) {
    gfx::replay::PlayerCallbacks callbacks;
    callbacks.loadAndCreateRenderInstance_ = [this, i](
                        const assets::AssetInfo& assetInfo,
                        const assets::RenderAssetInstanceCreationInfo& creation)
        -> gfx::replay::GfxReplayNode* {
      // TODO i have no idea what these are, skip. expected 0 but it is 7!!
      // CORRADE_ASSERT(!creation.flags, "ReplayBatchRenderer: no idea what these flags are for:" << unsigned(creation.flags), {});
        // TODO and this is no_lights?!!
      // CORRADE_ASSERT(creation.lightSetupKey.empty(), "ReplayBatchRenderer: no idea what light setup key is for:" << creation.lightSetupKey, {});

      /* if no such name is known yet, add as a file */
      if(!renderer_.hasMeshHierarchy(creation.filepath)) {
        // TODO asserts might be TOO BRUTAL?
        CORRADE_INTERNAL_ASSERT_OUTPUT(renderer_.addFile(creation.filepath, gfx_batch::RendererFileFlag::Whole|gfx_batch::RendererFileFlag::GenerateMipmap));
        CORRADE_INTERNAL_ASSERT(renderer_.hasMeshHierarchy(creation.filepath));
      }

      return reinterpret_cast<gfx::replay::GfxReplayNode*>(renderer_.addMeshHierarchy(i, creation.filepath,
        /* Baking the initial scaling and coordinate frame into the
           transformation */
        // TODO why the scale has to be an optional??
        Mn::Matrix4::scaling(creation.scale ? *creation.scale : Mn::Vector3{1.0f})*
        Mn::Matrix4::from(Mn::Quaternion{assetInfo.frame.rotationFrameToWorld()}.toMatrix(), {}))
        /* Returning incremented by 1 because 0 (nullptr) is treated as an
           error */
        + 1);
    };
    callbacks.deleteAssetInstance_ = [this, i](const gfx::replay::GfxReplayNode* node) {
      // TODO actually remove from the scene instead of setting a zero scale
      renderer_.transformations(i)[reinterpret_cast<std::size_t>(node) - 1] = Mn::Matrix4{Mn::Math::ZeroInit};
    };
    callbacks.setNodeTransform_ = [this, i](const gfx::replay::GfxReplayNode* node, const Mn::Vector3& translation, const Mn::Quaternion& rotation) {
      renderer_.transformations(i)[reinterpret_cast<std::size_t>(node) - 1] = Mn::Matrix4::from(rotation.toMatrix(), translation);
    };
    callbacks.setNodeSemanticId_ = [](const gfx::replay::GfxReplayNode*, int) {
      // CORRADE_INTERNAL_ASSERT_UNREACHABLE(); // TODO
    };
    callbacks.changeLightSetup_ = [](const gfx::LightSetup&) {
      // CORRADE_INTERNAL_ASSERT_UNREACHABLE(); // TODO
    };
    // TODO arrayAppend() doesn't work because something in here is not
    //  nothrow-movable. FUN (yes the _FUN is to blame!)
    new(&envs_[i]) EnvironmentRecord{gfx::replay::Player{callbacks}};
  }
}

ReplayBatchRenderer::~ReplayBatchRenderer() = default;

Mn::Vector2i ReplayBatchRenderer::sensorSize(
  int /* all environments have the same size */
) {
  return renderer_.tileSize();
}

void ReplayBatchRenderer::setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe) {
  // TODO asserts for env index, do in base class

  // TODO why is this here at all????
  auto& env = envs_[envIndex];
  env.player_.setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromString(serKeyframe));
}

void ReplayBatchRenderer::setSensorTransformsFromKeyframe(int envIndex, const std::string& prefix) {
  // TODO asserts for env index, do in base class
  auto& env = envs_[envIndex];
  // TODO what iz diz??
  ESP_CHECK(env.player_.getNumKeyframes() == 1,
            "setSensorTransformsFromKeyframe: for environment "
                << envIndex
                << ", you have not yet called setEnvironmentKeyframe.");
  // TODO huh??
  std::string userName = prefix + theOnlySensorName_;
  Mn::Vector3 translation;
  Mn::Quaternion rotation;
  bool found =
    env.player_.getUserTransform(userName, &translation, &rotation);
  ESP_CHECK(found,
    "setSensorTransformsFromKeyframe: couldn't find user transform \""
    << userName << "\" for environment " << envIndex << ".");
  renderer_.camera(envIndex) = theOnlySensorProjection_*Mn::Matrix4::from(rotation.toMatrix(), translation).inverted();
}

void ReplayBatchRenderer::render(Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  renderer_.draw();

  // TODO ugh!! just read directly to the view
  const Mn::Image2D color = renderer_.colorImage();
  const Cr::Containers::StridedArrayView3D<const char> pixels = color.pixels();
  for(std::size_t y = 0; y != imageViews.size(); ++y) {
    /* The output might be three-component, skip alpha */
    // TODO probably rather slow, HEH
    Cr::Utility::copy(
    pixels.slice({y*renderer_.tileSize().y(), 0, 0},
                 {(y + 1)*renderer_.tileSize().y(), renderer_.tileSize().x(), imageViews[y].pixelSize()}),
      imageViews[y].pixels());
  }
}

}  // namespace sim
}  // namespace esp
