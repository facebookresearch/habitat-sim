// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayBatchRenderer.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/Renderer.h"
#include "esp/gfx/RenderTarget.h"
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

Mn::Vector2i AbstractReplayRenderer::environmentGridSize(Mn::Int environmentCount) {
  const Mn::Int x = Mn::Math::ceil(Mn::Math::sqrt(Mn::Float(environmentCount)));
  return {x, (environmentCount + x - 1)/x};
}

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
    // TODO make this more robust regarding the standalone option (assuming
    //  a context is there if standalone, assuming it's not if not)
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

void ReplayRenderer::setSensorTransform(int envIndex,
                                             const std::string& sensorName,
                                             const Mn::Matrix4& transform) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];

  ESP_CHECK(env.sensorMap_.count(sensorName),
            "ReplayBatchRenderer::setSensorTransform: sensor "
                << sensorName << " not found.");

  // note: can't use operator[] with map of reference_wrappers
  auto& thingy = env.sensorMap_.at(sensorName).get();
  auto& sensor = static_cast<sensor::VisualSensor&>(thingy);

  // auto& sensor =
  // static_cast<sensor::VisualSensor&>(env.sensorMap_[sensorName].get());
  sensor.node().setTransformation(transform);
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
  CORRADE_ASSERT(imageViews.size() == config_.numEnvironments,
    "ReplayRenderer::render(): expected" << envs_.size() << "image views but got" << imageViews.size(), );

  for (int envIndex = 0; envIndex < config_.numEnvironments; envIndex++) {
    auto& sensorMap = getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
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

void ReplayRenderer::render(Magnum::GL::AbstractFramebuffer& framebuffer) {
  const Mn::Vector2i gridSize = environmentGridSize(config_.numEnvironments);

  for (int envIndex = 0; envIndex < config_.numEnvironments; envIndex++) {


    auto& sensorMap = getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
    CORRADE_INTERNAL_ASSERT(sensorMap.begin()->second.get().isVisualSensor());
      auto& visualSensor =
          static_cast<esp::sensor::VisualSensor&>(sensorMap.begin()->second.get());

        visualSensor.renderTarget().renderEnter();

      auto& sceneGraph = getSceneGraph(envIndex);
      renderer_->draw(*visualSensor.getRenderCamera(), sceneGraph, esp::gfx::RenderCamera::Flags{});
      // for(auto&& drawables: sceneGraph.getDrawableGroups()) {
      //   visualSensor.getRenderCamera()->draw(drawables.second, esp::gfx::RenderCamera::Flags{});
      // }

      visualSensor.renderTarget().renderExit();

      // TODO ugh wait, this is calculating the size from scratch for every
      //  environment in a hope that all have the same?? UGH
      const auto size = Mn::Vector2i{visualSensor.specification()->resolution}.flipped();
      const auto rectangle = Mn::Range2Di::fromSize(size*Mn::Vector2i{envIndex% gridSize.x(), envIndex/gridSize.x()}, size);
      visualSensor.renderTarget().blitRgbaTo(framebuffer, rectangle);
  }
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
  // TODO ugh this needs to be in the base class, how to access the player there tho?
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

ReplayBatchRenderer::ReplayBatchRenderer(const ReplayRendererConfiguration& cfg) {
  // TODO
  CORRADE_ASSERT(cfg.sensorSpecifications.size() == 1,
    "ReplayBatchRenderer: expecting exactly one sensor", );
  const auto& sensor = static_cast<esp::sensor::CameraSensorSpec&>(*cfg.sensorSpecifications.front());

  gfx_batch::RendererConfiguration configuration;
  configuration
      .setTileSizeCount(Mn::Vector2i{sensor.resolution}.flipped(), environmentGridSize(cfg.numEnvironments));
  if((standalone_ = cfg.standalone))
    renderer_.emplace<gfx_batch::RendererStandalone>(
      configuration,
      gfx_batch::RendererStandaloneConfiguration{});
  else {
    CORRADE_ASSERT(Mn::GL::Context::hasCurrent(),
      "ReplayBatchRenderer: expecting a current GL context if a standalone renderer is disabled", );
    renderer_.emplace<gfx_batch::Renderer>(configuration);
  }

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
      if(!renderer_->hasMeshHierarchy(creation.filepath)) {
        // TODO asserts might be TOO BRUTAL?
        CORRADE_INTERNAL_ASSERT_OUTPUT(renderer_->addFile(creation.filepath, gfx_batch::RendererFileFlag::Whole|gfx_batch::RendererFileFlag::GenerateMipmap));
        CORRADE_INTERNAL_ASSERT(renderer_->hasMeshHierarchy(creation.filepath));
      }

      return reinterpret_cast<gfx::replay::GfxReplayNode*>(renderer_->addMeshHierarchy(i, creation.filepath,
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
      renderer_->transformations(i)[reinterpret_cast<std::size_t>(node) - 1] = Mn::Matrix4{Mn::Math::ZeroInit};
    };
    callbacks.setNodeTransform_ = [this, i](const gfx::replay::GfxReplayNode* node, const Mn::Vector3& translation, const Mn::Quaternion& rotation) {
      renderer_->transformations(i)[reinterpret_cast<std::size_t>(node) - 1] = Mn::Matrix4::from(rotation.toMatrix(), translation);
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
  return renderer_->tileSize();
}

void ReplayBatchRenderer::setEnvironmentKeyframe(int envIndex, const std::string& serKeyframe) {
  // TODO asserts for env index, do in base class

  // TODO why is this here at all???? needs to be in the base class, which should have access to the player somehow
  auto& env = envs_[envIndex];
  env.player_.setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromString(serKeyframe));
}

void ReplayBatchRenderer::setSensorTransform(int envIndex,
                          // TODO assumes there's just one sensor per env
                          const std::string&,
                          const Mn::Matrix4& transform) {
  renderer_->camera(envIndex) = theOnlySensorProjection_*transform.inverted();
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
  renderer_->camera(envIndex) = theOnlySensorProjection_*Mn::Matrix4::from(rotation.toMatrix(), translation).inverted();
}

void ReplayBatchRenderer::render(Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  CORRADE_ASSERT(standalone_,
    "ReplayBatchRenderer::render(): can use this function only with a standalone renderer", );
  CORRADE_ASSERT(imageViews.size() == envs_.size(),
    "ReplayBatchRenderer::render(): expected" << envs_.size() << "image views but got" << imageViews.size(), );
  static_cast<gfx_batch::RendererStandalone&>(*renderer_).draw();

  for(int envIndex = 0; envIndex != envs_.size(); ++envIndex) {
    const auto rectangle = Mn::Range2Di::fromSize(renderer_->tileSize()*Mn::Vector2i{envIndex%renderer_->tileCount().x(), envIndex/renderer_->tileCount().x()}, renderer_->tileSize());
    static_cast<gfx_batch::RendererStandalone&>(*renderer_).colorImageInto(rectangle, imageViews[envIndex]);
  }
}

void ReplayBatchRenderer::render(Magnum::GL::AbstractFramebuffer& framebuffer) {
  CORRADE_ASSERT(!standalone_,
    "ReplayBatchRenderer::render(): can't use this function with a standalone renderer", );

  /* The non-standalone renderer doesn't clear on its own, it's the user
     responsibility */
  framebuffer.clear(Mn::GL::FramebufferClear::Color |
                            Mn::GL::FramebufferClear::Depth);

  renderer_->draw(framebuffer);
}

}  // namespace sim
}  // namespace esp
