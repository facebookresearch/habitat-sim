// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ReplayBatchRenderer.h"

#include "esp/assets/ResourceManager.h"
#include "esp/gfx/RenderTarget.h"
#include "esp/gfx/Renderer.h"
#include "esp/metadata/MetadataMediator.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/SensorFactory.h"
#include "esp/sim/SimulatorConfiguration.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Algorithms.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>

namespace esp {
namespace sim {

using namespace Mn::Math::Literals;

Mn::Vector2i AbstractReplayRenderer::environmentGridSize(
    Mn::Int environmentCount) {
  const Mn::Int x = Mn::Math::ceil(Mn::Math::sqrt(Mn::Float(environmentCount)));
  return {x, (environmentCount + x - 1) / x};
}

AbstractReplayRenderer::~AbstractReplayRenderer() = default;

void AbstractReplayRenderer::preloadFile(Cr::Containers::StringView filename) {
  doPreloadFile(filename);
}

void AbstractReplayRenderer::doPreloadFile(Cr::Containers::StringView) {}

unsigned AbstractReplayRenderer::environmentCount() const {
  return doEnvironmentCount();
}

Mn::Vector2i AbstractReplayRenderer::sensorSize(unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  return doSensorSize(envIndex);
}

void AbstractReplayRenderer::setEnvironmentKeyframe(
    unsigned envIndex,
    const std::string& serKeyframe) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  doPlayerFor(envIndex).setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromString(serKeyframe));
}

void AbstractReplayRenderer::setEnvironmentKeyframeUnwrapped(
    unsigned envIndex,
    const Cr::Containers::StringView serKeyframe) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  doPlayerFor(envIndex).setSingleKeyframe(
      esp::gfx::replay::Player::keyframeFromStringUnwrapped(serKeyframe));
}

void AbstractReplayRenderer::setSensorTransform(unsigned envIndex,
                                                const std::string& sensorName,
                                                const Mn::Matrix4& transform) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  return doSetSensorTransform(envIndex, sensorName, transform);
}

void AbstractReplayRenderer::setSensorTransformsFromKeyframe(
    unsigned envIndex,
    const std::string& prefix) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  ESP_CHECK(doPlayerFor(envIndex).getNumKeyframes() == 1,
            "setSensorTransformsFromKeyframe: for environment "
                << envIndex
                << ", you have not yet called setEnvironmentKeyframe.");
  return doSetSensorTransformsFromKeyframe(envIndex, prefix);
}

void AbstractReplayRenderer::render(
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  CORRADE_ASSERT(imageViews.size() == doEnvironmentCount(),
                 "ReplayRenderer::render(): expected" << doEnvironmentCount()
                                                      << "image views but got"
                                                      << imageViews.size(), );
  return doRender(imageViews);
}

void AbstractReplayRenderer::render(
    Magnum::GL::AbstractFramebuffer& framebuffer) {
  return doRender(framebuffer);
}

ReplayRenderer::ReplayRenderer(const ReplayRendererConfiguration& cfg) {
  config_ = cfg;
  SimulatorConfiguration simConfig;
  simConfig.createRenderer = true;
  auto metadataMediator = metadata::MetadataMediator::create(simConfig);
  assets::ResourceManager::Flags flags{};
  resourceManager_ =
      std::make_unique<assets::ResourceManager>(metadataMediator, flags);

  sceneManager_ = scene::SceneManager::create_unique();

  class SceneGraphPlayerImplementation
      : public gfx::replay::AbstractSceneGraphPlayerImplementation {
   public:
    SceneGraphPlayerImplementation(ReplayRenderer& self, unsigned envIdx)
        : self_{self}, envIdx_{envIdx} {}

   private:
    gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
        const esp::assets::AssetInfo& assetInfo,
        const esp::assets::RenderAssetInstanceCreationInfo& creation) override {
      return self_.loadAndCreateRenderAssetInstance(envIdx_, assetInfo,
                                                    creation);
    }
    void changeLightSetup(const gfx::LightSetup& lights) override {
      return self_.resourceManager_->setLightSetup(lights);
    }

    ReplayRenderer& self_;
    unsigned envIdx_;
  };

  for (int envIdx = 0; envIdx < config_.numEnvironments; ++envIdx) {
    auto sceneID = sceneManager_->initSceneGraph();
    auto semanticSceneID = cfg.forceSeparateSemanticSceneGraph
                               ? sceneManager_->initSceneGraph()
                               : sceneID;

    auto& sceneGraph = sceneManager_->getSceneGraph(sceneID);
    auto& parentNode = sceneGraph.getRootNode().createChild();
    auto sensorMap = esp::sensor::SensorFactory::createSensors(
        parentNode, cfg.sensorSpecifications);

    envs_.emplace_back(EnvironmentRecord{
        .playerImplementation_ =
            std::make_unique<SceneGraphPlayerImplementation>(*this, envIdx),
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

unsigned ReplayRenderer::doEnvironmentCount() const {
  return envs_.size();
}

Mn::Vector2i ReplayRenderer::doSensorSize(unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex >= 0 && envIndex < envs_.size());
  auto& env = envs_[envIndex];

  CORRADE_INTERNAL_ASSERT(env.sensorMap_.size() == 1);
  return static_cast<esp::sensor::VisualSensor&>(
             env.sensorMap_.begin()->second.get())
      .framebufferSize();
}

gfx::replay::Player& ReplayRenderer::doPlayerFor(unsigned envIndex) {
  return envs_[envIndex].player_;
}

void ReplayRenderer::doSetSensorTransform(unsigned envIndex,
                                          const std::string& sensorName,
                                          const Mn::Matrix4& transform) {
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

void ReplayRenderer::doSetSensorTransformsFromKeyframe(
    unsigned envIndex,
    const std::string& prefix) {
  const auto& env = envs_[envIndex];
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

void ReplayRenderer::doRender(
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
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

void ReplayRenderer::doRender(Magnum::GL::AbstractFramebuffer& framebuffer) {
  const Mn::Vector2i gridSize = environmentGridSize(config_.numEnvironments);

  for (int envIndex = 0; envIndex < config_.numEnvironments; envIndex++) {
    auto& sensorMap = getEnvironmentSensors(envIndex);
    CORRADE_INTERNAL_ASSERT(sensorMap.size() == 1);
    CORRADE_INTERNAL_ASSERT(sensorMap.begin()->second.get().isVisualSensor());
    auto& visualSensor = static_cast<esp::sensor::VisualSensor&>(
        sensorMap.begin()->second.get());

    visualSensor.renderTarget().renderEnter();

    auto& sceneGraph = getSceneGraph(envIndex);
    renderer_->draw(*visualSensor.getRenderCamera(), sceneGraph,
                    esp::gfx::RenderCamera::Flags{});

    visualSensor.renderTarget().renderExit();

    // TODO ugh wait, this is calculating the size from scratch for every
    //  environment in a hope that all have the same?? UGH
    const auto size =
        Mn::Vector2i{visualSensor.specification()->resolution}.flipped();
    const auto rectangle = Mn::Range2Di::fromSize(
        size * Mn::Vector2i{envIndex % gridSize.x(), envIndex / gridSize.x()},
        size);
    visualSensor.renderTarget().blitRgbaTo(framebuffer, rectangle);
  }
}

esp::scene::SceneNode* ReplayRenderer::getEnvironmentSensorParentNode(
    unsigned envIndex) const {
  CORRADE_INTERNAL_ASSERT(envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return env.sensorParentNode_;
}

std::map<std::string, std::reference_wrapper<esp::sensor::Sensor>>&
ReplayRenderer::getEnvironmentSensors(unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex < envs_.size());
  auto& env = envs_[envIndex];
  return env.sensorMap_;
}

esp::scene::SceneGraph& ReplayRenderer::getSceneGraph(unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.sceneID_);
}

esp::scene::SceneGraph& ReplayRenderer::getSemanticSceneGraph(
    unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex < envs_.size());
  const auto& env = envs_[envIndex];
  return sceneManager_->getSceneGraph(env.semanticSceneID_ == ID_UNDEFINED
                                          ? env.sceneID_
                                          : env.semanticSceneID_);
}

gfx::replay::NodeHandle ReplayRenderer::loadAndCreateRenderAssetInstance(
    unsigned envIndex,
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
  return reinterpret_cast<gfx::replay::NodeHandle>(node);
}

ReplayBatchRenderer::ReplayBatchRenderer(
    const ReplayRendererConfiguration& cfg) {
  CORRADE_ASSERT(cfg.sensorSpecifications.size() == 1,
                 "ReplayBatchRenderer: expecting exactly one sensor", );
  const auto& sensor = static_cast<esp::sensor::CameraSensorSpec&>(
      *cfg.sensorSpecifications.front());

  gfx_batch::RendererConfiguration configuration;
  configuration.setTileSizeCount(Mn::Vector2i{sensor.resolution}.flipped(),
                                 environmentGridSize(cfg.numEnvironments));
  if ((standalone_ = cfg.standalone))
    renderer_.emplace<gfx_batch::RendererStandalone>(
        configuration, gfx_batch::RendererStandaloneConfiguration{});
  else {
    CORRADE_ASSERT(Mn::GL::Context::hasCurrent(),
                   "ReplayBatchRenderer: expecting a current GL context if a "
                   "standalone renderer is disabled", );
    renderer_.emplace<gfx_batch::Renderer>(configuration);
  }

  theOnlySensorName_ = sensor.uuid;
  theOnlySensorProjection_ = sensor.projectionMatrix();

  class BatchPlayerImplementation
      : public gfx::replay::AbstractSceneGraphPlayerImplementation {
   public:
    BatchPlayerImplementation(gfx_batch::Renderer& renderer,
                              Mn::UnsignedInt sceneId)
        : renderer_{renderer}, sceneId_{sceneId} {}

   private:
    gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
        const esp::assets::AssetInfo& assetInfo,
        const esp::assets::RenderAssetInstanceCreationInfo& creation) override {
      // TODO i have no idea what these are, skip. expected 0 but it is 7!!
      // CORRADE_ASSERT(!creation.flags, "ReplayBatchRenderer: no idea what
      // these flags are for:" << unsigned(creation.flags), {});
      // TODO and this is no_lights?!!
      // CORRADE_ASSERT(creation.lightSetupKey.empty(), "ReplayBatchRenderer: no
      // idea what light setup key is for:" << creation.lightSetupKey, {});

      /* If no such name is known yet, add as a file */
      if (!renderer_.hasMeshHierarchy(creation.filepath)) {
        Mn::Warning{} << creation.filepath << "not found in any composite file, loading from the filesystem";
        // TODO asserts might be TOO BRUTAL?
        CORRADE_INTERNAL_ASSERT_OUTPUT(
            renderer_.addFile(creation.filepath,
                              gfx_batch::RendererFileFlag::Whole |
                                  gfx_batch::RendererFileFlag::GenerateMipmap));
        CORRADE_INTERNAL_ASSERT(renderer_.hasMeshHierarchy(creation.filepath));
      }

      return reinterpret_cast<gfx::replay::NodeHandle>(
          renderer_.addMeshHierarchy(
              sceneId_, creation.filepath,
              /* Baking the initial scaling and coordinate frame into the
                 transformation */
              // TODO why the scale has to be an optional??
              Mn::Matrix4::scaling(creation.scale ? *creation.scale
                                                  : Mn::Vector3{1.0f}) *
                  Mn::Matrix4::from(
                      Mn::Quaternion{assetInfo.frame.rotationFrameToWorld()}
                          .toMatrix(),
                      {}))
          /* Returning incremented by 1 because 0 (nullptr) is treated as an
             error */
          + 1);
    }

    void deleteAssetInstance(const gfx::replay::NodeHandle node) override {
      // TODO actually remove from the scene instead of setting a zero scale
      renderer_.transformations(
          sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
          Mn::Matrix4{Mn::Math::ZeroInit};
    }

    void setNodeTransform(const gfx::replay::NodeHandle node,
                          const Mn::Vector3& translation,
                          const Mn::Quaternion& rotation) override {
      renderer_.transformations(
          sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
          Mn::Matrix4::from(rotation.toMatrix(), translation);
    }

    void setNodeSemanticId(esp::gfx::replay::NodeHandle,
                           Mn::UnsignedInt) override {
      // CORRADE_INTERNAL_ASSERT_UNREACHABLE(); // TODO
    }

    void changeLightSetup(const gfx::LightSetup&) override {
      // CORRADE_INTERNAL_ASSERT_UNREACHABLE(); // TODO
    }

    gfx_batch::Renderer& renderer_;
    Mn::UnsignedInt sceneId_;
  };

  // envs_ = Cr::Containers::Array<EnvironmentRecord>{Cr::NoInit,
  // std::size_t(cfg.numEnvironments)};
  for (Mn::UnsignedInt i = 0; i != cfg.numEnvironments; ++i) {
    // TODO arrayAppend() doesn't work because something in Player is not not
    arrayAppend(
        envs_,
        EnvironmentRecord{
            Cr::Containers::Pointer<gfx::replay::AbstractPlayerImplementation>{
                new BatchPlayerImplementation{*renderer_, i}}});
  }
}

ReplayBatchRenderer::~ReplayBatchRenderer() = default;

void ReplayBatchRenderer::doPreloadFile(Cr::Containers::StringView filename) {
  CORRADE_INTERNAL_ASSERT(renderer_->addFile(filename));
}

unsigned ReplayBatchRenderer::doEnvironmentCount() const {
  return envs_.size();
}

Mn::Vector2i ReplayBatchRenderer::doSensorSize(
    unsigned /* all environments have the same size */
) {
  return renderer_->tileSize();
}

gfx::replay::Player& ReplayBatchRenderer::doPlayerFor(unsigned envIndex) {
  return envs_[envIndex].player_;
}

void ReplayBatchRenderer::doSetSensorTransform(
    unsigned envIndex,
    // TODO assumes there's just one sensor per env
    const std::string&,
    const Mn::Matrix4& transform) {
  renderer_->camera(envIndex) = theOnlySensorProjection_ * transform.inverted();
}

void ReplayBatchRenderer::doSetSensorTransformsFromKeyframe(
    unsigned envIndex,
    const std::string& prefix) {
  auto& env = envs_[envIndex];
  std::string userName = prefix + theOnlySensorName_;
  Mn::Vector3 translation;
  Mn::Quaternion rotation;
  bool found = env.player_.getUserTransform(userName, &translation, &rotation);
  ESP_CHECK(found,
            "setSensorTransformsFromKeyframe: couldn't find user transform \""
                << userName << "\" for environment " << envIndex << ".");
  renderer_->camera(envIndex) =
      theOnlySensorProjection_ *
      Mn::Matrix4::from(rotation.toMatrix(), translation).inverted();
}

void ReplayBatchRenderer::doRender(
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  CORRADE_ASSERT(standalone_,
                 "ReplayBatchRenderer::render(): can use this function only "
                 "with a standalone renderer", );
  static_cast<gfx_batch::RendererStandalone&>(*renderer_).draw();

  for (int envIndex = 0; envIndex != envs_.size(); ++envIndex) {
    const auto rectangle = Mn::Range2Di::fromSize(
        renderer_->tileSize() *
            Mn::Vector2i{envIndex % renderer_->tileCount().x(),
                         envIndex / renderer_->tileCount().x()},
        renderer_->tileSize());
    static_cast<gfx_batch::RendererStandalone&>(*renderer_)
        .colorImageInto(rectangle, imageViews[envIndex]);
  }
}

void ReplayBatchRenderer::doRender(
    Magnum::GL::AbstractFramebuffer& framebuffer) {
  CORRADE_ASSERT(!standalone_,
                 "ReplayBatchRenderer::render(): can't use this function with "
                 "a standalone renderer", );

  /* The non-standalone renderer doesn't clear on its own, it's the user
     responsibility */
  framebuffer.clear(Mn::GL::FramebufferClear::Color |
                    Mn::GL::FramebufferClear::Depth);

  renderer_->draw(framebuffer);
}

}  // namespace sim
}  // namespace esp
