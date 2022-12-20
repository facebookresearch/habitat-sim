// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchReplayRenderer.h"

#include "esp/sensor/CameraSensor.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Corrade/Utility/Algorithms.h>
#include <Magnum/GL/Context.h>
#include <Magnum/Image.h>
#include <Magnum/ImageView.h>

namespace esp {
namespace sim {

// clang-tidy you're NOT HELPING
using namespace Mn::Math::Literals;  // NOLINT

BatchReplayRenderer::BatchReplayRenderer(
    const ReplayRendererConfiguration& cfg,
    gfx_batch::RendererConfiguration&& batchRendererConfiguration) {
  if (Magnum::GL::Context::hasCurrent()) {
    flextGLInit(Magnum::GL::Context::current());  // TODO: Avoid globals
                                                  // duplications across SOs.
  }
  CORRADE_ASSERT(cfg.sensorSpecifications.size() == 1,
                 "BatchReplayRenderer: expecting exactly one sensor", );
  const auto& sensor = static_cast<esp::sensor::CameraSensorSpec&>(
      *cfg.sensorSpecifications.front());

  batchRendererConfiguration.setTileSizeCount(
      Mn::Vector2i{sensor.resolution}.flipped(),
      environmentGridSize(cfg.numEnvironments));
  if ((standalone_ = cfg.standalone))
    renderer_.emplace<gfx_batch::RendererStandalone>(
        batchRendererConfiguration,
        gfx_batch::RendererStandaloneConfiguration{});
  else {
    CORRADE_ASSERT(Mn::GL::Context::hasCurrent(),
                   "BatchReplayRenderer: expecting a current GL context if a "
                   "standalone renderer is disabled", );
    renderer_.emplace<gfx_batch::Renderer>(batchRendererConfiguration);
  }

  theOnlySensorName_ = sensor.uuid;
  theOnlySensorProjection_ = sensor.projectionMatrix();

  class BatchPlayerImplementation
      : public gfx::replay::AbstractPlayerImplementation {
   public:
    BatchPlayerImplementation(gfx_batch::Renderer& renderer,
                              Mn::UnsignedInt sceneId)
        : renderer_{renderer}, sceneId_{sceneId} {}

   private:
    gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
        const esp::assets::AssetInfo& assetInfo,
        const esp::assets::RenderAssetInstanceCreationInfo& creation) override {
      // TODO anything to use creation.flags for?
      // TODO is creation.lightSetupKey actually mapping to anything in the
      //  replay file?

      /* If no such name is known yet, add as a file */
      if (!renderer_.hasNodeHierarchy(creation.filepath)) {
        ESP_WARNING()
            << creation.filepath
            << "not found in any composite file, loading from the filesystem";
        // TODO asserts might be TOO BRUTAL?
        CORRADE_INTERNAL_ASSERT_OUTPUT(
            renderer_.addFile(creation.filepath,
                              gfx_batch::RendererFileFlag::Whole |
                                  gfx_batch::RendererFileFlag::GenerateMipmap));
        CORRADE_INTERNAL_ASSERT(renderer_.hasNodeHierarchy(creation.filepath));
      }

      return reinterpret_cast<gfx::replay::NodeHandle>(
          renderer_.addNodeHierarchy(
              sceneId_, creation.filepath,
              /* Baking the initial scaling and coordinate frame into the
                 transformation */
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

    void deleteAssetInstances(
        const std::unordered_map<gfx::replay::RenderAssetInstanceKey,
                                 gfx::replay::NodeHandle>&) override {
      renderer_.clear(sceneId_);
    }

    void setNodeTransform(const gfx::replay::NodeHandle node,
                          const Mn::Vector3& translation,
                          const Mn::Quaternion& rotation) override {
      renderer_.transformations(
          sceneId_)[reinterpret_cast<std::size_t>(node) - 1] =
          Mn::Matrix4::from(rotation.toMatrix(), translation);
    }

    void changeLightSetup(const esp::gfx::LightSetup& lights) override {
      if (!renderer_.maxLightCount()) {
        ESP_WARNING() << "Attempted to change" << lights.size()
                      << "lights for scene" << sceneId_
                      << "but the renderer is configured without lights";
        return;
      }

      renderer_.clearLights(sceneId_);
      for (std::size_t i = 0; i != lights.size(); ++i) {
        const gfx::LightInfo& light = lights[i];
        CORRADE_INTERNAL_ASSERT(light.model == gfx::LightPositionModel::Global);

        const std::size_t nodeId = renderer_.addEmptyNode(sceneId_);

        /* Clang Tidy, you're stupid, why do you say that "lightId" is not
           initialized?! I'm initializing it right in the branches below, I
           won't zero-init it just to "prevent bugs" because an accidentally
           zero-initialized variable is *also* a bug, you know? Plus I have
           range asserts in all functions so your "suggestions" are completely
           unhelpful. */
        std::size_t lightId;  // NOLINT
        if (light.vector.z()) {
          renderer_.transformations(sceneId_)[nodeId] =
              Mn::Matrix4::translation(light.vector.xyz());
          lightId = renderer_.addLight(sceneId_, nodeId,
                                       gfx_batch::RendererLightType::Point);
        } else {
          /* The matrix will be partially NaNs if the light vector is in the
             direction of the Y axis, but that's fine -- we only really care
             about the Z axis direction, which is always the "target" vector
             normalized. */
          renderer_.transformations(sceneId_)[nodeId] =
              Mn::Matrix4::lookAt({}, light.vector.xyz(), Mn::Vector3::yAxis());
          lightId = renderer_.addLight(
              sceneId_, nodeId, gfx_batch::RendererLightType::Directional);
        }

        renderer_.lightColors(sceneId_)[lightId] = light.color;
        // TODO range, once Habitat has that
      }
    }

    gfx_batch::Renderer& renderer_;
    Mn::UnsignedInt sceneId_;
  };

  for (Mn::UnsignedInt i = 0; i != cfg.numEnvironments; ++i) {
    arrayAppend(
        envs_, EnvironmentRecord{
                   std::make_shared<BatchPlayerImplementation>(*renderer_, i)});
  }
}

BatchReplayRenderer::~BatchReplayRenderer() = default;

void BatchReplayRenderer::doPreloadFile(Cr::Containers::StringView filename) {
  CORRADE_INTERNAL_ASSERT(renderer_->addFile(filename));
}

unsigned BatchReplayRenderer::doEnvironmentCount() const {
  return envs_.size();
}

Mn::Vector2i BatchReplayRenderer::doSensorSize(
    unsigned /* all environments have the same size */
) {
  return renderer_->tileSize();
}

gfx::replay::Player& BatchReplayRenderer::doPlayerFor(unsigned envIndex) {
  return envs_[envIndex].player_;
}

void BatchReplayRenderer::doSetSensorTransform(
    unsigned envIndex,
    // TODO assumes there's just one sensor per env
    const std::string&,
    const Mn::Matrix4& transform) {
  renderer_->camera(envIndex) = theOnlySensorProjection_ * transform.inverted();
}

void BatchReplayRenderer::doSetSensorTransformsFromKeyframe(
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

void BatchReplayRenderer::doRender(
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> imageViews) {
  CORRADE_ASSERT(standalone_,
                 "BatchReplayRenderer::render(): can use this function only "
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

void BatchReplayRenderer::doRender(
    Magnum::GL::AbstractFramebuffer& framebuffer) {
  CORRADE_ASSERT(!standalone_,
                 "BatchReplayRenderer::render(): can't use this function with "
                 "a standalone renderer", );

  renderer_->draw(framebuffer);
}

}  // namespace sim
}  // namespace esp
