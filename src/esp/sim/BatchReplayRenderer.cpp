// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchReplayRenderer.h"

#include <esp/gfx_batch/DepthUnprojection.h>
#include "esp/sensor/CameraSensor.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/GL/AbstractFramebuffer.h>
#include <Magnum/GL/Context.h>
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
    bool isSupportedRenderAsset(
        const Corrade::Containers::StringView& filepath) {
      // Primitives aren't directly supported in the Magnum batch renderer. See
      // https://docs.google.com/document/d/1ngA73cXl3YRaPfFyICSUHONZN44C-XvieS7kwyQDbkI/edit#bookmark=id.yq39718gqbwz

      const std::array<Corrade::Containers::StringView, 12> primNamePrefixes = {
          "capsule3DSolid",     "capsule3DWireframe", "coneSolid",
          "coneWireframe",      "cubeSolid",          "cubeWireframe",
          "cylinderSolid",      "cylinderWireframe",  "icosphereSolid",
          "icosphereWireframe", "uvSphereSolid",      "uvSphereWireframe"};

      // primitive render asset filepaths start with one of the above prefixes.
      // Examples: icosphereSolid_subdivs_1
      // capsule3DSolid_hemiRings_4_cylRings_1_segments_12_halfLen_3.25_useTexCoords_false_useTangents_false
      for (const auto& primNamePrefix : primNamePrefixes) {
        if (filepath.size() < primNamePrefix.size()) {
          continue;
        }

        if (filepath.prefix(primNamePrefix.size()) == primNamePrefix) {
          return false;
        }
      }

      return true;
    }

    gfx::replay::NodeHandle loadAndCreateRenderAssetInstance(
        const esp::assets::AssetInfo& assetInfo,
        const esp::assets::RenderAssetInstanceCreationInfo& creation) override {
      // TODO anything to use creation.flags for?
      // TODO is creation.lightSetupKey actually mapping to anything in the
      //  replay file?

      if (!isSupportedRenderAsset(creation.filepath)) {
        ESP_WARNING() << "Unsupported render asset: " << creation.filepath;
        return nullptr;
      }

      /* If no such name is known yet, add as a file */
      if (!renderer_.hasNodeHierarchy(creation.filepath)) {
        ESP_WARNING()
            << creation.filepath
            << "not found in any composite file, loading from the filesystem";

        ESP_CHECK(
            renderer_.addFile(creation.filepath,
                              gfx_batch::RendererFileFlag::Whole |
                                  gfx_batch::RendererFileFlag::GenerateMipmap),
            "addFile failed for " << creation.filepath);
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

    void setNodeTransform(const gfx::replay::NodeHandle node,
                          const Mn::Matrix4& transform) override {
      renderer_.transformations(
          sceneId_)[reinterpret_cast<std::size_t>(node) - 1] = transform;
    }

    const Mn::Matrix4 getNodeTransform(
        const gfx::replay::NodeHandle node) const override {
      return renderer_.transformations(
          sceneId_)[reinterpret_cast<std::size_t>(node) - 1];
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
        if (light.vector.w()) {
          renderer_.transformations(sceneId_)[nodeId] =
              Mn::Matrix4::translation(light.vector.xyz());
          lightId = renderer_.addLight(sceneId_, nodeId,
                                       gfx_batch::RendererLightType::Point);
        } else {
          /* The matrix will be partially NaNs if the light vector is in the
             direction of the Y axis, but that's fine -- we only really care
             about the Z axis direction, which is always the "target" vector
             normalized. */
          // TODO for more robustness use something that "invents" some
          //  arbitrary orthogonal axes instead of the NaNs, once Magnum has
          //  such utility
          renderer_.transformations(sceneId_)[nodeId] =
              Mn::Matrix4::lookAt({}, light.vector.xyz(), Mn::Vector3::yAxis());
          lightId = renderer_.addLight(
              sceneId_, nodeId, gfx_batch::RendererLightType::Directional);
        }

        renderer_.lightColors(sceneId_)[lightId] = light.color;
        // TODO use gfx::getAmbientLightColor(lights) once it's not hardcoded
        //  to an arbitrary value and once it's possible to change the ambient
        //  factor in the renderer at runtime (and not just in
        //  RendererConfiguration::setAmbientFactor())
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
  renderer_->updateCamera(envIndex, theOnlySensorProjection_,
                          transform.inverted());
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
  renderer_->updateCamera(
      envIndex, theOnlySensorProjection_,
      Mn::Matrix4::from(rotation.toMatrix(), translation).inverted());
}

void BatchReplayRenderer::doRender(
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> colorImageViews,
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> depthImageViews) {
  CORRADE_ASSERT(standalone_,
                 "BatchReplayRenderer::render(): can use this function only "
                 "with a standalone renderer", );
  auto& standalone = static_cast<gfx_batch::RendererStandalone&>(*renderer_);
  standalone.draw();

  // todo: integrate debugLineRender_->flushLines
  CORRADE_INTERNAL_ASSERT(!debugLineRender_);

  for (int envIndex = 0; envIndex != envs_.size(); ++envIndex) {
    const auto rectangle = Mn::Range2Di::fromSize(
        renderer_->tileSize() *
            Mn::Vector2i{envIndex % renderer_->tileCount().x(),
                         envIndex / renderer_->tileCount().x()},
        renderer_->tileSize());

    if (colorImageViews.size() > 0) {
      standalone.colorImageInto(rectangle, colorImageViews[envIndex]);
    }
    if (depthImageViews.size() > 0) {
      Mn::MutableImageView2D depthBufferView{
          standalone.depthFramebufferFormat(), depthImageViews[envIndex].size(),
          depthImageViews[envIndex].data()};
      standalone.depthImageInto(rectangle, depthBufferView);

      // TODO: Add GPU depth unprojection support.
      gfx_batch::unprojectDepth(renderer_->cameraDepthUnprojection(envIndex),
                                depthBufferView.pixels<Mn::Float>());
    }
  }
}

void BatchReplayRenderer::doRender(
    Magnum::GL::AbstractFramebuffer& framebuffer) {
  CORRADE_ASSERT(!standalone_,
                 "BatchReplayRenderer::render(): can't use this function with "
                 "a standalone renderer", );

  renderer_->draw(framebuffer);

  if (debugLineRender_) {
    framebuffer.bind();
    constexpr unsigned envIndex = 0;
    auto projCamMatrix = renderer_->camera(envIndex);
    debugLineRender_->flushLines(projCamMatrix, renderer_->tileSize());
  }
}

esp::geo::Ray BatchReplayRenderer::doUnproject(
    CORRADE_UNUSED unsigned envIndex,
    const Mn::Vector2i& viewportPosition) {
  // temp stub implementation: produce a placeholder ray that varies with
  // viewportPosition
  return esp::geo::Ray(
      {static_cast<float>(viewportPosition.x()) / renderer_->tileSize().x(),
       0.5f,
       static_cast<float>(viewportPosition.y()) / renderer_->tileSize().y()},
      {0.f, -1.f, 0.f});
}

const void* BatchReplayRenderer::getCudaColorBufferDevicePointer() {
#ifdef ESP_BUILD_WITH_CUDA
  CORRADE_ASSERT(standalone_,
                 "ReplayBatchRenderer::colorCudaBufferDevicePointer(): can use "
                 "this function only "
                 "with a standalone renderer",
                 nullptr);
  return static_cast<gfx_batch::RendererStandalone&>(*renderer_)
      .colorCudaBufferDevicePointer();
#else
  ESP_ERROR() << "Failed to retrieve device pointer because CUDA is not "
                 "available in this build.";
  return nullptr;
#endif
}

const void* BatchReplayRenderer::getCudaDepthBufferDevicePointer() {
#ifdef ESP_BUILD_WITH_CUDA
  CORRADE_ASSERT(standalone_,
                 "ReplayBatchRenderer::getCudaDepthBufferDevicePointer(): can "
                 "use this function only "
                 "with a standalone renderer",
                 nullptr);
  return static_cast<gfx_batch::RendererStandalone&>(*renderer_)
      .depthCudaBufferDevicePointer();
#else
  ESP_ERROR() << "Failed to retrieve device pointer because CUDA is not "
                 "available in this build.";
  return nullptr;
#endif
}
}  // namespace sim
}  // namespace esp
