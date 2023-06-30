// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BatchReplayRenderer.h"

#include <esp/gfx_batch/DepthUnprojection.h>
#include <esp/sim/BatchPlayerImplementation.h>
#include "esp/sensor/CameraSensor.h"

#include <Corrade/Containers/GrowableArray.h>
#include <Magnum/GL/AbstractFramebuffer.h>
#include <Magnum/GL/Context.h>
#include <Magnum/ImageView.h>

namespace esp {
namespace sim {

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

  for (Mn::UnsignedInt i = 0; i != cfg.numEnvironments; ++i) {
    arrayAppend(
        envs_, EnvironmentRecord{
                   std::make_shared<BatchPlayerImplementation>(*renderer_, i)});
  }
}

BatchReplayRenderer::~BatchReplayRenderer() {
  doCloseImpl();
}

void BatchReplayRenderer::doClose() {
  doCloseImpl();
}

void BatchReplayRenderer::doCloseImpl() {
  for (int i = 0; i < envs_.size(); ++i) {
    envs_[i].player_.close();
  }
  envs_ = {};
  renderer_.reset();
}

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
