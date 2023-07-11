// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractReplayRenderer.h"

#include "esp/gfx/replay/Player.h"

namespace esp {
namespace sim {

namespace Cr = Corrade;
namespace Mn = Magnum;

ReplayRendererConfiguration::ReplayRendererConfiguration() = default;
ReplayRendererConfiguration::ReplayRendererConfiguration(
    ReplayRendererConfiguration&&) noexcept = default;
ReplayRendererConfiguration::ReplayRendererConfiguration(
    const ReplayRendererConfiguration&) = default;
ReplayRendererConfiguration::~ReplayRendererConfiguration() = default;
ReplayRendererConfiguration& ReplayRendererConfiguration::operator=(
    const ReplayRendererConfiguration&) = default;
ReplayRendererConfiguration& ReplayRendererConfiguration::operator=(
    ReplayRendererConfiguration&&) noexcept = default;

Mn::Vector2i AbstractReplayRenderer::environmentGridSize(
    Mn::Int environmentCount) {
  const auto x =
      Mn::Int(Mn::Math::ceil(Mn::Math::sqrt(Mn::Float(environmentCount))));
  return {x, (environmentCount + x - 1) / x};
}

AbstractReplayRenderer::~AbstractReplayRenderer() = default;

void AbstractReplayRenderer::close() {
  doClose();
}

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

void AbstractReplayRenderer::clearEnvironment(unsigned envIndex) {
  CORRADE_INTERNAL_ASSERT(envIndex < doEnvironmentCount());
  // TODO a strange API name, but it does what I need
  doPlayerFor(envIndex).close();
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
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> colorImageViews,
    Cr::Containers::ArrayView<const Mn::MutableImageView2D> depthImageViews) {
  if (colorImageViews.size() > 0) {
    ESP_CHECK(colorImageViews.size() == doEnvironmentCount(),
              "ReplayRenderer::render(): expected"
                  << doEnvironmentCount() << "color image views but got"
                  << colorImageViews.size());
  }
  if (depthImageViews.size() > 0) {
    ESP_CHECK(depthImageViews.size() == doEnvironmentCount(),
              "ReplayRenderer::render(): expected"
                  << doEnvironmentCount() << "depth image views but got"
                  << depthImageViews.size());
  }
  return doRender(colorImageViews, depthImageViews);
}

void AbstractReplayRenderer::render(
    Magnum::GL::AbstractFramebuffer& framebuffer) {
  return doRender(framebuffer);
}

const void* AbstractReplayRenderer::getCudaColorBufferDevicePointer() {
  ESP_ERROR() << "CUDA device pointer only available with the batch renderer.";
  return nullptr;
}

const void* AbstractReplayRenderer::getCudaDepthBufferDevicePointer() {
  ESP_ERROR() << "CUDA device pointer only available with the batch renderer.";
  return nullptr;
}

std::shared_ptr<esp::gfx::DebugLineRender>
AbstractReplayRenderer::getDebugLineRender(unsigned envIndex) {
  ESP_CHECK(envIndex == 0, "getDebugLineRender is only available for env 0");
  // We only create this if/when used (lazy creation)
  if (!debugLineRender_) {
    debugLineRender_ = std::make_shared<esp::gfx::DebugLineRender>();
  }
  return debugLineRender_;
}

esp::geo::Ray AbstractReplayRenderer::unproject(
    unsigned envIndex,
    const Mn::Vector2i& viewportPosition) {
  checkEnvIndex(envIndex);
  return doUnproject(envIndex, viewportPosition);
}

void AbstractReplayRenderer::checkEnvIndex(unsigned envIndex) {
  ESP_CHECK(envIndex < doEnvironmentCount(),
            "envIndex " << envIndex << " is out of range");
}

}  // namespace sim
}  // namespace esp
