// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractReplayRenderer.h"

#include <Magnum/Math/Functions.h>
#include <Magnum/Math/Vector2.h>

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

void AbstractReplayRenderer::clearEnviroment(unsigned envIndex) {
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

}  // namespace sim
}  // namespace esp
