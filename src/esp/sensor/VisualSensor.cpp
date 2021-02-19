// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "esp/gfx/RenderTarget.h"

namespace esp {
namespace sensor {

VisualSensorSpec::VisualSensorSpec()
    : SensorSpec(),
      ortho_scale(0.1f),
      resolution({84, 84}),
      encoding("rgba_uint8"),
      gpu2gpuTransfer(false) {
  sensorType = SensorType::Color;
}

void VisualSensorSpec::sanityCheck() {
  bool isVisualSensor =
      (sensorType == SensorType::Color || sensorType == SensorType::Depth ||
       sensorType == SensorType::Normal || sensorType == SensorType::Semantic);
  CORRADE_ASSERT(
      isVisualSensor,
      "VisualSensor::VisualSensorSpec(): sensorType must be Color, Depth, "
      "Normal, or Semantic", );
  if (noiseModel == "Gaussian" || noiseModel == "Poisson" ||
      noiseModel == "SaltAndPepper" || noiseModel == "Speckle") {
    CORRADE_ASSERT(
        sensorType == SensorType::Color,
        "VisualSensor::VisualSensorSpec(): sensorType must be Color if "
        "noiseModel is Gaussian, Poisson, SaltAndPepper, or Speckle", );
  }
  if (noiseModel == "Redwood") {
    CORRADE_ASSERT(
        sensorType == SensorType::Depth,
        "VisualSensor::VisualSensorSpec(): sensorType must be Depth if "
        "noiseModel is Redwood", );
  }
  CORRADE_ASSERT(resolution[0] > 0 && resolution[1] > 0,
                 "VisualSensor::VisualSensorSpec(): resolution height and "
                 "width must be greater than 0", );
}

bool VisualSensorSpec::operator==(const VisualSensorSpec& a) const {
  return SensorSpec::operator==(a) && ortho_scale == a.ortho_scale &&
         resolution == a.resolution && encoding == a.encoding &&
         gpu2gpuTransfer == a.gpu2gpuTransfer;
}

VisualSensor::VisualSensor(scene::SceneNode& node, VisualSensorSpec::ptr spec)
    : Sensor{node, std::move(spec)}, tgt_{nullptr} {
  visualSensorSpec_->sanityCheck();
}

VisualSensor::~VisualSensor() = default;

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

bool VisualSensor::displayObservation(sim::Simulator& sim) {
  if (!hasRenderTarget()) {
    return false;
  }
  drawObservation(sim);
  renderTarget().blitRgbaToDefault();
  return true;
}

void VisualSensor::setResolution(int height, int width) {
  visualSensorSpec_->resolution = {height, width};
}

void VisualSensor::setResolution(vec2i resolution) {
  visualSensorSpec_->resolution = {resolution[0], resolution[1]};
}

}  // namespace sensor
}  // namespace esp
