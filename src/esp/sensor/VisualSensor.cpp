// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "esp/gfx/RenderTarget.h"

namespace esp {
namespace sensor {

void VisualSensorSpec::sanityCheck() {
  bool isVisualSensor =
      (sensorType == SensorType::Color || sensorType == SensorType::Depth ||
       sensorType == SensorType::Normal || sensorType == SensorType::Semantic);
  CORRADE_ASSERT(
      isVisualSensor,
      "VisualSensor::VisualSensorSpec() sensorType must be Color, Depth, "
      "Normal, or Semantic", );
  if (noiseModel == "Gaussian" || noiseModel == "Poisson" ||
      noiseModel == "SaltAndPepper" || noiseModel == "Speckle") {
    CORRADE_ASSERT(
        sensorType == SensorType::Color,
        "VisualSensor::VisualSensorSpec() sensorType must be Color if "
        "noiseModel is Gaussian, Poisson, SaltAndPepper, or Speckle", );
  }
  if (noiseModel == "Redwood") {
    CORRADE_ASSERT(
        sensorType == SensorType::Depth,
        "VisualSensor::VisualSensorSpec() sensorType must be Depth if "
        "noiseModel is Redwood", );
  }
}

VisualSensorSpec::VisualSensorSpec()
    : SensorSpec(),
      hfov(Mn::Deg{90.f}),
      ortho_scale(0.1f),
      resolution({84, 84}),
      encoding("rgba_uint8"),
      gpu2gpuTransfer(false) {
  sensorType = SensorType::Color;
  sanityCheck();
}

VisualSensor::VisualSensor(scene::SceneNode& node, VisualSensorSpec::ptr spec)
    : Sensor{node, std::move(spec)}, tgt_{nullptr} {}

VisualSensor::~VisualSensor() = default;

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

void VisualSensor::setResolution(int height, int width) {
  visualSensorSpec_->resolution = {height, width};
}

void VisualSensor::setResolution(vec2i resolution) {
  visualSensorSpec_->resolution = resolution;
}

bool operator==(const VisualSensorSpec& a, const VisualSensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubType == b.sensorSubType && a.hfov == b.hfov &&
         a.ortho_scale == b.ortho_scale && a.position == b.position &&
         a.orientation == b.orientation && a.resolution == b.resolution &&
         a.encoding == b.encoding && a.noiseModel == b.noiseModel &&
         a.gpu2gpuTransfer == b.gpu2gpuTransfer;
}

}  // namespace sensor
}  // namespace esp
