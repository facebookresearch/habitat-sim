// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "VisualSensor.h"
#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

#include "esp/gfx/RenderTarget.h"

namespace esp {
namespace sensor {

VisualSensor::VisualSensor(scene::SceneNode& node, VisualSensorSpec::ptr spec)
    : Sensor{node, spec}, tgt_{nullptr}, spec_{spec} {
  setTransformationFromSpec();
}

VisualSensor::~VisualSensor() = default;

void VisualSensor::bindRenderTarget(gfx::RenderTarget::uptr&& tgt) {
  if (tgt->framebufferSize() != framebufferSize())
    throw std::runtime_error("RenderTarget is not the correct size");

  tgt_ = std::move(tgt);
}

void VisualSensor::setTransformationFromSpec() {
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. the specification is null.";
    return;
  }

  node().resetTransformation();

  node().translate(Magnum::Vector3(spec_->position));
  node().rotateX(Magnum::Rad(spec_->orientation[0]));
  node().rotateY(Magnum::Rad(spec_->orientation[1]));
  node().rotateZ(Magnum::Rad(spec_->orientation[2]));
}

void VisualSensor::updateResolution(float x_res, float y_res) {
  spec_->resolution = {x_res, y_res};
}

bool operator==(const VisualSensorSpec& a, const VisualSensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubType == b.sensorSubType && a.position == b.position &&
         a.orientation == b.orientation && a.resolution == b.resolution &&
         a.encoding == b.encoding && a.observationSpace == b.observationSpace &&
         a.noiseModel == b.noiseModel && a.gpu2gpuTransfer == b.gpu2gpuTransfer;
}

}  // namespace sensor
}  // namespace esp
