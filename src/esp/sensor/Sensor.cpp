// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"

#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

namespace esp {
namespace sensor {

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Magnum::SceneGraph::AbstractFeature3D{node}, spec_(std::move(spec)) {
  node.setType(scene::SceneNodeType::SENSOR);
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. The specification is null.";
  }
  ASSERT(spec_ != nullptr);

  setTransformationFromSpec();
}

void SensorSuite::add(const Sensor::ptr& sensor) {
  const std::string uuid = sensor->specification()->uuid;
  sensors_[uuid] = sensor;
}

Sensor::ptr SensorSuite::get(const std::string& uuid) const {
  return (sensors_.at(uuid));
}

void SensorSuite::clear() {
  sensors_.clear();
}

void Sensor::setTransformationFromSpec() {
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

bool operator==(const SensorSpec& a, const SensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubtype == b.sensorSubtype && a.parameters == b.parameters &&
         a.position == b.position && a.orientation == b.orientation &&
         a.resolution == b.resolution && a.channels == b.channels &&
         a.encoding == b.encoding && a.observationSpace == b.observationSpace &&
         a.noiseModel == b.noiseModel && a.gpu2gpuTransfer == b.gpu2gpuTransfer;
}
bool operator!=(const SensorSpec& a, const SensorSpec& b) {
  return !(a == b);
}

}  // namespace sensor
}  // namespace esp
