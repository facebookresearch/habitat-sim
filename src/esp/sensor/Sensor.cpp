// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"

#include <Magnum/EigenIntegration/Integration.h>

namespace esp {
namespace sensor {

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : scene::AttachedObject(node, scene::AttachedObjectType::SENSOR),
      spec_(spec) {
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. The specification is null.";
  }
  ASSERT(spec_ != nullptr);

  setTransformationFromSpec();
}

Observation Sensor::getObservation() {
  // TODO fill out observation
  Observation obs{};
  return obs;
}

void SensorSuite::add(Sensor::ptr sensor) {
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

  object().resetTransformation();

  object().translate(Magnum::Vector3(spec_->position));
  object().rotateX(Magnum::Rad(spec_->orientation[0]));
  object().rotateY(Magnum::Rad(spec_->orientation[1]));
  object().rotateZ(Magnum::Rad(spec_->orientation[2]));
}

bool operator==(const SensorSpec& a, const SensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubtype == b.sensorSubtype && a.parameters == b.parameters &&
         a.position == b.position && a.orientation == b.orientation &&
         a.resolution == b.resolution && a.channels == b.channels &&
         a.encoding == b.encoding && a.observationSpace == b.observationSpace;
}
bool operator!=(const SensorSpec& a, const SensorSpec& b) {
  return !(a == b);
}

}  // namespace sensor
}  // namespace esp
