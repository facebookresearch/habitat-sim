// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"

#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

namespace esp {
namespace sensor {

bool SensorSpec::operator==(const SensorSpec& a) const {
  return uuid == a.uuid && sensorType == a.sensorType &&
         sensorSubType == a.sensorSubType && position == a.position &&
         orientation == a.orientation && noiseModel == a.noiseModel;
}

bool SensorSpec::operator!=(const SensorSpec& a) const {
  return !(*this == a);
}

void SensorSpec::sanityCheck() {
  CORRADE_ASSERT(this, "SensorSpec::sanityCheck(): sensorSpec is illegal", );
  // Check that all parameters are initalized to legal values
  CORRADE_ASSERT(!uuid.empty(),
                 "SensorSpec::sanityCheck(): uuid cannot be an empty string", );
  CORRADE_ASSERT(
      sensorType >= SensorType::None && sensorType <= SensorType::Text,
      "SensorSpec::sanityCheck(): sensorType is illegal", );
  CORRADE_ASSERT(sensorSubType >= SensorSubType::None &&
                     sensorSubType <= SensorSubType::Orthographic,
                 "SensorSpec::sanityCheck(): sensorSubType is illegal", );
  CORRADE_ASSERT((abs(position.array()) >= 0).any(),
                 "SensorSpec::sanityCheck(): position is illegal", );
  CORRADE_ASSERT((abs(orientation.array()) >= 0).any(),
                 "SensorSpec::sanityCheck(): orientation is illegal", );
  CORRADE_ASSERT(!noiseModel.empty(),
                 "SensorSpec::sanityCheck(): noiseModel is unitialized", );
}

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Magnum::SceneGraph::AbstractFeature3D{node}, spec_(std::move(spec)) {
  node.setType(scene::SceneNodeType::SENSOR);
  CORRADE_ASSERT(spec_,
                 "Sensor::Sensor(): Cannot initialize sensor. The "
                 "specification is null.", );
  spec_->sanityCheck();
  setTransformationFromSpec();
}

void Sensor::setTransformationFromSpec() {
  CORRADE_ASSERT(spec_,
                 "Sensor::setTransformationFromSpec: Cannot set "
                 "transformation, the specification is null.", );

  node().resetTransformation();

  node().translate(Magnum::Vector3(spec_->position));
  node().rotateX(Magnum::Rad(spec_->orientation[0]));
  node().rotateY(Magnum::Rad(spec_->orientation[1]));
  node().rotateZ(Magnum::Rad(spec_->orientation[2]));
}

void SensorSuite::add(const Sensor::ptr& sensor) {
  const std::string uuid = sensor->specification()->uuid;
  sensors_[uuid] = sensor;
}

void SensorSuite::merge(SensorSuite& sensorSuite) {
  sensors_.insert(sensorSuite.getSensors().begin(),
                  sensorSuite.getSensors().end());
}

Sensor::ptr SensorSuite::get(const std::string& uuid) const {
  return (sensors_.at(uuid));
}

void SensorSuite::clear() {
  sensors_.clear();
}

}  // namespace sensor
}  // namespace esp
