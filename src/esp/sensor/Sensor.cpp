// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"

#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

namespace esp {
namespace sensor {

SensorSpec::~SensorSpec() {}

bool SensorSpec::operator==(const SensorSpec& a) const {
  return uuid == a.uuid && sensorType == a.sensorType &&
         sensorSubType == a.sensorSubType && position == a.position &&
         orientation == a.orientation && noiseModel == a.noiseModel;
}

bool SensorSpec::operator!=(const SensorSpec& a) const {
  return !(*this == a);
}

SensorSpec::SensorSpec()
    : uuid(""),
      sensorType(SensorType::None),
      sensorSubType(SensorSubType::None),
      position({0, 1.5, 0}),
      orientation({0, 0, 0}),
      noiseModel("None") {}

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Magnum::SceneGraph::AbstractFeature3D{node}, spec_(std::move(spec)) {
  node.setType(scene::SceneNodeType::SENSOR);
  if (spec_ == nullptr) {
    LOG(ERROR) << "Cannot initialize sensor. The specification is null.";
  }
  ASSERT(spec_ != nullptr);
  setTransformationFromSpec();
}

Sensor::~Sensor() {
  LOG(INFO) << "Deconstructing Sensor";
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
