// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"
#include <Magnum/EigenIntegration/Integration.h>

#include <utility>

namespace esp {
namespace sensor {

SensorSpec::~SensorSpec() {
  LOG(INFO) << "Deconstructing SensorSpec";
}

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
  node.getNodeSensorSuite().add(*this);
  static_cast<scene::SceneNode&>(*node.parent())
      .getNodeSensorSuite()
      .add(*this);
  node.getSubtreeSensorSuite().add(*this);
  // Traverse up to root node and add sensor to every subtreeSensorSuite
  auto parent = node.parent();
  while (parent->parent() != nullptr) {
    static_cast<scene::SceneNode&>(*parent).getSubtreeSensorSuite().add(*this);
    parent = parent->parent();
  }
  setTransformationFromSpec();
}

Sensor::~Sensor() {
  LOG(INFO) << "Deconstructing Sensor";
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

void Sensor::deleteSensor() {
  // Traverse up to root node and remove sensor from every subtreeSensorSuite,
  // only if the sensor is deleted and the sceneNode still exists
  node().getNodeSensorSuite().remove(*this);
  node().getSubtreeSensorSuite().remove(*this);
  if (node().parent() != nullptr) {
    auto current = node().parent();
    static_cast<scene::SceneNode&>(*current).getNodeSensorSuite().remove(*this);
    while (current->parent() != nullptr) {
      static_cast<scene::SceneNode&>(*current).getSubtreeSensorSuite().remove(
          *this);
      current = current->parent();
    }
  }
  delete this;
}

SensorSuite::SensorSuite(scene::SceneNode& node)
    : Magnum::SceneGraph::AbstractFeature3D{node} {}

void SensorSuite::add(sensor::Sensor& sensor) {
  sensors_.emplace(sensor.specification()->uuid, std::ref(sensor));
}

void SensorSuite::merge(const SensorSuite& sensorSuite) {
  sensors_.insert(sensorSuite.getSensors().begin(),
                  sensorSuite.getSensors().end());
}

void SensorSuite::remove(const sensor::Sensor& sensor) {
  sensors_.erase(sensor.specification()->uuid);
}

sensor::Sensor& SensorSuite::get(const std::string& uuid) const {
  return sensors_.at(uuid).get();
}

void SensorSuite::clear() {
  sensors_.clear();
}

}  // namespace sensor
}  // namespace esp
