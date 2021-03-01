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

  node.getNodeSensorSuite().add(*this);
  node.getSubtreeSensorSuite().add(*this);
  // Traverse up to root node and add sensor to every subtreeSensorSuite
  auto parent = node.parent();
  while(parent->parent() != nullptr) {
    static_cast<scene::SceneNode&>(*parent).getSubtreeSensorSuite().add(*this);
    parent = parent->parent();
  }
  setTransformationFromSpec();
}

Sensor::~Sensor() {
  LOG(INFO) << "Deconstructing SensorSuite";
  node().getNodeSensorSuite().remove(*this);
  node().getSubtreeSensorSuite().remove(*this);
  // Traverse up to root node and remove sensor from every subtreeSensorSuite
  auto parent = node().parent();
  while(parent->parent() != nullptr) {
    static_cast<scene::SceneNode&>(*parent).getSubtreeSensorSuite().remove(*this);
    parent = parent->parent();
  }
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

SensorSuite::SensorSuite(scene::SceneNode& node)
    : Magnum::SceneGraph::AbstractFeature3D{node} {}

void SensorSuite::add(sensor::Sensor& sensor) {
  const std::string uuid = sensor.specification()->uuid;
  sensors_.emplace(uuid, std::ref(sensor));
}

void SensorSuite::merge(SensorSuite& sensorSuite) {
  sensors_.insert(sensorSuite.getSensors().begin(),
                  sensorSuite.getSensors().end());
}

void SensorSuite::remove(sensor::Sensor& sensor) {
  const std::string uuid = sensor.specification()->uuid;
  std::map<std::string, std::reference_wrapper<sensor::Sensor>>::iterator it = sensors_.find(uuid);
  sensors_.erase(it);
}

sensor::Sensor& SensorSuite::get(const std::string& uuid) const{
  return sensors_.at(uuid).get();
}

void SensorSuite::clear() {
  sensors_.clear();
}


bool operator==(const SensorSpec& a, const SensorSpec& b) {
  return a.uuid == b.uuid && a.sensorType == b.sensorType &&
         a.sensorSubType == b.sensorSubType && a.parameters == b.parameters &&
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
