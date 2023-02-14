// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Sensor.h"
#include <Magnum/EigenIntegration/Integration.h>
#include "esp/core/Check.h"
#include "esp/scene/SceneGraph.h"

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

void SensorSpec::sanityCheck() const {
  CORRADE_ASSERT(this, "SensorSpec::sanityCheck(): sensorSpec is illegal", );
  // Check that all parameters are initialized to legal values
  CORRADE_ASSERT(!uuid.empty(),
                 "SensorSpec::sanityCheck(): uuid cannot be an empty string", );
  CORRADE_ASSERT(
      sensorType > SensorType::None && sensorType < SensorType::SensorTypeCount,
      "SensorSpec::sanityCheck(): sensorType" << int32_t(sensorType)
                                              << "is illegal", );
  CORRADE_ASSERT(sensorSubType > SensorSubType::None &&
                     sensorSubType < SensorSubType::SensorSubTypeCount,
                 "SensorSpec::sanityCheck(): sensorSubType"
                     << int32_t(sensorType) << "is illegal", );
  CORRADE_ASSERT((abs(position.array()) >= 0).any(),
                 "SensorSpec::sanityCheck(): position is illegal", );
  CORRADE_ASSERT((abs(orientation.array()) >= 0).any(),
                 "SensorSpec::sanityCheck(): orientation is illegal", );
  CORRADE_ASSERT(!noiseModel.empty(),
                 "SensorSpec::sanityCheck(): noiseModel is unitialized", );
}

Sensor::Sensor(scene::SceneNode& node, SensorSpec::ptr spec)
    : Magnum::SceneGraph::AbstractFeature3D{node}, spec_(std::move(spec)) {
  CORRADE_ASSERT(node.children().first() == nullptr,
                 "Sensor::Sensor(): Cannot attach a sensor to a non-LEAF node. "
                 "The number of children of this node is not zero.", );
  CORRADE_ASSERT(
      node.getSceneNodeTags() & scene::SceneNodeTag::Leaf,
      "Sensor::Sensor(): Cannot attach a sensor to a non-LEAF node.", );
  node.setType(scene::SceneNodeType::SENSOR);
  CORRADE_ASSERT(spec_,
                 "Sensor::Sensor(): Cannot initialize sensor. The "
                 "specification is null.", );
  spec_->sanityCheck();
  node.getNodeSensorSuite().add(*this);
  node.getSubtreeSensorSuite().add(*this);
  node.addSensorToParentNodeSensorSuite();
  // Traverse up to root node and add sensor to every subtreeSensorSuite
  node.addSubtreeSensorsToAncestors();
  setTransformationFromSpec();
}

Sensor::~Sensor() {
  // Updating of info in SensorSuites will be handled by SceneNode
  ESP_DEBUG() << "Deconstructing Sensor";
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

SensorSuite::SensorSuite(scene::SceneNode& node)
    : Magnum::SceneGraph::AbstractFeature3D{node} {}

void SensorSuite::add(sensor::Sensor& sensor) {
  sensors_.emplace(sensor.specification()->uuid, std::ref(sensor));
}

void SensorSuite::remove(const sensor::Sensor& sensor) {
  remove(sensor.specification()->uuid);
}

void SensorSuite::remove(const std::string& uuid) {
  sensors_.erase(uuid);
}

sensor::Sensor& SensorSuite::get(const std::string& uuid) const {
  auto sensorIter = sensors_.find(uuid);
  ESP_CHECK(
      sensorIter != sensors_.end(),
      "SensorSuite::get(): SensorSuite does not contain key:" << uuid.c_str());
  return sensorIter->second.get();
}

void SensorSuite::clear() {
  sensors_.clear();
}

}  // namespace sensor
}  // namespace esp
