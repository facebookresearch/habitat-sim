// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {
std::map<std::string, std::reference_wrapper<sensor::Sensor>>&
SensorFactory::createSensors(scene::SceneNode& node,
                             const sensor::SensorSetup& sensorSetup) {
  for (const SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode =
        node.createChild({scene::SceneNodeTag::Leaf});
    // VisualSensor Setup
    if (spec->isVisualSensorSpec()) {
      if (spec->sensorSubType == SensorSubType::Orthographic ||
          spec->sensorSubType == SensorSubType::Pinhole) {
        sensorNode.addFeature<sensor::CameraSensor>(
            std::dynamic_pointer_cast<sensor::CameraSensorSpec>(spec));
      }
      // TODO: Implement fisheye sensor, Equirectangular sensor, Panorama sensor
      // else if(spec->sensorSubType == SensorSubType::Fisheye) {
      //   sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
      //
    }
    // TODO: Implement NonVisualSensorSpecs
    // else if (!spec->isVisualSensorSpec()) {}
    //   //NonVisualSensor Setup
    // }
  }
  return node.getNodeSensors();
}

void SensorFactory::deleteSensor(const sensor::Sensor& sensor) {
  delete (&sensor.node());
}

void SensorFactory::deleteSensor(scene::SceneNode& node,
                                 const std::string& uuid) {
  // If sensor does not exist, SensorFactory will log a message but proceed
  // without errors thrown
  if (node.getSubtreeSensors().count(uuid) == 0) {
    LOG(INFO) << "SensorFactory::deleteSensor(): Sensor with uuid " << uuid
              << " does not exist at node" << node.getId();
    return;
  }
  deleteSensor(node.getSubtreeSensorSuite().get(uuid));
}

}  // namespace sensor
}  // namespace esp
