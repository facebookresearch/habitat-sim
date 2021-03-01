// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/CameraSensor.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {
void SensorFactory::createSensors(scene::SceneNode& node,
                                  const sensor::SensorSetup& sensorSetup) {
  for (const SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();
    // VisualSensor Setup
    if (spec->isVisualSensorSpec()) {
      if (spec->sensorSubType == SensorSubType::Orthographic ||
          spec->sensorSubType == SensorSubType::Pinhole) {
        sensor::CameraSensor::create(
            sensorNode,
            std::dynamic_pointer_cast<sensor::CameraSensorSpec>(spec));
      }
      // TODO: Implement fisheye sensor, Equirectangle sensor, Panorama sensor
      // else if(spec->sensorSubType == SensorSubType::Fisheye) {
      //   sensorSuite.add(sensor::FisheyeSensor::create(sensorNode, spec));
      //
    }
    // TODO: Implement NonVisualSensorSpecs
    // else if (!spec->isVisualSensorSpec()) {}
    //   //NonVisualSensor Setup
    // }
  }
}
}  // namespace sensor
}  // namespace esp
