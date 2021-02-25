// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/sensor/SensorFactory.h"

#include "esp/scene/SceneNode.h"
#include "esp/sensor/Sensor.h"
#include "esp/sensor/CameraSensor.h"

namespace esp {
namespace sensor {
void SensorFactory::createSensors(
    scene::SceneNode& node,
    const sensor::SensorSetup& sensorSetup) {
  for (const sensor::SensorSpec::ptr& spec : sensorSetup) {
    scene::SceneNode& sensorNode = node.createChild();
    sensor::CameraSensor::create(sensorNode, spec);
  }
}
}  // namespace sensor
}  // namespace esp
