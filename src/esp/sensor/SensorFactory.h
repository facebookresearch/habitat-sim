// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/scene/SceneNode.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace sensor {
class SensorFactory {
 public:
  /**
   * @brief Static method to initialize SensorSuite of Sensors
   * @param[in] node SceneNode of SensorSuite to create child SceneNodes and attach sensors to
   * @param[in] sensorSetup SensorSetup a vector of SensorSpec::ptr defining specs for each
   * sensor
   *
   */
  static void createSensors(
      scene::SceneNode& node,
      const sensor::SensorSetup& sensorSetup);
};
}  // namespace sensor
}  // namespace esp
