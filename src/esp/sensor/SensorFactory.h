// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
   * @param[in] node SceneNode of SensorSuite to create child SceneNodes and
   * attach sensors to
   * @param[in] sensorSetup SensorSetup a vector of SensorSpec::ptr defining
   * specs for each sensor
   *
   */
  static std::map<std::string, std::reference_wrapper<sensor::Sensor>>&
  createSensors(scene::SceneNode& node, const sensor::SensorSetup& sensorSetup);

  /**
   * @brief Static method to delete a Sensor
   * @param[in] sensor Sensor to delete
   *
   */
  static void deleteSensor(const sensor::Sensor& sensor);

  /**
   * @brief Static method to delete a Sensor from the subtree rooted at node
   * @param[in] node root SceneNode of the subtree in which the Sensor will be
   * searched
   * @param[in] uuid Sensor's uuid
   * NOTE: If sensor does not exist, SensorFactory will log a message but
   * proceed without any errors thrown
   *
   */
  static void deleteSubtreeSensor(scene::SceneNode& node,
                                  const std::string& uuid);
};
}  // namespace sensor
}  // namespace esp
