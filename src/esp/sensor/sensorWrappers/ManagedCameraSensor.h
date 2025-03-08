// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDCAMERASENSOR_H_
#define ESP_SENSOR_MANAGEDCAMERASENSOR_H_

#include "ManagedVisualSensorBase.h"
#include "esp/sensor/CameraSensor.h"

namespace esp {
namespace sensor {

/**
 * @brief Class for wrapper for camera sensor objects
 */
class ManagedCameraSensor : public AbstractManagedVisualSensor<CameraSensor> {
 public:
  explicit ManagedCameraSensor(
      const std::string& classKey = "ManagedCameraSensor")
      : AbstractManagedVisualSensor<CameraSensor>::AbstractManagedVisualSensor(
            classKey) {}

  // TODO Add appropriate camera sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */
  std::string getVisualSensorObjInfoHeaderInternal() const override {
    return "";
  }

  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */

  std::string getVisualSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<esp::sensor::CameraSensor>& sp)
      const override {
    // TODO provide info stream for sensors
    return "";
  }

 public:
  ESP_SMART_POINTERS(ManagedCameraSensor)
};  // class ManagedCameraSensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDCAMERASENSOR_H_
