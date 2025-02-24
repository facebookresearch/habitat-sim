// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDFISHEYESENSOR_H_
#define ESP_SENSOR_MANAGEDFISHEYESENSOR_H_

#include "ManagedCubeMapSensorBase.h"
#include "esp/sensor/FisheyeSensor.h"

namespace esp {
namespace sensor {

/**
 * @brief Class for wrapper for sensor objects of all kinds to
 * enable Managed Container access.
 */
class ManagedFisheyeSensor
    : public AbstractManagedCubeMapSensorBase<FisheyeSensor> {
 public:
  explicit ManagedFisheyeSensor(
      const std::string& classKey = "ManagedFisheyeSensor")
      : AbstractManagedCubeMapSensorBase<
            FisheyeSensor>::AbstractManagedCubeMapSensorBase(classKey) {}

  // TODO Add appropriate fisheye sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */
  std::string getCubeMapSensorObjInfoHeaderInternal() const override {
    return "";
  }

  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */

  std::string getCubeMapSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<FisheyeSensor>& sp) const override {
    // TODO provide info stream for sensors
    return "";
  }

 public:
  ESP_SMART_POINTERS(ManagedFisheyeSensor)
};  // namespace sensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDFISHEYESENSOR_H_
