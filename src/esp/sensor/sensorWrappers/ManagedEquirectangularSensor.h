// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDEQUIRECTANGULARSENSOR_H_
#define ESP_SENSOR_MANAGEDEQUIRECTANGULARSENSOR_H_

#include "ManagedCubeMapSensorBase.h"
#include "esp/sensor/EquirectangularSensor.h"

namespace esp {
namespace sensor {

/**
 * @brief Class for wrapper for sensor objects of all kinds to
 * enable Managed Container access.
 */
class ManagedEquirectangularSensor
    : public AbstractManagedCubeMapSensorBase<EquirectangularSensor> {
 public:
  explicit ManagedEquirectangularSensor(
      const std::string& classKey = "ManagedEquirectangularSensor")
      : AbstractManagedCubeMapSensorBase<
            EquirectangularSensor>::AbstractManagedCubeMapSensorBase(classKey) {
  }

  // TODO Add appropriate equirectangular sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed Equirectangular CubeMap sensor,
   * type-specific.
   */
  std::string getCubeMapSensorObjInfoHeaderInternal() const override {
    return "";
  }

  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */

  std::string getCubeMapSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<EquirectangularSensor>& sp)
      const override {
    // TODO provide info stream for sensors
    return "";
  }

 public:
  ESP_SMART_POINTERS(ManagedEquirectangularSensor)
};  // namespace sensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDEQUIRECTANGULARSENSOR_H_
