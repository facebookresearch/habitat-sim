// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDCUBEMAPSENSORBASE_H_
#define ESP_SENSOR_MANAGEDCUBEMAPSENSORBASE_H_

#include "ManagedVisualSensorBase.h"
#include "esp/sensor/CubeMapSensorBase.h"

namespace esp {
namespace sensor {

/**
 * @brief Class Template for wrapper for CubeMap sensor objects
 */
template <class T>
class AbstractManagedCubeMapSensorBase
    : public esp::sensor::AbstractManagedVisualSensor<T> {
 public:
  static_assert(
      std::is_base_of<esp::sensor::CubeMapSensorBase, T>::value,
      "AbstractManagedCubeMapSensorBase :: Managed CubeMap sensor object "
      "type must be derived from esp::sensor::CubeMapSensorBase");

  explicit AbstractManagedCubeMapSensorBase(const std::string& classKey)
      : AbstractManagedVisualSensor<T>(classKey) {}

  // TODO Add appropriate camera sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */
  std::string getVisualSensorObjInfoHeaderInternal() const override {
    return "" + getCubeMapSensorObjInfoHeaderInternal();
  }

  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */

  virtual std::string getCubeMapSensorObjInfoHeaderInternal() const = 0;
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */

  std::string getVisualSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<T>& sp) const override {
    // TODO provide info stream for sensors
    std::string res =
        Cr::Utility::formatString("{},", getCubeMapSensorObjInfoInternal());

    return res;
  }

  /**
   * @brief Specialization-specific extension of getSensorObjInfoInternal, comma
   * separated info ideal for saving to csv
   */
  virtual std::string getCubeMapSensorObjInfoInternal(
      std::shared_ptr<T>& sp) const = 0;

 public:
  ESP_SMART_POINTERS(AbstractManagedCubeMapSensorBase<T>)
};  // class ManagedCubeMapSensorBase

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDCUBEMAPSENSORBASE_H_
