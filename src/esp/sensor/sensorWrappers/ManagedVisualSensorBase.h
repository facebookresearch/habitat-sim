// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDVISUALSENSORBASE_H_
#define ESP_SENSOR_MANAGEDVISUALSENSORBASE_H_

#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Macros.h>

#include "ManagedSensorTemplates.h"
#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace sensor {

/**
 * @brief Base class template for wrapper for visual sensor objects of all kinds
 * that extends AbstractManagedSensorAccess
 */

template <class T>
class AbstractManagedVisualSensor
    : public esp::sensor::AbstractManagedSensor<T> {
 public:
  static_assert(std::is_base_of<esp::sensor::VisualSensor, T>::value,
                "AbstractManagedVisualSensor :: Managed visual sensor object "
                "type must be derived from esp::sensor::VisualSensor");

  explicit AbstractManagedVisualSensor(const std::string& classKey)
      : AbstractManagedSensor<T>(classKey) {}
  // TODO Add appropriate abstract visual sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */

  std::string getSensorObjInfoHeaderInternal() const override {
    return "" + getVisualSensorObjInfoHeaderInternal();
  }
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */

  virtual std::string getVisualSensorObjInfoHeaderInternal() const = 0;
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */
  std::string getSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<VisualSensor>& sp) const override {
    // TODO provide info stream for sensors
    std::string res = Cr::Utility::formatString(
        "{},{}", "VisualSensor", getVisualSensorObjInfoInternal());

    return res;
  }

  /**
   * @brief Specialization-specific extension of getSensorObjInfoInternal, comma
   * separated info ideal for saving to csv
   */
  virtual std::string getVisualSensorObjInfoInternal(
      std::shared_ptr<T>& sp) const = 0;

 public:
  ESP_SMART_POINTERS(AbstractManagedVisualSensor<T>)
};  // class AbstractManagedVisualSensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDVISUALSENSORBASE_H_
