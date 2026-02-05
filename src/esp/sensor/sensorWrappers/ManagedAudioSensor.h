// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDAUDIOSENSOR_H_
#define ESP_SENSOR_MANAGEDAUDIOSENSOR_H_

#include "ManagedSensorTemplates.h"
#include "esp/sensor/AudioSensor.h"

namespace esp {
namespace sensor {

/**
 * @brief Class for wrapper for sensor objects of all kinds to
 * enable Managed Container access.
 */
class ManagedAudioSensor
    : public esp::sensor::AbstractManagedSensor<AudioSensor> {
 public:
  explicit ManagedAudioSensor()
      : AbstractManagedSensor<AudioSensor>::AbstractManagedSensor(
            "ManagedAudioSensor") {}

  // TODO Add appropriate audio sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */

  std::string getSensorObjInfoHeaderInternal() const override {
    return "Output Directory";
  }
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */
  std::string getSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<AudioSensor>& sp) const override {
    // TODO provide info stream for sensors
    return "";
  }

 public:
  ESP_SMART_POINTERS(ManagedAudioSensor)
};  // class ManagedAudioSensor
}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDAUDIOSENSOR_H_
