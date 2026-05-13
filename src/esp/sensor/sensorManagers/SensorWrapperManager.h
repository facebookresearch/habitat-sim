// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_SENSORWRAPPERMANAGER_H_
#define ESP_SENSOR_SENSORWRAPPERMANAGER_H_

#include "SensorWrapperBaseManager.h"
#include "esp/sensor/sensorWrappers/ManagedAudioSensor.h"
#include "esp/sensor/sensorWrappers/ManagedSensorBase.h"

namespace esp {
namespace sensor {

class SensorWrapperManager
    : public SensorWrapperBaseManager<esp::sensor::ManagedSensorBase> {
 public:
  explicit SensorWrapperManager();

 public:
  ESP_SMART_POINTERS(SensorWrapperManager)
};  // class SensorWrapperManager

}  // namespace sensor
}  // namespace esp
#endif  // ESP_SENSOR_SENSORWRAPPERMANAGER_H_
