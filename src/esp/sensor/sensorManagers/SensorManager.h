// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_SENSORMANAGER_H_
#define ESP_SENSOR_SENSORMANAGER_H_

#include "SensorBaseManager.h"
#include "esp/sensor/sensorWrappers/ManagedAudioSensor.h"
#include "esp/sensor/sensorWrappers/ManagedSensorBase.h"

namespace esp {
namespace sensor {

class SensorManager : public SensorBaseManager<esp::sensor::ManagedSensorBase> {
 public:
  explicit SensorManager();

 public:
  ESP_SMART_POINTERS(SensorManager)
};  // class SensorManager

}  // namespace sensor
}  // namespace esp
#endif  // ESP_SENSOR_SENSORMANAGER_H_
