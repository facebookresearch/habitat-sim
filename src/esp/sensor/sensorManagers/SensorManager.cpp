// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SensorManager.h"

namespace esp {
namespace sensor {

SensorManager::SensorManager()
    : SensorBaseManager<esp::sensor::ManagedSensorBase>::SensorBaseManager(
          "Sensor") {
  this->copyConstructorMap_["ManagedAudioSensor"] =
      &SensorManager::createObjCopyCtorMapEntry<
          esp::sensor::ManagedAudioSensor>;

}  // SensorManager ctor

}  // namespace sensor
}  // namespace esp
