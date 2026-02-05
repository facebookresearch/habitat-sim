// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SensorWrapperManager.h"

namespace esp {
namespace sensor {

SensorWrapperManager::SensorWrapperManager()
    : SensorWrapperBaseManager<
          esp::sensor::ManagedSensorBase>::SensorWrapperBaseManager("Sensor") {
  this->copyConstructorMap_["ManagedAudioSensor"] =
      &SensorWrapperManager::createObjCopyCtorMapEntry<
          esp::sensor::ManagedAudioSensor>;

}  // SensorWrapperManager ctor

}  // namespace sensor
}  // namespace esp
