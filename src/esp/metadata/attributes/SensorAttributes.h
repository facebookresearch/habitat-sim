// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SENSORATTRIBUTES_H_

#include "AbstractAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Attributes object holding the descriptions of a Sensor
 */
class SensorAttributes : public AbstractAttributes {
 public:
  SensorAttributes(const std::string& classKey, const std::string& handle);

  ~SensorAttributes() override = default;

 protected:
 public:
  ESP_SMART_POINTERS(SensorAttributes)
};  // class SensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SENSORATTRIBUTES_H_
