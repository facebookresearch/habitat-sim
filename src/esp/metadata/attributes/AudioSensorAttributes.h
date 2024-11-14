// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_

#include "AbstractSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to support creating audio sensors
 */
class AudioSensorAttributes : public AbstractSensorAttributes {
 public:
  explicit AudioSensorAttributes();

 protected:
  /**
   * @brief Write Audio Sensor-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  /**
   * @brief get AbstractSensorAttributes-specific info header
   */
  std::string getAbstractSensorInfoHeaderInternal() const override;
  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(AudioSensorAttributes)
};  // class AudioSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_
