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
 * @brief Attributes object holding the descriptions of a Sensor object
 */
class AbstractSensorAttributes : public AbstractAttributes {
 public:
  AbstractSensorAttributes(const std::string& classKey,
                           const std::string& handle);

  ~AbstractSensorAttributes() override = default;

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration. Default is overridden to handle special cases for
   * AbstractSensorAttributes and deriving (i.e. AudioSensorAttributes or
   * VisualSensorAttributes) classes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Write child-class-specific values to json object
   *
   */
  virtual void writeValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {}

 public:
  ESP_SMART_POINTERS(AbstractSensorAttributes)
};  // class SensorAttributes

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

 public:
  ESP_SMART_POINTERS(AudioSensorAttributes)
};  // class AudioSensorAttributes

/**
 * @brief Class to suppoort creating visual sensors
 */
class VisualSensorAttributes : public AbstractSensorAttributes {
 public:
 protected:
  /**
   * @brief Write Audio Sensor-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

 public:
  ESP_SMART_POINTERS(VisualSensorAttributes)
};  // class VisualSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SENSORATTRIBUTES_H_
