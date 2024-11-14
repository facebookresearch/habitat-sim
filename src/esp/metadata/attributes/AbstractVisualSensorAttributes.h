// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_

#include "AbstractSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to suppoort creating visual sensors by providing common
 * attributes.
 */
class AbstractVisualSensorAttributes : public AbstractSensorAttributes {
 public:
 protected:
  /**
   * @brief Write Visual Sensor-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  virtual void writeVisualSensorValuesToJsonInternal(
      io::JsonGenericValue& jsonObj,
      io::JsonAllocator& allocator) const;

 public:
  ESP_SMART_POINTERS(AbstractVisualSensorAttributes)
};  // class VisualSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_
