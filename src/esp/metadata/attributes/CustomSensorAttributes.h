// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_CUSTOMSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_CUSTOMSENSORATTRIBUTES_H_

#include "AbstractSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to support creating custom sensors via python that will be
 * managed in Sim.
 */
class CustomSensorAttributes : public AbstractSensorAttributes {
 public:
  explicit CustomSensorAttributes(const std::string& handle = "");

 protected:
  /**
   * @brief Write CustomSensorAttributes data to JSON
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override {
    Configuration::writeValuesToJson(jsonObj, allocator);
  };

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(CustomSensorAttributes)
};  // class CustomSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_CUSTOMSENSORATTRIBUTES_H_
