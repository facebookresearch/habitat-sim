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

  /**
   * @brief Populate this CustomSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;

  /**
   * @brief Gets a smart pointer reference to a copy of the custom attributes
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object.
   */
  std::shared_ptr<Configuration> getCustomAttrFields() const {
    return getSubconfigCopy<Configuration>("custom_attributes");
  }

  /**
   * @brief Gets a const smart pointer reference to a view of the custom
   * attributes configuration data from config file. Habitat does not parse or
   * process this data, but it will be available to the user via python bindings
   * for each object.
   */
  std::shared_ptr<const Configuration> getCustomAttrFieldsView() const {
    return getSubconfigView("custom_attributes");
  }

  /**
   * @brief Gets a smart pointer reference to the actual custom attributes
   * configuration data from config file. Habitat does not parse or process this
   * data, but it will be available to the user via python bindings for each
   * object. This method is for editing the configuration.
   */
  std::shared_ptr<Configuration> editCustomAttrFields() {
    return editSubconfig<Configuration>("custom_attributes");
  }

  /**
   * @brief Move an existing custom_attributes subconfiguration into this
   * configuration, overwriting the existing copy if it exists. Habitat does not
   * parse or process this data, but it will be available to the user via python
   * bindings for each object. This method is for editing the configuration.
   */
  void setCustomAttrFields(std::shared_ptr<Configuration>& custAttrs) {
    setSubconfigPtr("custom_attributes", custAttrs);
  }

  /**
   * @brief Returns the number of custom attributes values (within the
   * "custom_attributes" sub-Configuration) this attributes has.
   */
  int getNumCustomAttrFields() const {
    return getSubconfigNumEntries("custom_attributes");
  }

  /**
   * @brief Returns the number of custom attributes values and subconfig values
   * (recursive) (within the "custom_attributes" sub-Configuration) this
   * attributes has in its entire tree.
   */
  int getTotalNumCustomAttrFields() const {
    return getSubconfigTreeNumEntries("custom_attributes");
  }

 protected:
  /**
   * @brief get AbstractSensorAttributes-specific info header
   */
  std::string getAbstractSensorInfoHeaderInternal() const override;

  /**
   * @brief get AbstractSensorAttributes specific info for csv string
   */
  std::string getAbstractSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(CustomSensorAttributes)
};  // class CustomSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_CUSTOMSENSORATTRIBUTES_H_
