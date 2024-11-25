// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CustomSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

CustomSensorAttributes::CustomSensorAttributes(const std::string& handle)
    : AbstractSensorAttributes("CustomSensorAttributes", handle) {
  addOrEditSubgroup<Configuration>("custom_attributes");
}  // CustomSensorAttributes ctor

void CustomSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Base class version
  AbstractSensorAttributes::populateWithSensorSpec(spec);
  // TODO handle setting custom_attributes subconfig with any values in spec
  // beyond base values.
  // std::shared_ptr<Configuration> custAttrs = editCustomAttrFields();
  // put values in custAttrs directly
}  // CustomSensorAttributes::populateWithSensorSpec
std::string CustomSensorAttributes::getAbstractSensorInfoHeaderInternal()
    const {
  // get every key from custom attributes
  const std::vector<std::string> sortedKeys =
      getCustomAttrFieldsView()->getKeys(true);
  std::string res = "";
  for (const std::string& key : sortedKeys) {
    Cr::Utility::formatInto(res, res.size(), "{},", key);
  }
  return res;
}  // CustomSensorAttributes::getObjectInfoHeaderInternal

std::string CustomSensorAttributes::getAbstractSensorInfoInternal() const {
  std::shared_ptr<const Configuration> custAttrs = getCustomAttrFieldsView();
  // get every key from custom attributes
  const std::vector<std::string> sortedKeys = custAttrs->getKeys(true);
  std::string res = "";
  for (const std::string& key : sortedKeys) {
    Cr::Utility::formatInto(res, res.size(), "{},",
                            custAttrs->getAsString(key));
  }
  return res;
}  // CustomSensorAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
