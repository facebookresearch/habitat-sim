// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CustomSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

CustomSensorAttributes::CustomSensorAttributes(const std::string& handle)
    : AbstractSensorAttributes("CustomSensorAttributes", handle) {
}  // CustomSensorAttributes ctor

void CustomSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Base class version
  AbstractSensorAttributes::populateWithSensorSpec(spec);
}  // CustomSensorAttributes::populateWithSensorSpec
std::string CustomSensorAttributes::getObjectInfoHeaderInternal() const {
  const std::vector<std::string> sortedKeys = getKeys(true);
  std::string res = "";
  for (const std::string& key : sortedKeys) {
    Cr::Utility::formatInto(res, res.size(), "{},", key);
  }
  return res;
}  // CustomSensorAttributes::getObjectInfoHeaderInternal

std::string CustomSensorAttributes::getObjectInfoInternal() const {
  const std::vector<std::string> sortedKeys = getKeys(true);
  std::string res = "";
  for (const std::string& key : sortedKeys) {
    Cr::Utility::formatInto(res, res.size(), "{},", getAsString(key));
  }
  return res;
}  // CustomSensorAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
