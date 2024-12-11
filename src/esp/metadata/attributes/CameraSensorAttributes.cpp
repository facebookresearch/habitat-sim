
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CameraSensorAttributes.h"
#include "esp/sensor/CameraSensor.h"

using Magnum::Math::Literals::operator""_degf;

namespace esp {
namespace metadata {
namespace attributes {

CameraSensorAttributes::CameraSensorAttributes(const std::string& handle)
    : AbstractVisualSensorAttributes("CameraSensorAttributes", handle) {
  init("hfov", 90.0_degf);
  init("ortho_scale", 0.1f);
}  // CameraSensorAttributes ctor

void CameraSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Parent class version
  AbstractVisualSensorAttributes::populateWithSensorSpec(spec);
  // Appropriately cast to get camera spec data if exists
  const esp::sensor::CameraSensorSpec::ptr& cameraSpec =
      std::dynamic_pointer_cast<esp::sensor::CameraSensorSpec>(spec);
  setHFOV(cameraSpec->hfov);
  setOrthoScale(cameraSpec->orthoScale);
}  // CameraSensorAttributes::populateWithSensorSpec
void CameraSensorAttributes::writeVisualSensorValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write CameraSensorAttributes values to json
  writeValueToJson("hfov", jsonObj, allocator);
  writeValueToJson("ortho_scale", jsonObj, allocator);

}  // CameraSensorAttributes::writeVisualSensorValuesToJsonInternal

std::string CameraSensorAttributes::getAbstractVisualSensorInfoHeaderInternal()
    const {
  return "Horizontal FOV,Orthographic Scale,";
}  // CameraSensorAttributes::getAbstractVisualSensorInfoHeaderInternal()

std::string CameraSensorAttributes::getAbstractVisualSensorInfoInternal()
    const {
  return Cr::Utility::formatString("{},{}", getAsString("hfov"),
                                   getAsString("ortho_scale"));

}  // CameraSensorAttributes::getAbstractVisualSensorInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
