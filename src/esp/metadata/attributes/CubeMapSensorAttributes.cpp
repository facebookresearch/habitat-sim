// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "CubeMapSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {
AbstractCubeMapSensorAttributes::AbstractCubeMapSensorAttributes(
    const std::string& classKey,
    const std::string& handle)
    : AbstractVisualSensorAttributes(classKey, handle) {
  setHidden("__useSpecifiedCubeMapSize", false);
  // Replacing nullopt field - -1 means to ignore
  init("cubemap_size", -1);
}  // AbstractCubeMapSensorAttributes ctor

void AbstractCubeMapSensorAttributes::writeVisualSensorValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractCubeMapSensorAttributes values to json
  if (getUseSpecifiedCubeMapSize()) {
    // Only write if actually specified by user input
    writeValueToJson("cubemap_size", jsonObj, allocator);
  }
  // call child-class specific
  writeCubeMapSensorValuesToJsonInternal(jsonObj, allocator);
}  // AbstractCubeMapSensorAttributes::writeVisualSensorValuesToJsonInternal

std::string
AbstractCubeMapSensorAttributes::getAbstractVisualSensorInfoHeaderInternal()
    const {
  return "User-specified CubeMap Size (-1 means unspecified)" +
         getCubeMapSensorInfoHeaderInternal();
}  // AbstractCubeMapSensorAttributes::getAbstractVisualSensorInfoHeaderInternal()

std::string
AbstractCubeMapSensorAttributes::getAbstractVisualSensorInfoInternal() const {
  return Cr::Utility::formatString("{},{}", getAsString("cubemap_size"),
                                   getCubeMapSensorInfoInternal());
}  // AbstractCubeMapSensorAttributes::getAbstractVisualSensorInfoInternal

///////////////////////////////////
// Equirectangular CubeMap

EquirectangularSensorAttributes::EquirectangularSensorAttributes(
    const std::string& handle)
    : AbstractCubeMapSensorAttributes("EquirectangularSensorAttributes",
                                      handle) {
}  // EquirectangularSensorAttributes ctor
void EquirectangularSensorAttributes::writeCubeMapSensorValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // Currently no EquirectangularSensor-specific attributes to write
}  // EquirectangularSensorAttributes::writeCubeMapSensorValuesToJsonInternal

std::string
EquirectangularSensorAttributes::getCubeMapSensorInfoHeaderInternal() const {
  return "";
}  // EquirectangularSensorAttributes::getCubeMapSensorInfoHeaderInternal()

std::string EquirectangularSensorAttributes::getCubeMapSensorInfoInternal()
    const {
  return "";
}  // EquirectangularSensorAttributes::getCubeMapSensorInfoInternal()

///////////////////////////////////
// Fisheye CubeMap

FisheyeSensorAttributes::FisheyeSensorAttributes(const std::string& handle)
    : AbstractCubeMapSensorAttributes("FisheyeSensorAttributes", handle) {
  // init double-sphere model-specific values
  init("ds_alpha", 0.59f);
  init("ds_xi", -0.18f);
  // TODO not a legal assignment - focal length needs to be positive in both
  // fields
  init("focal_length", Magnum::Vector2(0.0, 0.0));
  setHidden("__useSpecifiedPPO", false);
  init("principle_point_offset", Magnum::Vector2(0.0, 0.0));
}  // FisheyeSensorAttributes ctor

void FisheyeSensorAttributes::writeCubeMapSensorValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // Write fisheye-sensor-specific values to json
  writeValueToJson("ds_alpha", jsonObj, allocator);
  writeValueToJson("ds_xi", jsonObj, allocator);
  writeValueToJson("focal_length", jsonObj, allocator);
  if (getUsePrincipalPointOffset()) {
    writeValueToJson("principle_point_offset", jsonObj, allocator);
  }

}  // FisheyeSensorAttributes::writeCubeMapSensorValuesToJsonInternal

std::string FisheyeSensorAttributes::getCubeMapSensorInfoHeaderInternal()
    const {
  return "Focal Length fx fy,Use PPO,Principal Point Offset,Alpha "
         "(double-sphere model),Xi (double-sphere model)";
}  // FisheyeSensorAttributes::getCubeMapSensorInfoHeaderInternal()

std::string FisheyeSensorAttributes::getCubeMapSensorInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{}", getAsString("focal_length"),
      getAsString("use_specified_ppo"), getAsString("principle_point_offset"),
      getAsString("ds_alpha"), getAsString("ds_xi"));

}  // FisheyeSensorAttributes::getCubeMapSensorInfoInternal()

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
