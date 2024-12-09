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
  // Replacing nullopt field - 0 means to ignore
  init("cubemap_size", 0);
}  // AbstractCubeMapSensorAttributes ctor

void AbstractCubeMapSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Parent class version
  AbstractVisualSensorAttributes::populateWithSensorSpec(spec);
  // Appropriately cast to get camera spec data if exists
  const esp::sensor::CubeMapSensorBaseSpec::ptr& cubemapSpec =
      std::dynamic_pointer_cast<esp::sensor::CubeMapSensorBaseSpec>(spec);
  if (cubemapSpec->cubemapSize != Cr::Containers::NullOpt) {
    // automatically sets useSpecifiedCubeMapSize boolean to true
    setCubeMapSize(*cubemapSpec->cubemapSize);
  }
}  // AbstractCubeMapSensorAttributes::populateWithSensorSpec

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
  return "User-specified CubeMap Size (0 : use min dim of resolution)" +
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

void EquirectangularSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Parent class version
  AbstractCubeMapSensorAttributes::populateWithSensorSpec(spec);
  // Appropriately cast to get EquirectangularSensorAttributes-specific spec
  // data if exists
  // No EquirectangularSensorAttributes-specific data
}  // EquirectangularSensorAttributes::populateWithSensorSpec

void EquirectangularSensorAttributes::writeCubeMapSensorValuesToJsonInternal(
    CORRADE_UNUSED io::JsonGenericValue& jsonObj,
    CORRADE_UNUSED io::JsonAllocator& allocator) const {
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
  init("focal_length", Magnum::Vector2(0.01, 0.01));
  setHidden("__useSpecifiedPPO", false);
  init("principle_point_offset", Magnum::Vector2(0.0, 0.0));
}  // FisheyeSensorAttributes ctor

void FisheyeSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Parent class version
  AbstractCubeMapSensorAttributes::populateWithSensorSpec(spec);
  // Appropriately cast to get FisheyeSensorDoubleSphereSpec-spec data if exists
  const esp::sensor::FisheyeSensorDoubleSphereSpec::ptr& fisheyeDSSpec =
      std::dynamic_pointer_cast<esp::sensor::FisheyeSensorDoubleSphereSpec>(
          spec);
  setFocalLength(fisheyeDSSpec->focalLength);
  if (fisheyeDSSpec->principalPointOffset != Cr::Containers::NullOpt) {
    setPrincipalPointOffset(*fisheyeDSSpec->principalPointOffset);
  }
  setDoubleSphereXi(fisheyeDSSpec->xi);
  setDoubleSphereAlpha(fisheyeDSSpec->alpha);
  // Currently no other Fisheye algorithm is implemented

}  // FisheyeSensorAttributes::populateWithSensorSpec

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
