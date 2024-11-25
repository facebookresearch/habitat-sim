// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractSensorAttributes.h"
namespace esp {
namespace metadata {
namespace attributes {

AbstractSensorAttributes::AbstractSensorAttributes(
    const std::string& attributesClassKey,
    const std::string& handle)
    : AbstractAttributes(attributesClassKey, handle) {
  init("position", Mn::Vector3{0.0, 1.5, 0.0});
  init("orientation", Mn::Vector3{0.0, 0.0, 0.0});
  init("noise_model", "None");

  initTranslated("sensor_type",
                 getSensorTypeName(sensor::SensorType::Unspecified));
  initTranslated("sensor_subtype",
                 getSensorSubTypeName(sensor::SensorSubType::Unspecified));
}  // AbstractSensorAttributes ctor

void AbstractSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  setPosition(spec->position);
  setOrientation(spec->orientation);
  setNoiseModel(spec->noiseModel);
  setSensorTypeEnum(spec->sensorType);
  setSensorSubTypeEnum(spec->sensorSubType);
}  // AbstractSensorAttributes::populateWithSensorSpec

void AbstractSensorAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractSensorAttributes to JSON
  writeValueToJson("position", jsonObj, allocator);
  writeValueToJson("orientation", jsonObj, allocator);
  writeValueToJson("noise_model", jsonObj, allocator);
  writeValueToJson("sensor_type", jsonObj, allocator);
  writeValueToJson("sensor_subtype", jsonObj, allocator);

  // call child-class-specific
  writeValuesToJsonInternal(jsonObj, allocator);
}  // AbstractSensorAttributes::writeValuesToJson

std::string AbstractSensorAttributes::getObjectInfoHeaderInternal() const {
  return "Position XYZ,Orientation XYZ,Noise Model,Sensor "
         "Type,Sensor Subtype," +
         getAbstractSensorInfoHeaderInternal();
}  // AbstractSensorAttributes::getObjectInfoHeaderInternal

std::string AbstractSensorAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString("{},{},{},{},{},{}", getAsString("position"),
                                   getAsString("orientation"), getNoiseModel(),
                                   getSensorTypeName(getSensorType()),
                                   getSensorSubTypeName(getSensorSubType()),
                                   getAbstractSensorInfoInternal());
}  // AbstractSensorAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
