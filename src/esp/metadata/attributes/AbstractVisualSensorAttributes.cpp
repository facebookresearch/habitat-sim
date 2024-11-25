// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AbstractVisualSensorAttributes.h"
#include "esp/sensor/VisualSensor.h"

namespace esp {
namespace metadata {
namespace attributes {

AbstractVisualSensorAttributes::AbstractVisualSensorAttributes(
    const std::string& classKey,
    const std::string& handle)
    : AbstractSensorAttributes(classKey, handle) {
  init("resolution", Mn::Vector2i{128, 128});
  init("channels", 4);
  init("gpu_to_gpu_transfer", false);
  init("near_plane", 0.01);
  init("far_plane", 1000.0);
  init("clear_color", Mn::Color4{0, 0, 0, 1});
  initTranslated("semantic_sensor_target",
                 getSemanitcSensorTargetName(
                     esp::sensor::SemanticSensorTarget::SemanticID));

}  // AbstractVisualSensorAttributes ctor

void AbstractVisualSensorAttributes::populateWithSensorSpec(
    const sensor::SensorSpec::ptr& spec) {
  // Call Base class version
  AbstractSensorAttributes::populateWithSensorSpec(spec);
  // Appropriately cast to get visual spec data if exists
  const esp::sensor::VisualSensorSpec::ptr& visualSpec =
      std::dynamic_pointer_cast<esp::sensor::VisualSensorSpec>(spec);

  setResolution(visualSpec->resolution);
  setChannels(visualSpec->channels);
  setGPUToGPUTransfer(visualSpec->gpu2gpuTransfer);
  setNearPlane(visualSpec->near);
  setFarPlane(visualSpec->far);
  setClearColor(visualSpec->clearColor);
  setSemanticSensorTargetEnum(visualSpec->semanticTarget);

}  // AbstractVisualSensorAttributes::populateWithSensorSpec

void AbstractVisualSensorAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractVisualSensorAttributes values to json
  writeValueToJson("resolution", jsonObj, allocator);
  writeValueToJson("channels", jsonObj, allocator);
  writeValueToJson("gpu_to_gpu_transfer", jsonObj, allocator);
  writeValueToJson("near_plane", jsonObj, allocator);
  writeValueToJson("far_plane", jsonObj, allocator);
  writeValueToJson("clear_color", jsonObj, allocator);
  writeValueToJson("semantic_sensor_target", jsonObj, allocator);
  // call child-class-specific
  writeVisualSensorValuesToJsonInternal(jsonObj, allocator);
}  // AbstractVisualSensorAttributes::writeValuesToJsonInternal

std::string
AbstractVisualSensorAttributes::getAbstractSensorInfoHeaderInternal() const {
  return "Resolution YX,Channels,GPU to GPU Transfer,Near Plane,Far "
         "Plane,Clear Color RGBA,Semantic Sensor Target," +
         getAbstractVisualSensorInfoHeaderInternal();
}  // AbstractVisualSensorAttributes::getAbstractSensorInfoHeaderInternal

std::string AbstractVisualSensorAttributes::getAbstractSensorInfoInternal()
    const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getAsString("resolution"),
      getAsString("channels"), getAsString("gpu_to_gpu_transfer"),
      getAsString("near_plane"), getAsString("far_plane"),
      getAsString("clear_color"),
      getSemanitcSensorTargetName(getSemanticSensorTarget()),
      getAbstractVisualSensorInfoInternal());
}  // AbstractVisualSensorAttributes::getAbstractSensorInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
