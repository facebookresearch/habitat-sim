// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AudioSensorAttributes.h"
#include "esp/sensor/AudioSensor.h"

namespace esp {
namespace metadata {
namespace attributes {

AudioSensorAttributes::AudioSensorAttributes(const std::string& handle)
    : AbstractSensorAttributes("AudioSensorAttributes", handle) {
}  // AudioSensorAttributes ctor

void AudioSensorAttributes::populateWithSensorSpec(
    const std::shared_ptr<esp::sensor::SensorSpec>& spec) {
  AbstractSensorAttributes::populateWithSensorSpec(spec);
#ifdef ESP_BUILD_WITH_AUDIO
  // Appropriately cast to get audio data if exists
  const esp::sensor::AudioSensorSpec::ptr& audioSpec =
      std::dynamic_pointer_cast<esp::sensor::AudioSensorSpec>(spec);
  setOutputDirectory(audioSpec->outputDirectory_);
  setAcousticsConfig(audioSpec->acousticsConfig_);
  setChannelLayout(audioSpec->channelLayout_);

#endif  // ESP_BUILD_WITH_AUDIO
}  // AudioSensorAttributes::populateWithSensorSpec

void AudioSensorAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractObjectAttributes values to json
  writeValueToJson("output_directory", jsonObj, allocator);
  // TODO need some way to transform RLRAudioPropagation constructs to strings
}  // AudioSensorAttributes::writeValuesToJsonInternal

std::string AudioSensorAttributes::getAbstractSensorInfoHeaderInternal() const {
// TODO need some way to transform RLRAudioPropagation constructs to strings
#ifdef ESP_BUILD_WITH_AUDIO
  return "Output Directory";
#else
  return "";
#endif
}  // AudioSensorAttributes::getAbstractSensorInfoHeaderInternal

std::string AudioSensorAttributes::getAbstractSensorInfoInternal() const {
// TODO need some way to transform RLRAudioPropagation constructs to strings
#ifdef ESP_BUILD_WITH_AUDIO
  return Cr::Utility::formatString("{}", getOutputDirectory());
#else
  return "";
#endif
}  // AudioSensorAttributes::getAbstractSensorInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
