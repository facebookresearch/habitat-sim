// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AudioSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

AudioSensorAttributes::AudioSensorAttributes(const std::string& handle)
    : AbstractSensorAttributes("AudioSensorAttributes", handle) {
}  // AudioSensorAttributes ctor

void AudioSensorAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // write AbstractObjectAttributes values to json
  writeValueToJson("output_directory", jsonObj, allocator);
  // TODO need some way to transform RLRAudioPropagation constructs to strings
}  // AudioSensorAttributes::writeValuesToJsonInternal

std::string AudioSensorAttributes::getAbstractSensorInfoHeaderInternal() const {
  // TODO need some way to transform RLRAudioPropagation constructs to strings
  return "Output Directory";
}  // AudioSensorAttributes::getAbstractSensorInfoHeaderInternal

std::string AudioSensorAttributes::getAbstractSensorInfoInternal() const {
  // TODO need some way to transform RLRAudioPropagation constructs to strings
  return Cr::Utility::formatString("{}", getOutputDirectory());
}  // AudioSensorAttributes::getAbstractSensorInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
