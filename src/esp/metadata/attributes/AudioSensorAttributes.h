// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_

#include "AbstractSensorAttributes.h"

#ifdef ESP_BUILD_WITH_AUDIO
#include "rlr-audio-propagation/RLRAudioPropagationPkg/headers/RLRAudioPropagation.h"
#endif  // ESP_BUILD_WITH_AUDIO

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to support creating audio sensors
 */
class AudioSensorAttributes : public AbstractSensorAttributes {
 public:
  explicit AudioSensorAttributes(const std::string& handle = "");

  /**
   * @brief Populate this audio attributes from an appropriate @ref sensor::AudioSensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(
      const std::shared_ptr<esp::sensor::SensorSpec>& spec) override;

#ifdef ESP_BUILD_WITH_AUDIO
 private:
  RLRAudioPropagation::Configuration acousticsConfig_;
  RLRAudioPropagation::ChannelLayout channelLayout_;

 public:
  /**
   * @brief Set the output directory
   */
  void setOutputDirectory(const std::string& output_directory) {
    set("output_directory", output_directory);
  }

  /**
   * @brief Get the output directory
   */
  std::string getOutputDirectory() const {
    return get<std::string>("output_directory");
  }

  /**
   * @brief Sets the acoustics configuration.
   */
  void setAcousticsConfig(
      const RLRAudioPropagation::Configuration& acousticsConfig) {
    acousticsConfig_ = acousticsConfig;
  }
  /**
   * @brief Gets the acoustics configuration.
   */
  RLRAudioPropagation::Configuration getAcousticsConfig() const {
    return acousticsConfig_;
  }
  /**
   * @brief Sets the acoustic channel layout.
   */
  void setChannelLayout(
      const RLRAudioPropagation::ChannelLayout& channelLayout) {
    channelLayout_ = channelLayout;
  }
  /**
   * @brief Gets the acoustic channel layout.
   */
  RLRAudioPropagation::ChannelLayout getChannelLayout() const {
    return channelLayout_;
  }

#endif  // ESP_BUILD_WITH_AUDIO

 protected:
  /**
   * @brief Write Audio Sensor-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  /**
   * @brief get AbstractSensorAttributes-specific info header
   */
  std::string getAbstractSensorInfoHeaderInternal() const override;
  /**
   * @brief get AbstractObject specific info for csv string
   */
  std::string getAbstractSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(AudioSensorAttributes)
};  // class AudioSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_AUDIOSENSORATTRIBUTES_H_
