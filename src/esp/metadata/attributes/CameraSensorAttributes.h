
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_CAMERASENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_CAMERASENSORATTRIBUTES_H_

#include "AbstractVisualSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to suppoort creating camera sensors
 */
class CameraSensorAttributes : public AbstractVisualSensorAttributes {
 public:
  explicit CameraSensorAttributes(const std::string& handle = "");

  /**
   * @brief Populate this CameraSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;
  /**
   * @brief Set the horizontal Field of View (in degrees) for the CameraSensor
   * built by this attributes.
   */
  void setHFOV(Magnum::Deg hfov) { set("hfov", hfov); }
  /**
   * @brief Get the horizontal Field of View (in degrees) for the CameraSensor
   * built by this attributes.
   */
  Magnum::Deg getHFOV() const { return get<Magnum::Deg>("hfov"); }

  /**
   * @brief Set the orthographic scale for the CameraSensor built by this
   * attributes.
   */
  void setOrthoScale(float ortho_scale) { set("ortho_scale", ortho_scale); }

  /**
   * @brief Get the orthographic scale for the CameraSensor built by this
   * attributes.
   */
  float getOrthoScale() const {
    return static_cast<float>(get<double>("ortho_scale"));
  }

 protected:
  /**
   * @brief Write Camera Sensor-specific values to json object
   */
  void writeVisualSensorValuesToJsonInternal(
      io::JsonGenericValue& jsonObj,
      io::JsonAllocator& allocator) const override;

  /**
   * @brief get CameraSensorAttributes-specific info header
   */
  std::string getAbstractVisualSensorInfoHeaderInternal() const override;

  /**
   * @brief get CameraSensorAttributes specific info for csv string
   */
  std::string getAbstractVisualSensorInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(CameraSensorAttributes)
};  // class CameraSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_CAMERASENSORATTRIBUTES_H_
