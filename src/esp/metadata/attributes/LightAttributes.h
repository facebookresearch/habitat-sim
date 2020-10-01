// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_LIGHTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_LIGHTATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

class LightAttributes : public AbstractAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref LightAttributes.
   * Has values that are different than defaults so this can be used to test
   * json loading. These values are set to be purposefully weird/invalid, for
   * testing purposes, and so should not be used for an actual light.
   */
  static const std::string JSONConfigTestString;

  LightAttributes(const std::string& handle = "");

  /**
   * @brief Set the position of the light.
   */
  void setPosition(const Magnum::Vector3& position) {
    setVec3("position", position);
  }
  Magnum::Vector3 getPosition() const { return getVec3("position"); }

  /**
   * @brief Set the color of the light.
   */
  void setColor(const Magnum::Vector3& color) { setVec3("color", color); }
  Magnum::Vector3 getColor() const { return getVec3("color"); }

  /**
   * @brief Set the color scale of the light.
   */
  void setColorScale(double colorScale) {
    setDouble("color_scale", colorScale);
  }
  double getColorScale() const { return getDouble("color_scale"); }

 public:
  ESP_SMART_POINTERS(LightAttributes)

};  // class LightAttributes
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_LIGHTATTRIBUTES_H_
