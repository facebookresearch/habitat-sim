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

  /**
   * @brief Constant static map to provide mappings from string tags to types of
   * lights.  This will be used to map values set in json to types.  Keys (json
   * values) must be lowercase.
   */
  // static const std::map<std::string, (put enum here)> LightTypeNamesMap;

  LightAttributes(const std::string& handle = "");

  /**
   * @brief Get/Set the position of the light.
   */
  void setPosition(const Magnum::Vector3& position) {
    setVec3("position", position);
  }
  Magnum::Vector3 getPosition() const { return getVec3("position"); }

  /**
   * @brief Get/Set the direction of the light.
   */
  void setDirection(const Magnum::Vector3& direction) {
    setVec3("direction", direction);
  }
  Magnum::Vector3 getDirection() const { return getVec3("direction"); }

  /**
   * @brief Get/Set the color of the light.
   */
  void setColor(const Magnum::Vector3& color) { setVec3("color", color); }
  Magnum::Vector3 getColor() const { return getVec3("color"); }

  /**
   * @brief Get/Set the color scale of the light.
   */
  void setIntensity(double intensity) { setDouble("intensity", intensity); }
  double getIntensity() const { return getDouble("intensity"); }

  /**
   * @brief Get/Set the type of the light
   */
  void setType(const std::string& type) { setString("type", type); }
  std::string getType() const { return getString("type"); }

  /**
   * @brief Get/Set inner cone angle for spotlights.  Should be ignored for
   * other lights
   */
  void setInnerConeAngle(double innerConeAngle) {
    setDouble("innerConeAngle", innerConeAngle);
  }
  double getInnerConeAngle() const { return getDouble("innerConeAngle"); }

  /**
   * @brief Get/Set inner cone angle for spotlights. Should be ignored for other
   * lights
   */
  void setOuterConeAngle(double outerConeAngle) {
    setDouble("outerConeAngle", outerConeAngle);
  }
  double getOuterConeAngle() const { return getDouble("outerConeAngle"); }

 public:
  ESP_SMART_POINTERS(LightAttributes)

};  // class LightAttributes
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_LIGHTATTRIBUTES_H_
