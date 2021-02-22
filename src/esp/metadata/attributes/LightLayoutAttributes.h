// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_

#include "AttributesBase.h"
#include "esp/gfx/LightSetup.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief This class describes an instance of a light -
 * it's template name, location/direction, color, intensity, type and other
 * parameters if appropriate.
 */
class LightInstanceAttributes : public AbstractAttributes {
 public:
  /**
   * @brief Constant static map to provide mappings from string tags to @ref
   * esp::gfx::LightType values.  This will be used to map values set in json
   * for light type to @ref esp::gfx::LightType.  Keys must be lowercase.
   */
  static const std::map<std::string, esp::gfx::LightType> LightTypeNamesMap;
  explicit LightInstanceAttributes(const std::string& handle = "");

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
  void setType(int type) { setInt("type", type); }
  int getType() const { return getInt("type"); }

  /**
   * @brief Get/Set inner cone angle for spotlights.  Should be ignored for
   * other lights
   */
  void setInnerConeAngle(Magnum::Rad innerConeAngle) {
    setRad("innerConeAngle", innerConeAngle);
  }
  Magnum::Rad getInnerConeAngle() const { return getRad("innerConeAngle"); }

  /**
   * @brief Get/Set inner cone angle for spotlights. Should be ignored for other
   * lights
   */
  void setOuterConeAngle(Magnum::Rad outerConeAngle) {
    setRad("outerConeAngle", outerConeAngle);
  }
  Magnum::Rad getOuterConeAngle() const { return getRad("outerConeAngle"); }

 public:
  ESP_SMART_POINTERS(LightInstanceAttributes)

};  // class LightInstanceAttributes

/**
 * @brief This class describes a lighting layout, consisting of a series of
 * lights.
 */
class LightLayoutAttributes : public AbstractAttributes {
 public:
  explicit LightLayoutAttributes(const std::string& handle = "");

  /**
   * @brief Add a light instance to this lighting layout
   */
  void addLightInstance(const LightInstanceAttributes::ptr& _lightInstance) {
    lightInstances_.emplace(_lightInstance->getHandle(), _lightInstance);
  }

  /**
   * @brief Remove a light from this lighting layout
   */
  LightInstanceAttributes::ptr removeLightInstance(const std::string& handle) {
    auto inst = getLightInstance(handle);
    if (nullptr != inst) {
      lightInstances_.erase(handle);
    }
    return inst;
  }

  LightInstanceAttributes::ptr getLightInstance(const std::string& handle) {
    if (lightInstances_.count(handle) == 0) {
      return nullptr;
    }
    auto inst = lightInstances_.at(handle);
    return inst;
  }

  /**
   * @brief Get the lighting instances for this layout
   */
  const std::map<std::string, LightInstanceAttributes::ptr>& getLightInstances()
      const {
    return lightInstances_;
  }

  /**
   * @brief Return how many lights are in this light layout
   */
  int getNumLightInstances() { return lightInstances_.size(); }

 protected:
  /**
   * @brief The light instances used by this lighting layout
   */
  std::map<std::string, LightInstanceAttributes::ptr> lightInstances_;

 public:
  ESP_SMART_POINTERS(LightLayoutAttributes)
};  // namespace attributes
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_
