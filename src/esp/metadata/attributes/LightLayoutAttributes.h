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
   * for light type to @ref esp::gfx::LightType.  Keys must be lowercase - will
   * support any case values in JSON.
   */
  static const std::map<std::string, esp::gfx::LightType> LightTypeNamesMap;

  /**
   * @brief Constant static map to provide mappings from string tags to @ref
   * esp::gfx::LightPositionModel values.  This will be used to map values set
   * in json to specify what translations are measured from for a lighting
   * instance.
   */
  static const std::map<std::string, esp::gfx::LightPositionModel>
      LightPositionNamesMap;

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
   * @brief Get/Set the position model to use when placing the light - whether
   * the lights translation should be relative to the camera, the global scene
   * origin, or some object.
   */
  void setPositionModel(int position_model) {
    setInt("position_model", position_model);
  }
  int getPositionModel() const { return getInt("position_model"); }

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

 protected:
  /**
   * @brief Used for info purposes.  Return a string name corresponding to the
   * currently specified light type value;
   */
  std::string getCurrLightTypeName() const {
    // Must always be valid value
    esp::gfx::LightType type = static_cast<esp::gfx::LightType>(getType());
    for (const auto& it : LightTypeNamesMap) {
      if (it.second == type) {
        return it.first;
      }
    }
    return "unknown light type";
  }

  std::string getCurrLightPositionModelName() const {
    // Must always be valid value
    esp::gfx::LightPositionModel type =
        static_cast<esp::gfx::LightPositionModel>(getPositionModel());
    for (const auto& it : LightPositionNamesMap) {
      if (it.second == type) {
        return it.first;
      }
    }
    return "unknown position model";
  }
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override {
    return "Position XYZ, Direction XYZ, Color RGB, Intensity, Light Type, "
           "Light Position Model,";
  }
  /**
   * @brief Retrieve a comma-separated informational string about the
   * contents of this managed object.
   * TODO : once Magnum supports retrieving key-values of configurations,
   * use that to build this data.
   */
  std::string getObjectInfoInternal() const override {
    return cfg.value("position")
        .append(1, ',')
        .append(cfg.value("direction"))
        .append(1, ',')
        .append(cfg.value("color"))
        .append(1, ',')
        .append(cfg.value("intensity"))
        .append(1, ',')
        .append(getCurrLightTypeName())
        .append(1, ',')
        .append(getCurrLightPositionModelName())
        .append(1, ',');
  }

 protected:
  static int _count;

 public:
  ESP_SMART_POINTERS(LightInstanceAttributes)

};  // namespace attributes

/**
 * @brief This class describes a lighting layout, consisting of a series of
 * lights.
 */
class LightLayoutAttributes : public AbstractAttributes {
 public:
  explicit LightLayoutAttributes(const std::string& handle = "");

  /**
   * @brief Set a scale of all positive intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  void setPositiveIntensityScale(double positive_intensity_scale) {
    setDouble("positive_intensity_scale", positive_intensity_scale);
  }
  /**
   * @brief Get a scale of all positive intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  double getPositiveIntensityScale() const {
    return getDouble("positive_intensity_scale");
  }

  /**
   * @brief Set a scale of all negative intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  void setNegativeIntensityScale(double negative_intensity_scale) {
    setDouble("negative_intensity_scale", negative_intensity_scale);
  }
  /**
   * @brief Get a scale of all negative intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  double getNegativeIntensityScale() const {
    return getDouble("negative_intensity_scale");
  }

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
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific. The individual light
   * instances return a header for this.
   */

  std::string getObjectInfoHeaderInternal() const override { return ","; };
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

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
