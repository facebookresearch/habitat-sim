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
 * its template name, location/direction, color, intensity, type and other
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

  /** @brief Set the position of the light. Used for positional lights.  */
  void setPosition(const Magnum::Vector3& position) {
    set("position", position);
  }

  /** @brief Get the position of the light. Used for positional lights. */
  Magnum::Vector3 getPosition() const {
    return get<Magnum::Vector3>("position");
  }

  /** @brief Set the direction of the light. Used for directional lights. */
  void setDirection(const Magnum::Vector3& direction) {
    set("direction", direction);
  }

  /** @brief Get the direction of the light. Used for directional lights. */
  Magnum::Vector3 getDirection() const {
    return get<Magnum::Vector3>("direction");
  }

  /** @brief Set the color of the light.*/
  void setColor(const Magnum::Vector3& color) { set("color", color); }

  /** @brief Get the color of the light.*/
  Magnum::Vector3 getColor() const { return get<Magnum::Vector3>("color"); }

  /** @brief Set the intensity of the light. */
  void setIntensity(double intensity) { set("intensity", intensity); }

  /** @brief Get the intensity of the light. */
  double getIntensity() const { return get<double>("intensity"); }

  /** @brief Set the type of the light */
  void setType(int type) { set("type", type); }

  /** @brief Get the type of the light */
  int getType() const { return get<int>("type"); }

  /**
   * @brief Set the string key to the esp::gfx::LightPositionModel to use when
   * placing the light - whether the lights translation should be relative to
   * the camera, the global scene origin, or some object.
   */
  void setPositionModel(const std::string& position_model) {
    // force to lowercase before setting
    const std::string posModelLC =
        Cr::Utility::String::lowercase(position_model);
    auto mapIter = LightPositionNamesMap.find(posModelLC);
    if (mapIter != LightPositionNamesMap.end()) {
      set("position_model", posModelLC);
    } else {
      set("position_model", "global");
    }
  }

  /**
   * @brief Get the @ref esp::gfx::LightPositionModel to use when placing the
   * light - whether the lights translation should be relative to the camera,
   * the global scene origin, or some object.
   */
  esp::gfx::LightPositionModel getPositionModel() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("position_model"));
    auto mapIter = LightPositionNamesMap.find(val);
    if (mapIter != LightPositionNamesMap.end()) {
      return mapIter->second;
    }
    // global is default value
    return LightPositionNamesMap.at("global");
  }

  /**
   * @brief Set inner cone angle for spotlights.  Should be ignored for
   * other lights
   */
  void setInnerConeAngle(Magnum::Rad innerConeAngle) {
    set("innerConeAngle", innerConeAngle);
  }

  /**
   * @brief Get inner cone angle for spotlights.  Should be ignored for
   * other lights
   */
  Magnum::Rad getInnerConeAngle() const {
    return get<Magnum::Rad>("innerConeAngle");
  }

  /**
   * @brief Set outer cone angle for spotlights. Should be ignored for other
   * lights
   */
  void setOuterConeAngle(Magnum::Rad outerConeAngle) {
    set("outerConeAngle", outerConeAngle);
  }

  /**
   * @brief Get outer cone angle for spotlights. Should be ignored for other
   * lights
   */
  Magnum::Rad getOuterConeAngle() const {
    return get<Magnum::Rad>("outerConeAngle");
  }

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
    esp::gfx::LightPositionModel type = getPositionModel();
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
    return "Position XYZ,Direction XYZ,Color RGB,Intensity,Light Type,"
           "Light Position Model,";
  }

  /**
   * @brief Retrieve a comma-separated informational string about the
   * contents of this managed object.
   * TODO : once Magnum supports retrieving key-values of configurations,
   * use that to build this data.
   */
  std::string getObjectInfoInternal() const override {
    return Cr::Utility::formatString(
        "{},{},{},{},{},{},", getAsString("position"), getAsString("direction"),
        getAsString("color"), getAsString("intensity"), getCurrLightTypeName(),
        getCurrLightPositionModelName());
  }

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

  LightLayoutAttributes(const LightLayoutAttributes& otr);
  LightLayoutAttributes(LightLayoutAttributes&& otr) noexcept;

  LightLayoutAttributes& operator=(const LightLayoutAttributes& otr);
  LightLayoutAttributes& operator=(LightLayoutAttributes&& otr) noexcept;

  /**
   * @brief Set a scale of all positive intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  void setPositiveIntensityScale(double positive_intensity_scale) {
    set("positive_intensity_scale", positive_intensity_scale);
  }
  /**
   * @brief Get a scale of all positive intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  double getPositiveIntensityScale() const {
    return get<double>("positive_intensity_scale");
  }

  /**
   * @brief Set a scale of all negative intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  void setNegativeIntensityScale(double negative_intensity_scale) {
    set("negative_intensity_scale", negative_intensity_scale);
  }
  /**
   * @brief Get a scale of all negative intensities by specified amount.
   * This is to make simple, sweeping adjustments to scene lighting in habitat.
   */
  double getNegativeIntensityScale() const {
    return get<double>("negative_intensity_scale");
  }

  /**
   * @brief Add a light instance to this lighting layout
   */
  void addLightInstance(LightInstanceAttributes::ptr _lightInstance) {
    this->setSubAttributesInternal<LightInstanceAttributes>(
        _lightInstance, availableLightIDs_, lightInstConfig_, "");
  }

  /**
   * @brief Remove a light from this lighting layout
   */
  LightInstanceAttributes::ptr removeLightInstance(const std::string& handle) {
    return this->removeNamedSubAttributesInternal<LightInstanceAttributes>(
        handle, availableLightIDs_, lightInstConfig_);
  }

  LightInstanceAttributes::cptr getLightInstance(const std::string& handle) {
    return getNamedSubAttributesInternal<LightInstanceAttributes>(
        handle, lightInstConfig_);
  }

  /**
   * @brief Get the lighting instances for this layout
   */
  std::vector<LightInstanceAttributes::cptr> getLightInstances() const {
    return this->getSubAttributesListInternal<LightInstanceAttributes>(
        lightInstConfig_);
  }

  /**
   * @brief Return how many lights are in this light layout - number of
   * subconfigs in @ref lightInstConfig_ subconfig.
   */
  int getNumLightInstances() const {
    return this->getNumSubAttributesInternal("", lightInstConfig_);
  }

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
   * @brief Smartpointer to created light instance configuration. The
   * configuration is created on LightLayoutAttributes construction.
   */
  std::shared_ptr<Configuration> lightInstConfig_{};

  /**
   * @brief Deque holding all released IDs to consume for light instances when
   * one is deleted, before using size of lightInstances_ container.
   */
  std::deque<int> availableLightIDs_;

 public:
  ESP_SMART_POINTERS(LightLayoutAttributes)
};  // class LightLayoutAttribute
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_
