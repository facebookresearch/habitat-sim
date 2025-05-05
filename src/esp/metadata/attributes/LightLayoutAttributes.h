// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_LIGHTLAYOUTATTRIBUTES_H_

#include "AbstractAttributes.h"
#include "esp/gfx/LightSetup.h"

#include <Corrade/Utility/String.h>

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
  void setType(const std::string& type) {
    // force to lowercase before setting
    const std::string lightType =
        Cr::Utility::String::lowercase(Cr::Containers::StringView{type});
    auto mapIter = LightTypeNamesMap.find(lightType);
    ESP_CHECK(mapIter != LightTypeNamesMap.end(),
              "Illegal type value"
                  << type << ":" << lightType
                  << "attempted to be set in LightInstanceAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("type", type);
  }

  /** @brief Get the type of the light */
  gfx::LightType getType() const {
    const std::string val = Cr::Utility::String::lowercase(
        Cr::Containers::StringView{get<std::string>("type")});
    auto mapIter = LightTypeNamesMap.find(val);
    if (mapIter != LightTypeNamesMap.end()) {
      return mapIter->second;
    }
    // point is default value - should never be returned since setter verifies
    // value
    return gfx::LightType::Point;
  }

  /**
   * @brief Set the string key to the gfx::LightPositionModel to use when
   * placing the light - whether the lights translation should be relative to
   * the camera, the global scene origin, or some object.
   */
  void setPositionModel(const std::string& position_model) {
    // force to lowercase before setting
    const std::string posModelLC = Cr::Utility::String::lowercase(
        Cr::Containers::StringView{position_model});
    auto mapIter = LightPositionNamesMap.find(posModelLC);
    ESP_CHECK(mapIter != LightPositionNamesMap.end(),
              "Illegal position_model value"
                  << position_model << ":" << posModelLC
                  << "attempted to be set in LightInstanceAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("position_model", position_model);
  }

  /**
   * @brief Get the @ref gfx::LightPositionModel to use when placing the
   * light - whether the lights translation should be relative to the camera,
   * the global scene origin, or some object.
   */
  gfx::LightPositionModel getPositionModel() const {
    const std::string val = Cr::Utility::String::lowercase(
        Cr::Containers::StringView{get<std::string>("position_model")});
    auto mapIter = LightPositionNamesMap.find(val);
    if (mapIter != LightPositionNamesMap.end()) {
      return mapIter->second;
    }
    // global is default value - should never be returned since setter verifies
    // value
    return gfx::LightPositionModel::Global;
  }

  /**
   * @brief Gets a smart pointer reference to a copy of the spotlight
   * configuration data for this @ref LightInstanceAttributes.
   */
  std::shared_ptr<Configuration> getSpotlightConfiguration() const {
    return getSubconfigCopy<Configuration>("spot");
  }

  /**
   * @brief Gets a smart pointer reference to the actual spotlight
   * configuration data for this @ref LightInstanceAttributes.
   */
  std::shared_ptr<Configuration> editSpotlightConfiguration() {
    return editSubconfig<Configuration>("spot");
  }

  /**
   * @brief Set inner cone angle for spotlights. This value resides in subconfig
   * 'spot'.  Should be ignored for other lights
   */
  void setInnerConeAngle(Magnum::Rad innerConeAngle) {
    editSubconfig<Configuration>("spot")->set("innerConeAngle", innerConeAngle);
  }

  /**
   * @brief Get inner cone angle for spotlights.  This value resides in
   * subconfig 'spot'.  Should be ignored for other lights
   */
  Magnum::Rad getInnerConeAngle() const {
    const auto spotCfg = getSubconfigCopy<Configuration>("spot");
    return spotCfg->get<Magnum::Rad>("innerConeAngle");
  }

  /**
   * @brief Set outer cone angle for spotlights. This value resides in subconfig
   * 'spot'.  Should be ignored for other lights
   */
  void setOuterConeAngle(Magnum::Rad outerConeAngle) {
    editSubconfig<Configuration>("spot")->set("outerConeAngle", outerConeAngle);
  }

  /**
   * @brief Get outer cone angle for spotlights. This value resides in subconfig
   * 'spot'.  Should be ignored for other lights
   */
  Magnum::Rad getOuterConeAngle() const {
    const auto spotCfg = getSubconfigCopy<Configuration>("spot");
    return spotCfg->get<Magnum::Rad>("outerConeAngle");
  }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * LightInstanceAttributes.
   */

  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
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
        getAsString("color"), getAsString("intensity"),
        getLightTypeName(getType()),
        getLightPositionModelName(getPositionModel()));
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
   * @brief Add a @ref LightInstanceAttributes to this lighting layout
   */
  void addLightInstance(LightInstanceAttributes::ptr _lightInstance) {
    this->setSubAttributesInternal<LightInstanceAttributes>(
        _lightInstance, availableLightIDs_, lightInstConfig_, "", false);
  }

  /**
   * @brief Remove a named @ref LightInstanceAttributes from this lighting layout
   */
  LightInstanceAttributes::ptr removeLightInstance(const std::string& handle) {
    return this->removeNamedSubAttributesInternal<LightInstanceAttributes>(
        handle, availableLightIDs_, lightInstConfig_);
  }

  /**
   * @brief Retrieve a reference to the named @ref LightInstanceAttributes
   */
  LightInstanceAttributes::cptr getLightInstance(const std::string& handle) {
    return getNamedSubAttributesInternal<LightInstanceAttributes>(
        handle, lightInstConfig_);
  }

  /**
   * @brief Get all the @ref LightInstanceAttributes for this layout
   */
  std::vector<LightInstanceAttributes::cptr> getLightInstances() const {
    return this->getSubAttributesListInternal<LightInstanceAttributes>(
        lightInstConfig_);
  }

  /**
   * @brief Return how many lights are in this light layout - number of
   * @ref LightInstanceAttributes in @ref lightInstConfig_ subconfig.
   */
  int getNumLightInstances() const {
    return this->getNumSubAttributesInternal("", lightInstConfig_);
  }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * LightLayoutAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

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
   * @brief Smartpointer to created @ref LightInstanceAttributes configuration. The
   * configuration is created on LightLayoutAttributes construction.
   */
  std::shared_ptr<Configuration> lightInstConfig_{};

  /**
   * @brief Deque holding all released IDs to consume for @ref LightInstanceAttributes when
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
