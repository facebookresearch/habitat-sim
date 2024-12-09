// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_

#include "AbstractSensorAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief Class to suppoort creating visual sensors by providing common
 * attributes.
 */
class AbstractVisualSensorAttributes : public AbstractSensorAttributes {
 public:
  AbstractVisualSensorAttributes(const std::string& classKey,
                                 const std::string& handle);

  /**
   * @brief Populate this AbstractVisualSensorAttributes from an appropriate @ref sensor::SensorSpec.
   * @todo Remove when SensorSpecs are removed
   *
   */
  void populateWithSensorSpec(const sensor::SensorSpec::ptr& spec) override;

  /**
   * @brief Set the resolution (Height, Width) of the Visual Sensor built from
   * this attributes.
   */
  void setResolution(const Mn::Vector2i& resolution) {
    set("resolution", resolution);
  }

  /**
   * @brief Get the resolution (Height, Width) of the Visual Sensor built from
   * this attributes.
   */

  Mn::Vector2i getResolution() const { return get<Mn::Vector2i>("resolution"); }

  /**
   * @brief Set the number of channels for the Visual Sensor built from
   * this attributes.
   */
  void setChannels(int channels) { set("channels", channels); }

  /**
   * @brief get the number of channels for the Visual Sensor built from this
   * attributes.
   */
  int getChannels() const { return get<int>("channels"); }

  /**
   * @brief Set whether to enable gpu-to-gpu transfer for the sensor built
   * from this attributes.
   */
  void setGPUToGPUTransfer(bool transfer) {
    set("gpu_to_gpu_transfer", transfer);
  }

  /**
   * @brief get whether to enable gpu-to-gpu transfer for the sensor built
   * from this attributes.
   */
  bool getGPUToGPUTransfer() const { return get<bool>("gpu_to_gpu_transfer"); }

  /**
   * @brief Set the near plane distance for visual sensors built from this
   * attributes.
   */
  void setNearPlane(float near_plane) { set("near_plane", near_plane); }
  /**
   * @brief Get the near plane distance for visual sensors built from this
   * attributes.
   */
  float getNearPlane() const {
    return static_cast<float>(get<double>("near_plane"));
  }

  /**
   * @brief Set the far plane distance for visual sensors built from this
   * attributes.
   */
  void setFarPlane(float far_plane) { set("far_plane", far_plane); }

  /**
   * @brief Get the far plane distance for visual sensors built from this
   * attributes.
   */
  float getFarPlane() const {
    return static_cast<float>(get<double>("far_plane"));
  }

  /**
   * @brief Set the clear color to use for the visual sensor built from this
   * attributes.
   */

  void setClearColor(const Mn::Color4& clear_color) {
    set("clear_color", clear_color);
  }

  /**
   * @brief Get the clear color to use for the visual sensor built from this
   * attributes.
   */
  Mn::Color4 getClearColor() const { return get<Mn::Color4>("clear_color"); }

  /**
   * @brief Set the SemanticSensorTarget to use for a Semantic Sensor built
   * from this attributes.
   */
  void setSemanticSensorTarget(const std::string& semantic_target) {
    // force to lowercase before setting
    const std::string semanticTargetLC =
        Cr::Utility::String::lowercase(semantic_target);
    auto mapIter = SemanticSensorTargetMap.find(semanticTargetLC);
    ESP_CHECK(mapIter != SemanticSensorTargetMap.end(),
              "Illegal semantic sensor target value"
                  << semantic_target
                  << "attempted to be set in AbstractVisualSensorAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("semantic_sensor_target", semantic_target);
  }

  /**
   * @brief Set the SemanticSensorTarget to use for a Semantic Sensor built
   * from this attributes using the given @ref sensor::SemanticSensorTarget enum value.
   */
  void setSemanticSensorTargetEnum(
      sensor::SemanticSensorTarget semanticTargetEnum) {
    // force to lowercase before setting
    const std::string semanticTarget =
        getSemanitcSensorTargetName(semanticTargetEnum);
    auto mapIter = SemanticSensorTargetMap.find(semanticTarget);
    ESP_CHECK(mapIter != SemanticSensorTargetMap.end(),
              "Illegal Semantic Sensor Tyarget enum value given"
                  << static_cast<int>(semanticTargetEnum) << ":"
                  << semanticTarget
                  << "attempted to be set in AbstractVisualSensorAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("semantic_sensor_target", semanticTarget);
  }

  /**
   * @brief Get the SemanticSensorTarget to use for a Semantic Sensor built
   * from this attributes.
   */
  sensor::SemanticSensorTarget getSemanticSensorTarget() const {
    const std::string val = Cr::Utility::String::lowercase(
        get<std::string>("semantic_sensor_target"));
    auto mapIter = SemanticSensorTargetMap.find(val);
    if (mapIter != SemanticSensorTargetMap.end()) {
      return mapIter->second;
    }
    // Default to semantic ID
    return esp::sensor::SemanticSensorTarget::SemanticID;
  }

 protected:
  /**
   * @brief Write Visual Sensor-specific values to json object
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

  virtual void writeVisualSensorValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {};

  /**
   * @brief get AbstractSensorAttributes-specific info header
   */
  std::string getAbstractSensorInfoHeaderInternal() const override;

  /**
   * @brief get AbstractSensorAttributes specific info for csv string
   */
  std::string getAbstractSensorInfoInternal() const override;

  /**
   * @brief get AbstractVisualSensorAttributes-specific info header
   */
  virtual std::string getAbstractVisualSensorInfoHeaderInternal() const {
    return "";
  }

  /**
   * @brief get AbstractSensorAttributes specific info for csv string
   */
  virtual std::string getAbstractVisualSensorInfoInternal() const {
    return "";
  };

 public:
  ESP_SMART_POINTERS(AbstractVisualSensorAttributes)
};  // class VisualSensorAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ABSTRACTVISUALSENSORATTRIBUTES_H_
