// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief attributes class describing essential and default quantities used to
 * instantiate an Articulated object
 */
class ArticulatedObjectAttributes : public AbstractAttributes {
 public:
  explicit ArticulatedObjectAttributes(const std::string& handle = "");
  /**
   * @brief Sets the string name for the Articulated Object URDF relative path
   */
  void setURDFPath(const std::string& urdfFilepath) {
    set("urdf_filepath", urdfFilepath);
  }
  /**
   * @brief Gets the string name for the Articulated Object URDF relative path
   */
  std::string getURDFPath() const { return get<std::string>("urdf_filepath"); }
  /**
   * @brief Sets the string name for the render asset relative path
   */
  void setRenderAssetHandle(const std::string& renderAsset) {
    set("render_asset", renderAsset);
  }
  /**
   * @brief Gets the string name for the render asset relative path
   */
  std::string getRenderAssetHandle() const {
    return get<std::string>("render_asset");
  }

  /**
   * @brief Set uniform scaling of the articulated object.
   */
  void setUniformScale(float scale) { set("uniform_scale", scale); }
  /**
   * @brief Get uniform scaling of the articulated object.
   */
  float getUniformScale() const {
    return static_cast<float>(get<double>("uniform_scale"));
  }

  /**
   * @brief Set mass scaling of the articulated object.
   */
  void setMassScale(double scale) { set("mass_scale", scale); }
  /**
   * @brief Get mass scaling of the articulated object.
   */
  double getMassScale() const { return get<double>("mass_scale"); }

  /**
   * @brief Set the type of base/root joint to use to add this Articulated
   * Object to the world. Cannot be "UNSPECIFIED"
   */
  void setBaseType(const std::string& baseType) {
    // force to lowercase to check if present
    const std::string baseTypeLC = Cr::Utility::String::lowercase(baseType);
    auto mapIter = AOBaseTypeMap.find(baseTypeLC);
    if ((mapIter == AOBaseTypeMap.end()) ||
        (mapIter->second == ArticulatedObjectBaseType::Unspecified)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "'" << baseType
          << "' is an illegal value for "
             "ArticulatedObjectAttributes::setBaseType, so default value "
          << get<std::string>("base_type") << " not changed.";
      return;
    }
    set("base_type", baseType);
  }

  /**
   * @brief Get the type of base/root joint to use to add this Articulated
   * Object to the world.
   */
  ArticulatedObjectBaseType getBaseType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("base_type"));
    auto mapIter = AOBaseTypeMap.find(val);
    if (mapIter != AOBaseTypeMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return ArticulatedObjectBaseType::Unspecified;
  }

  /**
   * @brief Set the source of the inertia tensors to use for this Articulated
   * Object.
   */
  void setInertiaSource(const std::string& inertiaSrc) {
    // force to lowercase before setting
    const std::string inertiaSrcLC = Cr::Utility::String::lowercase(inertiaSrc);
    auto mapIter = AOInertiaSourceMap.find(inertiaSrcLC);
    if ((mapIter == AOInertiaSourceMap.end()) ||
        (mapIter->second == ArticulatedObjectInertiaSource::Unspecified)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "'" << inertiaSrc
          << "' is an illegal value for "
             "ArticulatedObjectAttributes::setInertiaSource, so default value '"
          << get<std::string>("inertia_source") << "' not changed.";
      return;
    }
    set("inertia_source", inertiaSrc);
  }

  /**
   * @brief Get the source of the inertia tensors to use for this Articulated
   * Object.
   */
  ArticulatedObjectInertiaSource getInertiaSource() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("inertia_source"));
    auto mapIter = AOInertiaSourceMap.find(val);
    if (mapIter != AOInertiaSourceMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return ArticulatedObjectInertiaSource::Unspecified;
  }

  /**
   * @brief Set the link order to use for the linkages of this Articulated
   * Object
   */
  void setLinkOrder(const std::string& linkOrder) {
    // force to lowercase before setting
    const std::string linkOrderLC = Cr::Utility::String::lowercase(linkOrder);
    auto mapIter = AOLinkOrderMap.find(linkOrderLC);
    if ((mapIter == AOLinkOrderMap.end()) ||
        (mapIter->second == ArticulatedObjectLinkOrder::Unspecified)) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "'" << linkOrder
          << "' is an illegal value for "
             "ArticulatedObjectAttributes::setLinkOrder, so default value '"
          << get<std::string>("link_order") << "' not changed.";
      return;
    }
    set("link_order", linkOrder);
  }

  /**
   * @brief Get the link order to use for the linkages of this Articulated
   * Object
   */
  ArticulatedObjectLinkOrder getLinkOrder() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("link_order"));
    auto mapIter = AOLinkOrderMap.find(val);
    if (mapIter != AOLinkOrderMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return ArticulatedObjectLinkOrder::Unspecified;
  }

  /**
   * @brief Set the render mode to use to render this Articulated Object
   */
  void setRenderMode(const std::string& renderMode) {
    // force to lowercase before setting
    const std::string renderModeLC = Cr::Utility::String::lowercase(renderMode);
    auto mapIter = AORenderModesMap.find(renderModeLC);
    ESP_CHECK(mapIter != AORenderModesMap.end(),
              "Illegal render mode value"
                  << renderMode
                  << "attempted to be set in ArticulatedObjectAttributes:"
                  << getHandle() << ". Aborting.");
    set("render_mode", renderMode);
  }

  /**
   * @brief Get the render mode to use to render this Articulated Object
   */
  ArticulatedObjectRenderMode getRenderMode() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("render_mode"));
    auto mapIter = AORenderModesMap.find(val);
    if (mapIter != AORenderModesMap.end()) {
      return mapIter->second;
    }
    // This should never get to here. It would mean that this field was set
    // to an invalid value somehow.
    return ArticulatedObjectRenderMode::Unspecified;
  }

  /**
   * @brief Set the default shader to use for this Articulated Object.  This may
   * be overridden by a scene instance specification.
   */
  void setShaderType(const std::string& shader_type) {
    // force to lowercase before setting
    const std::string shaderTypeLC =
        Cr::Utility::String::lowercase(shader_type);
    auto mapIter = ShaderTypeNamesMap.find(shaderTypeLC);
    ESP_CHECK(mapIter != ShaderTypeNamesMap.end(),
              "Illegal shader_type value"
                  << shader_type
                  << "attempted to be set in ArticulatedObjectAttributes:"
                  << getHandle() << ". Aborting.");
    set("shader_type", shader_type);
  }

  /**
   * @brief Get the default shader to use for this Articulated Object.  This may
   * be overridden by a scene instance specification.
   */
  ObjectInstanceShaderType getShaderType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("shader_type"));
    auto mapIter = ShaderTypeNamesMap.find(val);
    if (mapIter != ShaderTypeNamesMap.end()) {
      return mapIter->second;
    }
    // Unspecified is default value - should never be returned since setter
    // verifies value
    return ObjectInstanceShaderType::Unspecified;
  }

  /**
   * @brief Sets the semantic ID for instanced Articulated Objects.
   */
  void setSemanticId(int semanticId) { set("semantic_id", semanticId); }

  /**
   * @brief Gets the semantic ID for instanced Articulated Objects.
   */
  uint32_t getSemanticId() const { return get<int>("semantic_id"); }

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * ArticulatedObjectAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(ArticulatedObjectAttributes)
};  // class ArticulatedObjectAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_ARTICULATEDOBJECTATTRIBUTES_H_
