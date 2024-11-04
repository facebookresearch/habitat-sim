// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SCENEINSTANCEATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SCENEINSTANCEATTRIBUTES_H_

#include <deque>
#include <utility>

#include "AbstractAttributes.h"

namespace esp {
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace managers {
class SceneInstanceAttributesManager;
enum class SceneInstanceTranslationOrigin;
}  // namespace managers
namespace attributes {

class AbstractObjectAttributes;
class ArticulatedObjectAttributes;
class ObjectAttributes;
class StageAttributes;

/**
 * @brief This class describes an instance of a stage, object or articulated
 * object in a scene - its template name, translation from the origin, rotation,
 * motiontype, and other values required to instantiate the construct described.
 */
class SceneObjectInstanceAttributes : public AbstractAttributes {
 public:
  /**
   * @brief SceneObjectInstanceAttributes handle is also the handle of the
   * underlying @ref AbstractObjectAttributes for the object being instanced.
   */
  explicit SceneObjectInstanceAttributes(
      const std::string& handle,
      const std::string& type = "SceneObjectInstanceAttributes");

  /**
   * @brief Set the translation from the origin of the described
   * stage/object instance.
   */
  void setTranslation(const Mn::Vector3& translation) {
    set("translation", translation);
  }
  /**
   * @brief Get the translation from the origin of the described
   * stage/object instance.
   */
  Mn::Vector3 getTranslation() const { return get<Mn::Vector3>("translation"); }

  /**
   * @brief Set a value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  void setTranslationOrigin(const std::string& translation_origin) {
    // force to lowercase before setting
    const std::string transOriginLC =
        Cr::Utility::String::lowercase(translation_origin);
    auto mapIter = InstanceTranslationOriginMap.find(transOriginLC);

    ESP_CHECK((mapIter != InstanceTranslationOriginMap.end() ||
               (transOriginLC == getTranslationOriginName(
                                     SceneInstanceTranslationOrigin::Unknown))),
              "Illegal translation_origin value"
                  << translation_origin
                  << "attempted to be set in SceneObjectInstanceAttributes :"
                  << getHandle() << ". Aborting.");
    setTranslated("translation_origin", translation_origin);
  }

  /**
   * @brief Get the value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  SceneInstanceTranslationOrigin getTranslationOrigin() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("translation_origin"));
    auto mapIter = InstanceTranslationOriginMap.find(val);
    if (mapIter != InstanceTranslationOriginMap.end()) {
      return mapIter->second;
    }
    // Unknown is default value
    return SceneInstanceTranslationOrigin::Unknown;
  }

  /**
   * @brief Get string representation of translation origin
   */
  std::string getTranslationOriginStr() const {
    return get<std::string>("translation_origin");
  }

  /**
   * @brief Set the rotation of the object
   */
  void setRotation(const Mn::Quaternion& rotation) {
    set("rotation", rotation);
  }
  /**
   * @brief Get the rotation of the object
   */
  Mn::Quaternion getRotation() const { return get<Mn::Quaternion>("rotation"); }

  /**
   * @brief If not visible can add dynamic non-rendered object into a scene
   * object.  If is not visible then should not add object to drawables.
   */
  void setIsInstanceVisible(bool isVisible) {
    // needs to be int to cover "no specification"
    set("is_instance_visible", (isVisible ? 1 : 0));
  }
  int getIsInstanceVisible() const { return get<int>("is_instance_visible"); }
  void clearIsInstanceVisible() { set("is_instance_visible", ID_UNDEFINED); }

  /**
   * @brief Set the motion type for the object.  Ignored for stage instances.
   */
  void setMotionType(const std::string& motionType);

  /**
   * @brief Get the motion type for the object.  Ignored for stage instances.
   */
  esp::physics::MotionType getMotionType() const;

  /**
   * @brief Set the default shader to use for an object or stage.  Uses values
   * specified in stage or object attributes if not overridden here.  Uses map
   * of string values in JSON to @ref
   * esp::metadata::attributes::ObjectInstanceShaderType int values.
   */
  void setShaderType(const std::string& shader_type) {
    // force to lowercase before setting
    const std::string shaderTypeLC =
        Cr::Utility::String::lowercase(shader_type);
    auto mapIter = ShaderTypeNamesMap.find(shaderTypeLC);

    ESP_CHECK((mapIter != ShaderTypeNamesMap.end() ||
               (shaderTypeLC ==
                getShaderTypeName(ObjectInstanceShaderType::Unspecified))),
              "Illegal shader_type value"
                  << shader_type
                  << "attempted to be set in SceneObjectInstanceAttributes :"
                  << getHandle() << ". Aborting.");

    setTranslated("shader_type", shader_type);
  }

  /**
   * @brief Get the default shader to use for an object or stage.  This may be
   * overriding a stage or object config specification.
   */
  ObjectInstanceShaderType getShaderType() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("shader_type"));
    auto mapIter = ShaderTypeNamesMap.find(val);
    if (mapIter != ShaderTypeNamesMap.end()) {
      return mapIter->second;
    }
    // Unspecified is default value
    return ObjectInstanceShaderType::Unspecified;
  }

  /**
   * @brief Get the uniform scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  float getUniformScale() const {
    return static_cast<float>(get<double>("uniform_scale"));
  }

  /**
   * @brief Set the uniform scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  void setUniformScale(double uniform_scale) {
    set("uniform_scale", uniform_scale);
  }

  /**
   * @brief Get the non-uniform scale vector of the described stage/object
   * instance.
   */
  Mn::Vector3 getNonUniformScale() const {
    return get<Mn::Vector3>("non_uniform_scale");
  }

  /**
   * @brief Set the non-uniform scale vector of the described stage/object
   * instance.
   */
  void setNonUniformScale(const Mn::Vector3& non_uniform_scale) {
    set("non_uniform_scale", non_uniform_scale);
  }

  /**
   * @brief Object and Articulated Object Instance only. Get whether or
   * not to apply geometric scaling to mass (i.e. to maintain object mean
   * density)
   */
  bool getApplyScaleToMass() const { return get<bool>("apply_scale_to_mass"); }

  /**
   * @brief Object and Articulated Object Instance only. Set whether or
   * not to apply geometric scaling to mass (i.e. to maintain object mean
   * density)
   */
  void setApplyScaleToMass(bool apply_scale_to_mass) {
    set("apply_scale_to_mass", apply_scale_to_mass);
  }

  /**
   * @brief Get or set the mass scaling of the instanced object.
   */
  double getMassScale() const { return get<double>("mass_scale"); }
  void setMassScale(double mass_scale) { set("mass_scale", mass_scale); }

  /**
   * @brief Populate a JSON object with all the first-level values held in this
   * SceneObjectInstanceAttributes.  Default is overridden to handle special
   * cases for SceneObjectInstanceAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object.
   */
  std::string getObjectInfoHeader() const override {
    return Cr::Utility::formatString("Template Name, ID, {}",
                                     getObjectInfoHeaderInternal());
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfo() const override {
    return Cr::Utility::formatString(
        "{},{},{}", getHandle(), getAsString("__ID"), getObjectInfoInternal());
  }

 protected:
  friend class esp::metadata::managers::SceneInstanceAttributesManager;
  /**
   * @brief Called only from SceneInstanceAttributesManager, to create and
   * initialize an instance attributes from an existing object
   * attributes.
   *
   * Called internally. SceneObjectInstanceAttributes handle is also the
   * handle of the underlying @ref AbstractObjectAttributes for the object being
   * instanced. This initializes the instance attributes with the values from
   * the passed object attributes.
   */
  explicit SceneObjectInstanceAttributes(
      const std::string& handle,
      const std::shared_ptr<AbstractObjectAttributes>& baseObjAttribs);

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  virtual std::string getSceneObjInstanceInfoInternal() const { return ""; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  virtual std::string getSceneObjInstanceInfoHeaderInternal() const {
    return "";
  }

  /**
   * @brief Populate the passed JSON object with all the first-level values
   * specific to this SceneObjectInstanceAttributes. This is to facilitate
   * SceneAOInstanceAttributes-specific values to be written.
   */
  virtual void writeValuesToJsonInternal(
      CORRADE_UNUSED io::JsonGenericValue& jsonObj,
      CORRADE_UNUSED io::JsonAllocator& allocator) const {}

 public:
  ESP_SMART_POINTERS(SceneObjectInstanceAttributes)
};  // class SceneObjectInstanceAttributes

/**
 * @brief This class describes an instance of an articulated object in a scene
 * - along with its template name, translation from the origin, rotation,
 * motiontype, and other values inherited from SceneObjectInstanceAttributes,
 * it also holds initial joint pose and joint velocities.
 */
class SceneAOInstanceAttributes : public SceneObjectInstanceAttributes {
 public:
  /**
   * @brief SceneAOInstanceAttributes handle is also the handle of the
   * underlying @ref ArticulatedObjectAttributes for the object being instanced.
   */
  explicit SceneAOInstanceAttributes(const std::string& handle);

  /**
   * @brief Set the type of base/root joint to use to add this Articulated
   * Object to the world.
   */
  void setBaseType(const std::string& baseType) {
    // force to lowercase before setting
    const std::string baseTypeLC = Cr::Utility::String::lowercase(baseType);
    auto mapIter = AOBaseTypeMap.find(baseTypeLC);
    ESP_CHECK(mapIter != AOBaseTypeMap.end(),
              "Illegal base type value"
                  << baseType
                  << "attempted to be set in ArticulatedObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("base_type", baseType);
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
    const std::string renderModeLC = Cr::Utility::String::lowercase(inertiaSrc);
    auto mapIter = AOInertiaSourceMap.find(renderModeLC);
    ESP_CHECK(mapIter != AOInertiaSourceMap.end(),
              "Illegal inertia source value"
                  << inertiaSrc
                  << "attempted to be set in ArticulatedObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("inertia_source", inertiaSrc);
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
    const std::string renderModeLC = Cr::Utility::String::lowercase(linkOrder);
    auto mapIter = AOLinkOrderMap.find(renderModeLC);
    ESP_CHECK(mapIter != AOLinkOrderMap.end(),
              "Illegal link order value"
                  << linkOrder
                  << "attempted to be set in ArticulatedObjectAttributes:"
                  << getHandle() << ". Aborting.");
    setTranslated("link_order", linkOrder);
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
    setTranslated("render_mode", renderMode);
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
   * @brief Articulated Object Instance only. Get or set whether or not dofs
   * should be automatically clamped to specified joint limits before physics
   * simulation step.
   */
  bool getAutoClampJointLimits() const {
    return get<bool>("auto_clamp_joint_limits");
  }
  void setAutoClampJointLimits(bool auto_clamp_joint_limits) {
    set("auto_clamp_joint_limits", auto_clamp_joint_limits);
  }

  void setInitJointPose(const std::vector<float>& _jointPose) {
    setSubconfigValsOfTypeInVector("initial_joint_pose", _jointPose);
  }

  /**
   * @brief retrieve a mutable reference to this scene attributes joint
   * initial pose map
   */
  std::vector<float> getInitJointPose() const {
    return getSubconfigValsOfTypeInVector<float>("initial_joint_pose");
  }

  void setInitJointVelocities(const std::vector<float>& _jointVels) {
    setSubconfigValsOfTypeInVector("initial_joint_velocities", _jointVels);
  }

  /**
   * @brief retrieve a mutable reference to this scene attributes joint
   * initial pose map
   */
  std::vector<float> getInitJointVelocities() const {
    return getSubconfigValsOfTypeInVector<float>("initial_joint_velocities");
  }

 protected:
  friend class esp::metadata::managers::SceneInstanceAttributesManager;
  /**
   * @brief Called only from SceneInstanceAttributesManager, to create and
   * initialize an instance attributes from an existing object
   * attributes.
   *
   * SceneAOInstanceAttributes handle is also the handle of the
   * underlying @ref ArticulatedObjectAttributes for the object being
   * instanced. This initializes the instance attributes with the values from
   * the passed object attributes.
   */
  SceneAOInstanceAttributes(
      const std::string& handle,
      const std::shared_ptr<ArticulatedObjectAttributes>& aObjAttribs);

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this SceneAOInstanceAttributes object.
   */
  std::string getSceneObjInstanceInfoInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string
   * about the contents of this managed object.
   */
  std::string getSceneObjInstanceInfoHeaderInternal() const override;

  /**
   * @brief Populate the passed JSON object with all the first-level values
   * specific to this SceneObjectInstanceAttributes. This is to facilitate
   * SceneAOInstanceAttributes-specific values to be written.
   */
  void writeValuesToJsonInternal(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const override;

 public:
  ESP_SMART_POINTERS(SceneAOInstanceAttributes)

};  // class SceneAOInstanceAttributes

class SceneInstanceAttributes : public AbstractAttributes {
 public:
  explicit SceneInstanceAttributes(const std::string& handle);

  SceneInstanceAttributes(const SceneInstanceAttributes& otr);
  SceneInstanceAttributes(SceneInstanceAttributes&& otr) noexcept;

  SceneInstanceAttributes& operator=(const SceneInstanceAttributes& otr);
  SceneInstanceAttributes& operator=(SceneInstanceAttributes&& otr) noexcept;

  /**
   * @brief Set a value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  void setTranslationOrigin(const std::string& translation_origin) {
    // force to lowercase before setting
    const std::string transOriginLC =
        Cr::Utility::String::lowercase(translation_origin);
    auto mapIter = InstanceTranslationOriginMap.find(transOriginLC);

    ESP_CHECK((mapIter != InstanceTranslationOriginMap.end() ||
               (transOriginLC == getTranslationOriginName(
                                     SceneInstanceTranslationOrigin::Unknown))),
              "Illegal translation_origin value"
                  << translation_origin
                  << "attempted to be set in SceneInstanceAttributes :"
                  << getHandle() << ". Aborting.");
    set("translation_origin", translation_origin);
  }

  /**
   * @brief Get the value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  SceneInstanceTranslationOrigin getTranslationOrigin() const {
    const std::string val =
        Cr::Utility::String::lowercase(get<std::string>("translation_origin"));
    auto mapIter = InstanceTranslationOriginMap.find(val);
    if (mapIter != InstanceTranslationOriginMap.end()) {
      return mapIter->second;
    }
    // Unknown is default value
    return SceneInstanceTranslationOrigin::Unknown;
  }

  /**
   * @brief Set the name of the template that describes the scene's default
   * lighting
   */
  void setLightingHandle(const std::string& lightingHandle) {
    set("default_lighting", lightingHandle);
  }
  /**
   * @brief Get the name of the template that describes the scene's default
   * lighting
   */
  std::string getLightingHandle() const {
    return get<std::string>("default_lighting");
  }

  /**
   * @brief Set the name of the navmesh for the scene
   */
  void setNavmeshHandle(const std::string& navmeshHandle) {
    set("navmesh_instance", navmeshHandle);
  }
  /**
   * @brief Get the name of the navmesh for the scene
   */
  std::string getNavmeshHandle() const {
    return get<std::string>("navmesh_instance");
  }

  /**
   * @brief Set the name of the semantic scene descriptor
   */
  void setSemanticSceneHandle(const std::string& semanticSceneDesc) {
    set("semantic_scene_instance", semanticSceneDesc);
  }

  /**
   * @brief Get the name of the semantic scene descriptor
   */
  std::string getSemanticSceneHandle() const {
    return get<std::string>("semantic_scene_instance");
  }

  /**
   * @brief Set the description of the stage placement for this scene
   * instance. Scene instance will always have only 1 stage instance
   * reference.
   */
  void setStageInstanceAttrs(
      SceneObjectInstanceAttributes::ptr _stageInstance) {
    _stageInstance->setID(0);
    setSubconfigPtr<SceneObjectInstanceAttributes>("stage_instance",
                                                   _stageInstance);
  }
  /**
   * @brief Get a shared_pointer to the @ref SceneObjectInstanceAttributes
   * descibing the stage placement for this scene instance.
   */
  SceneObjectInstanceAttributes::cptr getStageInstance() const {
    return getSubconfigCopy<const SceneObjectInstanceAttributes>(
        "stage_instance");
  }

  /**
   * @brief Add an object instance attributes to this scene instance. Returns
   * false if not added due to a duplicate to @p _objInstance found in
   * @p objInstConfig_ during uniqueness validation.
   * @param _objInstance The object instance to add to the owning
   * subconfiguration.
   * @param _validateUnique Whether to validate uniqueness of @p _objInstance .
   * Note : hidden fields are ignored for this validation.
   * @return Whether or not @p _objInstance was added due to a duplicate being
   * found.
   */
  bool addObjectInstanceAttrs(SceneObjectInstanceAttributes::ptr _objInstance,
                              bool _validateUnique) {
    return setSubAttributesInternal<SceneObjectInstanceAttributes>(
        _objInstance, availableObjInstIDs_, objInstConfig_, "obj_inst_",
        _validateUnique);
  }

  /**
   * @brief Get the object instance descriptions for this scene
   */
  std::vector<SceneObjectInstanceAttributes::cptr> getObjectInstances() const {
    return getSubAttributesListInternal<SceneObjectInstanceAttributes>(
        objInstConfig_);
  }
  /**
   * @brief Return the number of defined @ref SceneObjectInstanceAttributes
   * subconfigs in this scene instance.
   */
  int getNumObjInstances() const {
    return getNumSubAttributesInternal("obj_inst_", objInstConfig_);
  }
  /**
   * @brief Clears current objInstConfig_ values.
   */
  void clearObjectInstances() {
    this->removeSubconfig("object_instances");
    objInstConfig_ = editSubconfig<Configuration>("object_instances");
  }

  /**
   * @brief Add an articulated object instance's attributes to this scene
   * instance. Returns false if not added due to a duplicate to
   * @p _artObjInstance found in @p artObjInstConfig_ .
   * @param _artObjInstance The object instance to add to the owning
   * subconfiguration.
   * @param _validateUnique Whether to validate uniqueness of @p _objInstance .
   * Note : hidden fields are ignored for this validation.
   * @return Whether or not @p _objInstance was added due to a duplicate being
   * found.
   */
  bool addArticulatedObjectInstanceAttrs(
      SceneAOInstanceAttributes::ptr _artObjInstance,
      bool _validateUnique) {
    return setSubAttributesInternal<SceneAOInstanceAttributes>(
        _artObjInstance, availableArtObjInstIDs_, artObjInstConfig_,
        "art_obj_inst_", _validateUnique);
  }

  /**
   * @brief Get the articulated object instance descriptions for this scene
   */
  std::vector<SceneAOInstanceAttributes::cptr> getArticulatedObjectInstances()
      const {
    return getSubAttributesListInternal<SceneAOInstanceAttributes>(
        artObjInstConfig_);
  }

  /**
   * @brief Return the number of defined @ref SceneAOInstanceAttributes
   * subconfigs in this scene instance.
   */
  int getNumAOInstances() const {
    return getNumSubAttributesInternal("art_obj_inst_", artObjInstConfig_);
  }
  /**
   * @brief Clears current artObjInstConfig_ values.
   */
  void clearArticulatedObjectInstances() {
    this->removeSubconfig("articulated_object_instances");
    artObjInstConfig_ =
        editSubconfig<Configuration>("articulated_object_instances");
  }

  /**
   * @brief Set the handle of the PbrShaderAttributes that would serve as
   * the default or is otherwise intended to be used across all semantic
   * regions in the scene not otherwise covered.
   * @param handle The handle of the PbrShaderAttributes to use for this scene
   * instance, as specified in the PbrShaderAttributesManager.
   */
  void setDefaultPbrShaderAttributesHandle(const std::string& handle) {
    set("default_pbr_shader_config", handle);
  }

  /**
   * @brief Get the handle of the PbrShaderAttributes that would serve as
   * the default or is otherwise intended to be used across all semantic
   * regions in the scene not otherwise covered.
   * @return The handle of the PbrShaderAttributes to use for this scene
   * instance, as specified in the PbrShaderAttributesManager.
   */
  std::string getDefaultPbrShaderAttributesHandle() const {
    return get<std::string>("default_pbr_shader_config");
  }

  /**
   * @brief Add the handle of a PbrShaderAttributes, keyed by semantic region
   * in scene where the config should be applied.
   * @param region The region/identifier in the scene to apply the specified
   * PbrShaderAttributes to the Pbr and Ibl shader calculations.
   * @param handle The handle of the PbrShaderAttributes to use for the given
   * @p region , as specified in the PbrShaderAttributesManager.
   */
  void addRegionPbrShaderAttributesHandle(const std::string& region,
                                          const std::string& handle) {
    pbrShaderRegionConfigHandles_->set(region, handle);
  }

  /**
   * @brief Get a vector of pairs of string,string, where the first value is a
   * region name, and the second is the handle to the PbrShaderAttributes to
   * apply to that region.
   */
  std::map<std::string, std::string> getRegionPbrShaderAttributesHandles()
      const;

  /**
   * @brief return how many PbrShaderAttributes handles have been found in the
   * SceneInstance.
   */
  int getNumRegionPbrShaderAttributes() const {
    return pbrShaderRegionConfigHandles_->getNumValues();
  }

  /**
   * @brief Populate a JSON object with all the first-level values held in this
   * configuration.  Default is overridden to handle special cases for
   * SceneInstanceAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

  /**
   * @brief Populate a JSON object with all the data from the subconfigurations,
   * held in JSON sub-objects, for this SceneInstance. Have special handling for
   * ao instances and object instances before handling other subConfigs.
   */
  void writeSubconfigsToJson(io::JsonGenericValue& jsonObj,
                             io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */
  std::string getObjectInfoHeaderInternal() const override { return ""; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

  /**
   * @brief Smartpointer to created object instance configuration. The
   * configuration is created on SceneInstanceAttributes construction.
   */
  std::shared_ptr<Configuration> objInstConfig_{};
  /**
   * @brief Deque holding all released IDs to consume for object instances
   * when one is deleted, before using size of objectInstances_ container.
   */
  std::deque<int> availableObjInstIDs_;

  /**
   * @brief Smartpointer to created articulated object instance configuration.
   * The configuration is created on SceneInstanceAttributes construction.
   */
  std::shared_ptr<Configuration> artObjInstConfig_{};

  /**
   * @brief Deque holding all released IDs to consume for articulated object
   * instances when one is deleted, before using size of
   * articulatedObjectInstances_ container.
   */
  std::deque<int> availableArtObjInstIDs_;

  /**
   * @brief Smartpointer to the subconfiguration holding the handles of the
   * PbrShaderConfiguration, keyed by the region they apply to.
   */
  std::shared_ptr<Configuration> pbrShaderRegionConfigHandles_{};

 public:
  ESP_SMART_POINTERS(SceneInstanceAttributes)
};  // class SceneInstanceAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SCENEINSTANCEATTRIBUTES_H_
