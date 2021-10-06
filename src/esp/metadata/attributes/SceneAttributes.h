// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_

#include <deque>
#include <utility>

#include "AttributesBase.h"

namespace esp {
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace managers {
enum class SceneInstanceTranslationOrigin;
}  // namespace managers
namespace attributes {

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
  void setTranslation(const Magnum::Vector3& translation) {
    set("translation", translation);
  }
  /**
   * @brief Get the translation from the origin of the described
   * stage/object instance.
   */
  Magnum::Vector3 getTranslation() const {
    return get<Magnum::Vector3>("translation");
  }

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
   * @brief Set the rotation of the object
   */
  void setRotation(const Magnum::Quaternion& rotation) {
    set("rotation", rotation);
  }
  /**
   * @brief Get the rotation of the object
   */
  Magnum::Quaternion getRotation() const {
    return get<Magnum::Quaternion>("rotation");
  }

  /**
   * @brief If not visible can add dynamic non-rendered object into a scene
   * object.  If is not visible then should not add object to drawables.
   */
  void setIsInstanceVisible(bool isVisible) {
    // needs to be int to cover "no specification"
    set("is_instance_visible", (isVisible ? 1 : 0));
  }
  int getIsInstanceVisible() const { return get<int>("is_instance_visible"); }

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
   * of string values in json to @ref
   * esp::metadata::attributes::ObjectInstanceShaderType int values.
   */
  void setShaderType(const std::string& shader_type) {
    // force to lowercase before setting
    const std::string shaderTypeLC =
        Cr::Utility::String::lowercase(shader_type);
    auto mapIter = ShaderTypeNamesMap.find(shaderTypeLC);

    ESP_CHECK((mapIter != ShaderTypeNamesMap.end() ||
               (shaderTypeLC ==
                getShaderTypeName(ObjectInstanceShaderType::Unknown))),
              "Illegal shader_type value"
                  << shader_type
                  << "attempted to be set in SceneObjectInstanceAttributes :"
                  << getHandle() << ". Aborting.");

    set("shader_type", shader_type);
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
    // unknown is default value
    return ObjectInstanceShaderType::Unknown;
  }

  /**
   * @brief Get or set the uniform scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  float getUniformScale() const {
    return static_cast<float>(get<double>("uniform_scale"));
  }
  void setUniformScale(double uniform_scale) {
    set("uniform_scale", uniform_scale);
  }

  /**
   * @brief Get or set the mass scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  float getMassScale() const {
    return static_cast<float>(get<double>("mass_scale"));
  }
  void setMassScale(double mass_scale) { set("mass_scale", mass_scale); }

 protected:
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
   * @brief SceneObjectInstanceAttributes handle is also the handle of the
   * underlying @ref AbstractObjectAttributes for the object being instanced.
   */
  explicit SceneAOInstanceAttributes(const std::string& handle);

  /**
   * @brief Articulated Object Instance only. Get or set whether or not base
   * is fixed.
   */
  bool getFixedBase() const { return get<bool>("fixed_base"); }
  void setFixedBase(bool fixed_base) { set("fixed_base", fixed_base); }

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

  /**
   * @brief retrieve a mutable reference to this scene attributes joint
   * initial pose map
   */
  const std::map<std::string, float>& getInitJointPose() const {
    return initJointPose_;
  }

  std::map<std::string, float>& copyIntoInitJointPose() {
    return initJointPose_;
  }

  /**
   * @brief Add a value to this scene attributes joint initial pose map
   * @param key the location/joint name to place the value
   * @param val the joint value to set
   */
  void addInitJointPoseVal(const std::string& key, float val) {
    initJointPose_[key] = val;
  }

  /**
   * @brief retrieve a mutable reference to this scene attributes joint
   * initial velocity map
   */
  const std::map<std::string, float>& getInitJointVelocities() const {
    return initJointVelocities_;
  }
  std::map<std::string, float>& copyIntoInitJointVelocities() {
    return initJointVelocities_;
  }

  /**
   * @brief Add a value to this scene attributes joint initial velocity map
   * @param key the location/joint name to place the value
   * @param val the joint angular velocity value to set
   */
  void addInitJointVelocityVal(const std::string& key, float val) {
    initJointVelocities_[key] = val;
  }

 protected:
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
   * @brief Map of joint names/idxs to values for initial pose
   */
  std::map<std::string, float> initJointPose_;

  /**
   * @brief Map of joint names/idxs to values for initial velocities
   */
  std::map<std::string, float> initJointVelocities_;

 public:
  ESP_SMART_POINTERS(SceneAOInstanceAttributes)

};  // class SceneAOInstanceAttributes

class SceneAttributes : public AbstractAttributes {
 public:
  explicit SceneAttributes(const std::string& handle);

  SceneAttributes(const SceneAttributes& otr);
  SceneAttributes(SceneAttributes&& otr) noexcept;

  SceneAttributes& operator=(const SceneAttributes& otr);
  SceneAttributes& operator=(SceneAttributes&& otr) noexcept;

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
  void setStageInstance(SceneObjectInstanceAttributes::ptr _stageInstance) {
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
   * @brief Add a description of an object instance to this scene instance
   */
  void addObjectInstance(SceneObjectInstanceAttributes::ptr _objInstance) {
    setSubAttributesInternal<SceneObjectInstanceAttributes>(
        _objInstance, availableObjInstIDs_, objInstConfig_, "obj_inst_");
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
   * @brief Add a description of an object instance to this scene instance
   */
  void addArticulatedObjectInstance(
      SceneAOInstanceAttributes::ptr _artObjInstance) {
    setSubAttributesInternal<SceneAOInstanceAttributes>(
        _artObjInstance, availableArtObjInstIDs_, artObjInstConfig_,
        "art_obj_inst_");
  }

  /**
   * @brief Get the object instance descriptions for this scene
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
   * configuration is created on SceneAttributes construction.
   */
  std::shared_ptr<Configuration> objInstConfig_{};
  /**
   * @brief Deque holding all released IDs to consume for object instances
   * when one is deleted, before using size of objectInstances_ container.
   */
  std::deque<int> availableObjInstIDs_;

  /**
   * @brief Smartpointer to created articulated object instance configuration.
   * The configuratio is created on SceneAttributes construction.
   */
  std::shared_ptr<Configuration> artObjInstConfig_{};

  /**
   * @brief Deque holding all released IDs to consume for articulated object
   * instances when one is deleted, before using size of
   * articulatedObjectInstances_ container.
   */
  std::deque<int> availableArtObjInstIDs_;

 public:
  ESP_SMART_POINTERS(SceneAttributes)
};  // class SceneAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
