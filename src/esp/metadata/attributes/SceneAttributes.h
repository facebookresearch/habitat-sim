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
   * @brief Constant static map to provide mappings from string tags to @ref
   * esp::physics::MotionType values.  This will be used to map values set in
   * json for mesh type to @ref esp::physics::MotionType.  Keys must be
   * lowercase.
   */
  static const std::map<std::string, esp::physics::MotionType>
      MotionTypeNamesMap;

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
  Magnum::Vector3 getTranslation() const { return getVec3("translation"); }

  /**
   * @brief Set a value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  void setTranslationOrigin(int translation_origin) {
    set("translation_origin", translation_origin);
  }

  /**
   * @brief Get the value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   * This acts as an instance-specific override to the scene-instance-wide
   * setting.
   */
  int getTranslationOrigin() const { return getInt("translation_origin"); }

  /**
   * @brief Set the rotation of the object
   */
  void setRotation(const Magnum::Quaternion& rotation) {
    set("rotation", rotation);
  }
  /**
   * @brief Get the rotation of the object
   */
  Magnum::Quaternion getRotation() const { return getQuat("rotation"); }

  /**
   * @brief Set the motion type for the object.  Ignored for stage instances.
   */
  void setMotionType(int motionType) { set("motion_type", motionType); }

  /**
   * @brief Get the motion type for the object.  Ignored for stage instances.
   */
  int getMotionType() const { return getInt("motion_type"); }

  /**
   * @brief Set the default shader to use for an object or stage.  Uses values
   * specified in stage or object attributes if not overridden here.  Uses map
   * of string values in json to @ref
   * esp::metadata::attributes::ObjectInstanceShaderType int values.
   */
  void setShaderType(int shader_type) { set("shader_type", shader_type); }
  int getShaderType() const { return getInt("shader_type"); }

  /**
   * @brief Get or set the uniform scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  float getUniformScale() const {
    return static_cast<float>(getDouble("uniform_scale"));
  }
  void setUniformScale(double uniform_scale) {
    set("uniform_scale", uniform_scale);
  }

  /**
   * @brief Get or set the mass scaling of the instanced object.  Want this
   * to be a float for consumption in instance creation
   */
  float getMassScale() const {
    return static_cast<float>(getDouble("mass_scale"));
  }
  void setMassScale(double mass_scale) { set("mass_scale", mass_scale); }

  /**
   * @brief Used for info purposes.  Return a string name corresponding to the
   * currently specified motion type value;
   */
  std::string getCurrMotionTypeName() const {
    // Must always be valid value
    esp::physics::MotionType motionType =
        static_cast<esp::physics::MotionType>(getMotionType());
    for (const auto& it : MotionTypeNamesMap) {
      if (it.second == motionType) {
        return it.first;
      }
    }
    return "unspecified";
  }

  /**
   * @brief Used for info purposes.  Return a string name corresponding to the
   * currently specified shader type value;
   */
  std::string getCurrShaderTypeName() const;

 protected:
  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
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
 * @brief This class describes an instance of an articulated object in a scene -
 * along with its template name, translation from the origin, rotation,
 * motiontype, and other values inherited from SceneObjectInstanceAttributes, it
 * also holds initial joint pose and joint velocities.
 */
class SceneAOInstanceAttributes : public SceneObjectInstanceAttributes {
 public:
  /**
   * @brief SceneObjectInstanceAttributes handle is also the handle of the
   * underlying @ref AbstractObjectAttributes for the object being instanced.
   */
  explicit SceneAOInstanceAttributes(const std::string& handle);

  /**
   * @brief Articulated Object Instance only. Get or set whether or not base is
   * fixed.
   */
  bool getFixedBase() const { return getBool("fixed_base"); }
  void setFixedBase(bool fixed_base) { set("fixed_base", fixed_base); }

  /**
   * @brief Articulated Object Instance only. Get or set whether or not dofs
   * should be automatically clamped to specified joint limits before physics
   * simulation step.
   */
  bool getAutoClampJointLimits() const {
    return getBool("auto_clamp_joint_limits");
  }
  void setAutoClampJointLimits(bool auto_clamp_joint_limits) {
    set("auto_clamp_joint_limits", auto_clamp_joint_limits);
  }

  /**
   * @brief retrieve a mutable reference to this scene attributes joint initial
   * pose map
   */
  std::map<std::string, float>& getInitJointPose() { return initJointPose_; }

  /**
   * @brief Add a value to this scene attributes joint initial pose map
   * @param key the location/joint name to place the value
   * @param val the joint value to set
   */
  void addInitJointPoseVal(const std::string& key, float val) {
    initJointPose_[key] = val;
  }

  /**
   * @brief retrieve a mutable reference to this scene attributes joint initial
   * velocity map
   */
  std::map<std::string, float>& getInitJointVelocities() {
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

  /**
   * @brief Set a value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   */
  void setTranslationOrigin(int translation_origin) {
    set("translation_origin", translation_origin);
  }

  /**
   * @brief Get the value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   */
  int getTranslationOrigin() const { return getInt("translation_origin"); }

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
    return getString("default_lighting");
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
  std::string getNavmeshHandle() const { return getString("navmesh_instance"); }

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
    return getString("semantic_scene_instance");
  }

  /**
   * @brief Set the description of the stage placement for this scene instance.
   * Scene instance will always have only 1 stage instance reference.
   */
  void setStageInstance(SceneObjectInstanceAttributes::ptr _stageInstance) {
    _stageInstance->setID(0);
    stageInstance_ = std::move(_stageInstance);
  }
  /**
   * @brief Get the description of the stage placement for this scene instance.
   */
  SceneObjectInstanceAttributes::ptr getStageInstance() const {
    return stageInstance_;
  }

  /**
   * @brief Add a description of an object instance to this scene instance
   */
  void addObjectInstance(
      const SceneObjectInstanceAttributes::ptr& _objInstance) {
    // set id
    if (availableObjInstIDs_.size() > 0) {
      // use saved value and then remove from storage
      _objInstance->setID(availableObjInstIDs_.front());
      availableObjInstIDs_.pop_front();
    } else {
      // use size of container to set ID
      _objInstance->setID(objectInstances_.size());
    }
    objectInstances_.push_back(_objInstance);
  }

  /**
   * @brief Get the object instance descriptions for this scene
   */
  const std::vector<SceneObjectInstanceAttributes::ptr>& getObjectInstances()
      const {
    return objectInstances_;
  }

  /**
   * @brief Add a description of an object instance to this scene instance
   */
  void addArticulatedObjectInstance(
      const SceneAOInstanceAttributes::ptr& _artObjInstance) {
    // set id
    if (availableArtObjInstIDs_.size() > 0) {
      // use saved value and then remove from storage
      _artObjInstance->setID(availableArtObjInstIDs_.front());
      availableArtObjInstIDs_.pop_front();
    } else {
      // use size of container to set ID
      _artObjInstance->setID(articulatedObjectInstances_.size());
    }
    articulatedObjectInstances_.push_back(_artObjInstance);
  }

  /**
   * @brief Get the object instance descriptions for this scene
   */
  const std::vector<SceneAOInstanceAttributes::ptr>&
  getArticulatedObjectInstances() const {
    return articulatedObjectInstances_;
  }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override { return ""; }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;
  /**
   * @brief The stage instance used by the scene
   */
  SceneObjectInstanceAttributes::ptr stageInstance_ = nullptr;

  /**
   * @brief All the object instance descriptors used by the scene
   */
  std::vector<SceneObjectInstanceAttributes::ptr> objectInstances_;
  /**
   * @brief Deque holding all released IDs to consume for object instances when
   * one is deleted, before using size of objectInstances_ container.
   */
  std::deque<int> availableObjInstIDs_;

  /**
   * @brief All the articulated object instance descriptors used by the scene
   */
  std::vector<SceneAOInstanceAttributes::ptr> articulatedObjectInstances_;
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
