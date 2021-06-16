// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_

#include <utility>

#include "AttributesBase.h"

namespace esp {
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace managers {
enum class SceneInstanceTranslationOrigin;
}
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
   * esp::assets::AssetType values.  This will be used to map values set in json
   * for mesh type to @ref esp::assets::AssetType.  Keys must be lowercase.
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
    setVec3("translation", translation);
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
    setInt("translation_origin", translation_origin);
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
    setQuat("rotation", rotation);
  }
  /**
   * @brief Get the rotation of the object
   */
  Magnum::Quaternion getRotation() const { return getQuat("rotation"); }

  /**
   * @brief Set the motion type for the object.  Ignored for stage instances.
   */
  void setMotionType(int motionType) { setInt("motion_type", motionType); }

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
  void setShaderType(int shader_type) { setInt("shader_type", shader_type); }
  int getShaderType() const { return getInt("shader_type"); }

  /**
   * @brief Get or set the uniform scaling of the instanced object.
   */
  float getUniformScale() const { return getFloat("uniform_scale"); }
  void setUniformScale(float uniform_scale) {
    setFloat("uniform_scale", uniform_scale);
  }

  /**
   * @brief Get or set the mass scaling of the instanced object.
   */
  float getMassScale() const { return getFloat("mass_scale"); }
  void setMassScale(float mass_scale) { setFloat("mass_scale", mass_scale); }

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
  void setFixedBase(bool fixed_base) { setBool("fixed_base", fixed_base); }

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
  /**
   * @brief Constant static map to provide mappings from string tags to @ref
   * metadata::managers::SceneInstanceTranslationOrigin values.  This will be
   * used to map values set in json for translation origin to @ref
   * metadata::managers::SceneInstanceTranslationOrigin.  Keys must be
   * lowercase.
   */
  static const std::map<std::string,
                        metadata::managers::SceneInstanceTranslationOrigin>
      InstanceTranslationOriginMap;

  explicit SceneAttributes(const std::string& handle);

  /**
   * @brief Set a value representing the mechanism used to create this scene
   * instance - should map to an enum value in @InstanceTranslationOriginMap.
   */
  void setTranslationOrigin(int translation_origin) {
    setInt("translation_origin", translation_origin);
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
    setString("default_lighting", lightingHandle);
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
    setString("navmesh_instance", navmeshHandle);
  }
  /**
   * @brief Get the name of the navmesh for the scene
   */
  std::string getNavmeshHandle() const { return getString("navmesh_instance"); }

  /**
   * @brief Set the name of the semantic scene descriptor
   */
  void setSemanticSceneHandle(const std::string& semanticSceneDesc) {
    setString("semantic_scene_instance", semanticSceneDesc);
  }

  /**
   * @brief Get the name of the semantic scene descriptor
   */
  std::string getSemanticSceneHandle() const {
    return getString("semantic_scene_instance");
  }

  /**
   * @brief Set the description of the stage placement for this scene instance.
   */
  void setStageInstance(SceneObjectInstanceAttributes::ptr _stageInstance) {
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
   * @brief The stage instance used by the scene
   */
  SceneObjectInstanceAttributes::ptr stageInstance_ = nullptr;

  /**
   * @brief All the object instance descriptors used by the scene
   */
  std::vector<SceneObjectInstanceAttributes::ptr> objectInstances_;

  /**
   * @brief All the articulated object instance descriptors used by the scene
   */
  std::vector<SceneAOInstanceAttributes::ptr> articulatedObjectInstances_;

 public:
  ESP_SMART_POINTERS(SceneAttributes)
};  // class SceneAttributes
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
