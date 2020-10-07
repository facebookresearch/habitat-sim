// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace physics {
enum class MotionType;
}
namespace metadata {
namespace attributes {

/**
 * @brief This class describes an instance of a stage or object in a scene -
 * it's template name, translation from the origin, rotation, and (if
 * appropriate) motiontype
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
  SceneObjectInstanceAttributes(const std::string& handle);

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

 public:
  ESP_SMART_POINTERS(SceneObjectInstanceAttributes)
};  // class SceneObjectInstanceAttributes

class SceneAttributes : public AbstractAttributes {
 public:
  /**
   * @brief This defines an example json descriptor for @ref SceneAttributes.
   * Has values that are different than defaults so this can be used to test
   * json loading. These values may be set to be purposefully weird/invalid
   * values, for testing purposes, and so should not be used for an actual
   * scene.
   */
  static const std::string JSONConfigTestString;
  SceneAttributes(const std::string& handle);

  /**
   * @brief Set the name of the template that describes the scene's stage
   */
  void setLightingHandle(const std::string& lightingHandle) {
    setString("default_lighting", lightingHandle);
  }
  /**
   * @brief Get the name of the template that describes the scene's stage
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
    stageInstance_ = _stageInstance;
  }
  /**
   * @brief Get the description of the stage placement for this scene instance.
   */
  const SceneObjectInstanceAttributes::ptr getStageInstance() const {
    return stageInstance_;
  }

  /**
   * @brief Add a description of an object instance to this scene instance
   */
  void addObjectInstance(SceneObjectInstanceAttributes::ptr _objInstance) {
    objectInstances_.push_back(_objInstance);
  }
  /**
   * @brief Get the object instance descriptions for this scene
   */
  const std::vector<SceneObjectInstanceAttributes::ptr> getObjectInstances()
      const {
    return objectInstances_;
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

 public:
  ESP_SMART_POINTERS(SceneAttributes)
};  // class SceneAttributes
}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_SCENEATTRIBUTES_H_
