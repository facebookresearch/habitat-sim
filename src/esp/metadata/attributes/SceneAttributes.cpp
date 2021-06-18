// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributes.h"
#include "esp/physics/RigidBase.h"
namespace esp {
namespace metadata {
namespace attributes {
// All keys must be lowercase
const std::map<std::string, esp::physics::MotionType>
    SceneObjectInstanceAttributes::MotionTypeNamesMap = {
        {"static", esp::physics::MotionType::STATIC},
        {"kinematic", esp::physics::MotionType::KINEMATIC},
        {"dynamic", esp::physics::MotionType::DYNAMIC},
};

SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle,
    const std::string& type)
    : AbstractAttributes(type, handle) {
  // default to unknown for object instances, to use attributes-specified
  // defaults
  setShaderType(static_cast<int>(ObjectInstanceShaderType::Unknown));

  // defaults to unknown/undefined
  setMotionType(static_cast<int>(esp::physics::MotionType::UNDEFINED));
  // set to no rotation
  setQuat("rotation", Mn::Quaternion(Mn::Math::IdentityInit));
  // defaults to unknown so that obj instances use scene instance setting
  setTranslationOrigin(
      static_cast<int>(managers::SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  setUniformScale(1.0f);
  setMassScale(1.0f);
}

/**
 * @brief Used for info purposes.  Return a string name corresponding to the
 * currently specified shader type value;
 */
std::string SceneObjectInstanceAttributes::getCurrShaderTypeName() const {
  int shaderTypeVal = getShaderType();
  if (shaderTypeVal <=
          static_cast<int>(attributes::ObjectInstanceShaderType::Unknown) ||
      shaderTypeVal >=
          static_cast<int>(
              attributes::ObjectInstanceShaderType::_EndShaderType)) {
    return "unknown shader type";
  }
  // Must always be valid value
  ObjectInstanceShaderType shaderType =
      static_cast<ObjectInstanceShaderType>(shaderTypeVal);
  for (const auto& it :
       attributes::AbstractObjectAttributes::ShaderTypeNamesMap) {
    if (it.second == shaderType) {
      return it.first;
    }
  }
  return "unknown shader type";
}

std::string SceneObjectInstanceAttributes::getObjectInfoInternal() const {
  return cfg.value("translation") + ", " + cfg.value("rotation") + ", " +
         getCurrMotionTypeName() + ", " + getCurrShaderTypeName() + ", " +
         cfg.value("uniform_scale") + ", " + cfg.value("mass_scale") + ", " +
         getSceneObjInstanceInfoInternal();
}  // SceneObjectInstanceAttributes::getObjectInfoInternal()

SceneAOInstanceAttributes::SceneAOInstanceAttributes(const std::string& handle)
    : SceneObjectInstanceAttributes(handle, "SceneAOInstanceAttributes") {
  // set default fixed base value (only used for articulated object)
  setFixedBase(false);
}

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal() const {
  std::string initJointPose = "[";
  for (const auto& it : initJointPose_) {
    initJointPose += std::to_string(it.second) + ", ";
  }
  initJointPose = "]";
  std::string initJointVels = "[";
  for (const auto& it : initJointPose_) {
    initJointVels += std::to_string(it.second) + ", ";
  }
  initJointVels = "]";
  return cfg.value("fixed_base") + ", " + initJointPose + ", " + initJointVels +
         ", ";
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal()

const std::map<std::string, managers::SceneInstanceTranslationOrigin>
    SceneAttributes::InstanceTranslationOriginMap = {
        {"asset_local", managers::SceneInstanceTranslationOrigin::AssetLocal},
        {"com", managers::SceneInstanceTranslationOrigin::COM},
};

SceneAttributes::SceneAttributes(const std::string& handle)
    : AbstractAttributes("SceneAttributes", handle) {
  // defaults to no lights
  setLightingHandle(NO_LIGHT_KEY);
  // defaults to asset local
  setTranslationOrigin(
      static_cast<int>(managers::SceneInstanceTranslationOrigin::AssetLocal));
}

std::string SceneAttributes::getObjectInfoInternal() const {
  std::string res = "\n";
  // stage instance info
  res += stageInstance_->getObjectInfo() + "\n";

  // object instance info
  for (const auto& objInst : objectInstances_) {
    res += objInst->getObjectInfo() + "\n";
  }

  // articulated object instance info
  for (const auto& artObjInst : articulatedObjectInstances_) {
    res += artObjInst->getObjectInfo() + "\n";
  }
  return res;
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
