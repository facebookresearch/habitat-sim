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
  setVec3("translation", Mn::Vector3());
  // defaults to unknown so that obj instances use scene instance setting
  setTranslationOrigin(
      static_cast<int>(SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  setUniformScale(1.0f);
  setMassScale(1.0f);
}

std::string SceneObjectInstanceAttributes::getCurrShaderTypeName() const {
  int shaderTypeVal = getShaderType();
  return getShaderTypeName(shaderTypeVal);
}

std::string SceneObjectInstanceAttributes::getObjectInfoHeaderInternal() const {
  return "Translation XYZ, Rotation XYZW, Motion Type, Shader Type, Uniform "
         "Scale, Mass Scale, Translation Origin, " +
         getSceneObjInstanceInfoHeaderInternal();
}

std::string SceneObjectInstanceAttributes::getObjectInfoInternal() const {
  return cfg.value("translation")
      .append(1, ',')
      .append(cfg.value("rotation"))
      .append(1, ',')
      .append(getCurrMotionTypeName())
      .append(1, ',')
      .append(getCurrShaderTypeName())
      .append(1, ',')
      .append(cfg.value("uniform_scale"))
      .append(1, ',')
      .append(cfg.value("mass_scale"))
      .append(1, ',')
      .append(getTranslationOriginName(getTranslationOrigin()))
      .append(1, ',')
      .append(getSceneObjInstanceInfoInternal());
}  // SceneObjectInstanceAttributes::getObjectInfoInternal()

SceneAOInstanceAttributes::SceneAOInstanceAttributes(const std::string& handle)
    : SceneObjectInstanceAttributes(handle, "SceneAOInstanceAttributes") {
  // set default fixed base and auto clamp values (only used for articulated
  // object)
  setFixedBase(false);
  setAutoClampJointLimits(false);
}

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal()
    const {
  const std::string posePrfx{"Init Pose "};
  std::string initPoseHdr;
  int iter = 0;
  for (const auto& it : initJointPose_) {
    initPoseHdr.append(posePrfx).append(std::to_string(iter++)).append(1, ',');
  }
  const std::string velPrfx{"Init Vel "};
  std::string initVelHdr;
  iter = 0;
  for (const auto& it : initJointPose_) {
    initVelHdr.append(velPrfx).append(std::to_string(iter++)).append(1, ',');
  }
  std::string res{"Is Fixed Base?, "};
  res.append(initPoseHdr).append(initVelHdr);
  return res;
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal() const {
  std::string initJointPose{"["};
  for (const auto& it : initJointPose_) {
    initJointPose.append(std::to_string(it.second)).append(1, ',');
  }
  initJointPose.append("]");
  std::string initJointVels{"["};
  for (const auto& it : initJointPose_) {
    initJointVels.append(std::to_string(it.second)).append(1, ',');
  }
  initJointVels.append("]");
  return cfg.value("fixed_base").append(1, ',') + initJointPose.append(1, ',') +
         initJointVels.append(1, ',');
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal()

SceneAttributes::SceneAttributes(const std::string& handle)
    : AbstractAttributes("SceneAttributes", handle) {
  // defaults to no lights
  setLightingHandle(NO_LIGHT_KEY);
  // defaults to asset local
  setTranslationOrigin(
      static_cast<int>(SceneInstanceTranslationOrigin::AssetLocal));
}

std::string SceneAttributes::getObjectInfoInternal() const {
  std::string res = "\n";
  // scene-specific info constants
  // default translation origin
  res.append(
      "Default Translation Origin, Default Lighting,Navmesh Handle,Semantic "
      "Scene Descriptor Handle,\n");

  res.append(getTranslationOriginName(getTranslationOrigin()))
      .append(1, ',')
      .append(getLightingHandle())
      .append(1, ',')
      .append(getNavmeshHandle())
      .append(1, ',')
      .append(getSemanticSceneHandle())
      .append(1, '\n');

  // stage instance info
  res.append("Stage Instance Info :\n");
  res.append(stageInstance_->getObjectInfoHeader()).append(1, '\n');
  res.append(stageInstance_->getObjectInfo()).append(1, '\n');

  int iter = 0;
  // object instance info
  for (const auto& objInst : objectInstances_) {
    if (iter == 0) {
      iter++;
      res.append("Object Instance Info :\n");
      res.append(objInst->getObjectInfoHeader()).append(1, '\n');
    }
    res.append(objInst->getObjectInfo()).append(1, '\n');
  }

  // articulated object instance info
  iter = 0;
  for (const auto& artObjInst : articulatedObjectInstances_) {
    if (iter == 0) {
      iter++;
      res.append("Articulated Object Instance Info :\n");
      res.append(artObjInst->getObjectInfoHeader()).append(1, '\n');
    }
    res.append(artObjInst->getObjectInfo()).append(1, '\n');
  }

  res.append("End of data for Scene Instance ")
      .append(getSimplifiedHandle())
      .append(1, '\n');

  return res;
}  // SceneAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
