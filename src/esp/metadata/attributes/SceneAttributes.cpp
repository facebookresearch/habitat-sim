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
  set("rotation", Mn::Quaternion(Mn::Math::IdentityInit));
  set("translation", Mn::Vector3());
  // defaults to unknown so that obj instances use scene instance setting
  setTranslationOrigin(
      static_cast<int>(SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  setUniformScale(1.0);
  setMassScale(1.0);
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
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getAsString("translation"),
      getAsString("rotation"), getCurrMotionTypeName(), getCurrShaderTypeName(),
      getAsString("uniform_scale"), getAsString("mass_scale"),
      getTranslationOriginName(getTranslationOrigin()),
      getSceneObjInstanceInfoInternal());
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
  std::string initPoseHdr;
  int iter = 0;
  for (const auto& it : initJointPose_) {
    initPoseHdr.append(
        Cr::Utility::formatString("Init Pose {},", std::to_string(iter++)));
  }
  std::string initVelHdr;
  iter = 0;
  for (const auto& it : initJointPose_) {
    initVelHdr.append(
        Cr::Utility::formatString("Init Vel {},", std::to_string(iter++)));
  }
  return Cr::Utility::formatString("Is Fixed Base?, {} {}", initPoseHdr,
                                   initVelHdr);
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal() const {
  std::string initJointPose{"["};
  for (const auto& it : initJointPose_) {
    initJointPose.append(
        Cr::Utility::formatString("{},", std::to_string(it.second)));
  }
  initJointPose.append("]");
  std::string initJointVels{"["};
  for (const auto& it : initJointPose_) {
    initJointVels.append(
        Cr::Utility::formatString("{},", std::to_string(it.second)));
  }
  initJointVels.append("]");

  return Cr::Utility::formatString("{},{},{},", getAsString("fixed_base"),
                                   initJointPose, initJointVels);
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
  // scene-specific info constants
  // default translation origin

  std::string res = Cr::Utility::formatString(
      "\nDefault Translation Origin, Default Lighting,Navmesh Handle,Semantic "
      "Scene Descriptor Handle,\n{},{},{},{}\nStage Instance Info :\n{}\n{}\n",
      getTranslationOriginName(getTranslationOrigin()), getLightingHandle(),
      getNavmeshHandle(), getSemanticSceneHandle(),
      stageInstance_->getObjectInfoHeader(), stageInstance_->getObjectInfo());
  // stage instance info
  int iter = 0;
  // object instance info
  for (const auto& objInst : objectInstances_) {
    if (iter == 0) {
      ++iter;
      res.append(Cr::Utility::formatString("Object Instance Info :\n{}\n",
                                           objInst->getObjectInfoHeader()));
    }
    res.append(Cr::Utility::formatString("{}\n", objInst->getObjectInfo()));
  }

  // articulated object instance info
  iter = 0;
  for (const auto& artObjInst : articulatedObjectInstances_) {
    if (iter == 0) {
      ++iter;
      res.append(
          Cr::Utility::formatString("Articulated Object Instance Info :\n{}\n",
                                    artObjInst->getObjectInfoHeader()));
    }
    res.append(Cr::Utility::formatString("{}\n", artObjInst->getObjectInfo()));
  }

  res.append(Cr::Utility::formatString("End of data for Scene Instance {}\n",
                                       getSimplifiedHandle()));
  return res;
}  // SceneAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
