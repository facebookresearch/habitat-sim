// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributes.h"
#include "esp/physics/RigidBase.h"
namespace esp {
namespace metadata {
namespace attributes {

SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle,
    const std::string& type)
    : AbstractAttributes(type, handle) {
  // default to unknown for object instances, to use attributes-specified
  // defaults
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Unknown));

  // defaults to unknown/undefined
  setMotionType(getMotionTypeName(esp::physics::MotionType::UNDEFINED));
  // set to no rotation
  set("rotation", Mn::Quaternion(Mn::Math::IdentityInit));
  set("translation", Mn::Vector3());
  // don't override attributes-specified visibility.
  set("is_instance_visible", ID_UNDEFINED);
  // defaults to unknown so that obj instances use scene instance setting
  setTranslationOrigin(
      getTranslationOriginName(SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  setUniformScale(1.0);
  setMassScale(1.0);
}

std::string SceneObjectInstanceAttributes::getObjectInfoHeaderInternal() const {
  return "Translation XYZ,Rotation XYZW,Motion Type,Shader Type,Uniform "
         "Scale,Mass Scale,Translation Origin," +
         getSceneObjInstanceInfoHeaderInternal();
}

std::string SceneObjectInstanceAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getAsString("translation"),
      getAsString("rotation"), getMotionTypeName(getMotionType()),
      getShaderTypeName(getShaderType()), getAsString("uniform_scale"),
      getAsString("mass_scale"),
      getTranslationOriginName(getTranslationOrigin()),
      getSceneObjInstanceInfoInternal());
}  // SceneObjectInstanceAttributes::getObjectInfoInternal()

/**
 * @brief Set the motion type for the object.  Ignored for stage instances.
 */
void SceneObjectInstanceAttributes::setMotionType(
    const std::string& motionType) {
  // force to lowercase before setting
  const std::string motionTypeLC = Cr::Utility::String::lowercase(motionType);
  auto mapIter = MotionTypeNamesMap.find(motionTypeLC);

  ESP_CHECK(
      (mapIter != MotionTypeNamesMap.end() ||
       (motionType == getMotionTypeName(esp::physics::MotionType::UNDEFINED))),
      "Illegal motion_type value"
          << motionType
          << "attempted to be set in SceneObjectInstanceAttributes :"
          << getHandle() << ". Aborting.");
  set("motion_type", motionType);
}

/**
 * @brief Get the motion type for the object.  Ignored for stage instances.
 */
esp::physics::MotionType SceneObjectInstanceAttributes::getMotionType() const {
  const std::string val =
      Cr::Utility::String::lowercase(get<std::string>("motion_type"));
  auto mapIter = MotionTypeNamesMap.find(val);
  if (mapIter != MotionTypeNamesMap.end()) {
    return mapIter->second;
  }
  // global is default value
  return esp::physics::MotionType::UNDEFINED;
}

SceneAOInstanceAttributes::SceneAOInstanceAttributes(const std::string& handle)
    : SceneObjectInstanceAttributes(handle, "SceneAOInstanceAttributes") {
  // set default fixed base and auto clamp values (only used for articulated
  // object)
  setFixedBase(false);
  setAutoClampJointLimits(false);
}

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal()
    const {
  std::string infoHdr{"Is Fixed Base?,"};
  int iter = 0;
  for (const auto& it : initJointPose_) {
    Cr::Utility::formatInto(infoHdr, infoHdr.size(), "Init Pose {},",
                            std::to_string(iter++));
  }
  iter = 0;
  for (const auto& it : initJointPose_) {
    Cr::Utility::formatInto(infoHdr, infoHdr.size(), "Init Vel {},",
                            std::to_string(iter++));
  }
  return infoHdr;
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal() const {
  std::string initPoseStr{"["};
  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},",
                          getAsString("fixed_base"));
  for (const auto& it : initJointPose_) {
    Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},",
                            std::to_string(it.second));
  }
  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "],[");
  for (const auto& it : initJointPose_) {
    Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},",
                            std::to_string(it.second));
  }
  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "]");

  return initPoseStr;
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal()

SceneAttributes::SceneAttributes(const std::string& handle)
    : AbstractAttributes("SceneAttributes", handle) {
  // defaults to no lights
  setLightingHandle(NO_LIGHT_KEY);
  // defaults to asset local
  setTranslationOrigin(
      getTranslationOriginName(SceneInstanceTranslationOrigin::AssetLocal));
  // get refs to internal subconfigs for object and ao instances
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ = editSubconfig<Configuration>("ao_instances");
}

SceneAttributes::SceneAttributes(const SceneAttributes& otr)
    : AbstractAttributes(otr),
      availableObjInstIDs_(otr.availableObjInstIDs_),
      availableArtObjInstIDs_(otr.availableArtObjInstIDs_) {
  // get refs to internal subconfigs for object and ao instances
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  copySubconfigIntoMe<SceneObjectInstanceAttributes>(otr.objInstConfig_,
                                                     objInstConfig_);
  artObjInstConfig_ = editSubconfig<Configuration>("ao_instances");
  copySubconfigIntoMe<SceneAOInstanceAttributes>(otr.artObjInstConfig_,
                                                 artObjInstConfig_);
}
SceneAttributes::SceneAttributes(SceneAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))),
      availableObjInstIDs_(std::move(otr.availableObjInstIDs_)),
      availableArtObjInstIDs_(std::move(otr.availableArtObjInstIDs_)) {
  // get refs to internal subconfigs for object and ao instances
  // originals were moved over so should retain full derived class
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ = editSubconfig<Configuration>("ao_instances");
}

SceneAttributes& SceneAttributes::operator=(const SceneAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
    availableObjInstIDs_ = otr.availableObjInstIDs_;
    availableArtObjInstIDs_ = otr.availableArtObjInstIDs_;
    // get refs to internal subconfigs for object and ao instances
    objInstConfig_ = editSubconfig<Configuration>("object_instances");
    copySubconfigIntoMe<SceneObjectInstanceAttributes>(otr.objInstConfig_,
                                                       objInstConfig_);
    artObjInstConfig_ = editSubconfig<Configuration>("ao_instances");
    copySubconfigIntoMe<SceneAOInstanceAttributes>(otr.artObjInstConfig_,
                                                   artObjInstConfig_);
  }
  return *this;
}
SceneAttributes& SceneAttributes::operator=(SceneAttributes&& otr) noexcept {
  availableObjInstIDs_ = std::move(otr.availableObjInstIDs_);
  availableArtObjInstIDs_ = std::move(otr.availableArtObjInstIDs_);
  this->AbstractAttributes::operator=(
      std::move(static_cast<AbstractAttributes>(otr)));

  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ = editSubconfig<Configuration>("ao_instances");
  return *this;
}
std::string SceneAttributes::getObjectInfoInternal() const {
  // scene-specific info constants
  // default translation origin

  std::string res = Cr::Utility::formatString(
      "\nDefault Translation Origin,Default Lighting,Navmesh Handle,Semantic "
      "Scene Descriptor Handle,\n{},{},{},{}\n",
      getTranslationOriginName(getTranslationOrigin()), getLightingHandle(),
      getNavmeshHandle(), getSemanticSceneHandle());

  const SceneObjectInstanceAttributes::cptr stageInstance = getStageInstance();
  Cr::Utility::formatInto(res, res.size(), "Stage Instance Info :\n{}\n{}\n",
                          stageInstance->getObjectInfoHeader(),
                          stageInstance->getObjectInfo());
  // stage instance info
  int iter = 0;
  // object instance info
  const auto& objInstances = getObjectInstances();
  for (const auto& objInst : objInstances) {
    if (iter == 0) {
      ++iter;
      Cr::Utility::formatInto(res, res.size(), "Object Instance Info :\n{}\n",
                              objInst->getObjectInfoHeader());
    }
    Cr::Utility::formatInto(res, res.size(), "{}\n", objInst->getObjectInfo());
  }

  // articulated object instance info
  iter = 0;
  const auto& articulatedObjectInstances = getArticulatedObjectInstances();
  for (const auto& artObjInst : articulatedObjectInstances) {
    if (iter == 0) {
      ++iter;
      Cr::Utility::formatInto(res, res.size(),
                              "Articulated Object Instance Info :\n{}\n",
                              artObjInst->getObjectInfoHeader());
    }
    Cr::Utility::formatInto(res, res.size(), "{}\n",
                            artObjInst->getObjectInfo());
  }

  Cr::Utility::formatInto(res, res.size(),
                          "End of data for Scene Instance {}\n",
                          getSimplifiedHandle());
  return res;
}  // SceneAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
