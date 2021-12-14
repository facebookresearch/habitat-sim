// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneInstanceAttributes.h"
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
  setShaderType(getShaderTypeName(ObjectInstanceShaderType::Unspecified));

  // defaults to unknown/undefined
  setMotionType(getMotionTypeName(esp::physics::MotionType::UNDEFINED));
  // set to no rotation
  setRotation(Mn::Quaternion(Mn::Math::IdentityInit));
  setTranslation(Mn::Vector3());
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
  return "Translation XYZ,Rotation W[XYZ],Motion Type,Shader Type,Uniform "
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

void SceneObjectInstanceAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // map "handle" to "template_name" key in json
  writeValueToJson("handle", "template_name", jsonObj, allocator);
  writeValueToJson("translation", jsonObj, allocator);
  if (getTranslationOrigin() != SceneInstanceTranslationOrigin::Unknown) {
    writeValueToJson("translation_origin", jsonObj, allocator);
  }
  writeValueToJson("rotation", jsonObj, allocator);
  // map "is_instance_visible" to boolean only if not -1, otherwise don't save
  int visSet = getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // set JSON value based on visSet (0,1) as bool
    auto jsonVal = io::toJsonValue(static_cast<bool>(visSet), allocator);
    jsonObj.AddMember("is_instance_visible", jsonVal, allocator);
  }
  if (getMotionType() != esp::physics::MotionType::UNDEFINED) {
    writeValueToJson("motion_type", jsonObj, allocator);
  }
  writeValueToJson("shader_type", jsonObj, allocator);
  writeValueToJson("uniform_scale", jsonObj, allocator);
  writeValueToJson("mass_scale", jsonObj, allocator);

  // take care of child class valeus, if any exist
  writeValuesToJsonInternal(jsonObj, allocator);

}  // SceneObjectInstanceAttributes::writeValuesToJson

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

void SceneAOInstanceAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("fixed_base", jsonObj, allocator);
  writeValueToJson("auto_clamp_joint_limits", jsonObj, allocator);

  // write out map where key is joint tag, and value is joint pose value.
  if (!initJointPose_.empty()) {
    io::addMember(jsonObj, "initial_joint_pose", initJointPose_, allocator);
  }

  // write out map where key is joint tag, and value is joint angular vel value.
  if (!initJointVelocities_.empty()) {
    io::addMember(jsonObj, "initial_joint_velocities", initJointVelocities_,
                  allocator);
  }

}  // SceneAOInstanceAttributes::writeValuesToJsonInternal

SceneInstanceAttributes::SceneInstanceAttributes(const std::string& handle)
    : AbstractAttributes("SceneInstanceAttributes", handle) {
  // defaults to no lights
  setLightingHandle(NO_LIGHT_KEY);
  // defaults to asset local
  setTranslationOrigin(
      getTranslationOriginName(SceneInstanceTranslationOrigin::AssetLocal));
  setNavmeshHandle("");
  setSemanticSceneHandle("");
  // get refs to internal subconfigs for object and ao instances
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ =
      editSubconfig<Configuration>("articulated_object_instances");
}

SceneInstanceAttributes::SceneInstanceAttributes(
    const SceneInstanceAttributes& otr)
    : AbstractAttributes(otr),
      availableObjInstIDs_(otr.availableObjInstIDs_),
      availableArtObjInstIDs_(otr.availableArtObjInstIDs_) {
  // get refs to internal subconfigs for object and ao instances
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  copySubconfigIntoMe<SceneObjectInstanceAttributes>(otr.objInstConfig_,
                                                     objInstConfig_);
  artObjInstConfig_ =
      editSubconfig<Configuration>("articulated_object_instances");
  copySubconfigIntoMe<SceneAOInstanceAttributes>(otr.artObjInstConfig_,
                                                 artObjInstConfig_);
}
SceneInstanceAttributes::SceneInstanceAttributes(
    SceneInstanceAttributes&& otr) noexcept
    : AbstractAttributes(std::move(static_cast<AbstractAttributes>(otr))),
      availableObjInstIDs_(std::move(otr.availableObjInstIDs_)),
      availableArtObjInstIDs_(std::move(otr.availableArtObjInstIDs_)) {
  // get refs to internal subconfigs for object and ao instances
  // originals were moved over so should retain full derived class
  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ =
      editSubconfig<Configuration>("articulated_object_instances");
}

void SceneInstanceAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  if (getTranslationOrigin() != SceneInstanceTranslationOrigin::Unknown) {
    writeValueToJson("translation_origin", jsonObj, allocator);
  }
  writeValueToJson("default_lighting", jsonObj, allocator);
  writeValueToJson("navmesh_instance", jsonObj, allocator);
  writeValueToJson("semantic_scene_instance", jsonObj, allocator);
}  // SceneInstanceAttributes::writeValuesToJson

void SceneInstanceAttributes::writeSubconfigsToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // build list of json objs from subconfigs describing object instances and
  // articulated object instances
  // object instances
  auto objCfgIterPair = objInstConfig_->getSubconfigIterator();
  io::JsonGenericValue objInstArray(rapidjson::kArrayType);
  for (auto& cfgIter = objCfgIterPair.first; cfgIter != objCfgIterPair.second;
       ++cfgIter) {
    objInstArray.PushBack(cfgIter->second->writeToJsonObject(allocator),
                          allocator);
  }
  jsonObj.AddMember("object_instances", objInstArray, allocator);

  // ao instances
  auto AObjCfgIterPair = artObjInstConfig_->getSubconfigIterator();
  io::JsonGenericValue AObjInstArray(rapidjson::kArrayType);
  for (auto& cfgIter = AObjCfgIterPair.first; cfgIter != AObjCfgIterPair.second;
       ++cfgIter) {
    AObjInstArray.PushBack(cfgIter->second->writeToJsonObject(allocator),
                           allocator);
  }
  jsonObj.AddMember("articulated_object_instances", AObjInstArray, allocator);

  // save stage_instance subconfig
  io::JsonGenericValue subObj =
      getStageInstance()->writeToJsonObject(allocator);
  jsonObj.AddMember("stage_instance", subObj, allocator);

  // iterate through other subconfigs using standard handling
  // do not resave ObjectInstances and AObjInstances
  // iterate through subconfigs
  // pair of begin/end const iterators to all subconfigurations
  auto cfgIterPair = getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    if ((cfgIter->first == "object_instances") ||
        (cfgIter->first == "stage_instance") ||
        (cfgIter->first == "articulated_object_instances")) {
      continue;
    }
    // only save if subconfig has entries
    if (cfgIter->second->getNumEntries() > 0) {
      rapidjson::GenericStringRef<char> name{cfgIter->first.c_str()};
      io::JsonGenericValue subObj =
          cfgIter->second->writeToJsonObject(allocator);
      jsonObj.AddMember(name, subObj, allocator);
    } else {
      ESP_VERY_VERBOSE()
          << "Unitialized/empty Subconfig in Configuration @ key ["
          << cfgIter->first
          << "], so nothing will be written to JSON for this key.";
    }
  }  // iterate through all configurations

}  // SceneInstanceAttributes::writeSubconfigsToJson

SceneInstanceAttributes& SceneInstanceAttributes::operator=(
    const SceneInstanceAttributes& otr) {
  if (this != &otr) {
    this->AbstractAttributes::operator=(otr);
    availableObjInstIDs_ = otr.availableObjInstIDs_;
    availableArtObjInstIDs_ = otr.availableArtObjInstIDs_;
    // get refs to internal subconfigs for object and ao instances
    objInstConfig_ = editSubconfig<Configuration>("object_instances");
    copySubconfigIntoMe<SceneObjectInstanceAttributes>(otr.objInstConfig_,
                                                       objInstConfig_);
    artObjInstConfig_ =
        editSubconfig<Configuration>("articulated_object_instances");
    copySubconfigIntoMe<SceneAOInstanceAttributes>(otr.artObjInstConfig_,
                                                   artObjInstConfig_);
  }
  return *this;
}
SceneInstanceAttributes& SceneInstanceAttributes::operator=(
    SceneInstanceAttributes&& otr) noexcept {
  availableObjInstIDs_ = std::move(otr.availableObjInstIDs_);
  availableArtObjInstIDs_ = std::move(otr.availableArtObjInstIDs_);
  this->AbstractAttributes::operator=(
      std::move(static_cast<AbstractAttributes>(otr)));

  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ =
      editSubconfig<Configuration>("articulated_object_instances");
  return *this;
}
std::string SceneInstanceAttributes::getObjectInfoInternal() const {
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
}  // SceneInstanceAttributes::getObjectInfoInternal

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
