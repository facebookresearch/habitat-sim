// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneInstanceAttributes.h"

#include <utility>
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
  // don't override attributes-specified visibility. - sets int value to
  // ID_UNDEFINED
  clearIsInstanceVisible();
  // defaults to unknown so that obj instances use scene instance setting
  setTranslationOrigin(
      getTranslationOriginName(SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  setUniformScale(1.0);
  setNonUniformScale({1.0, 1.0, 1.0});
  setMassScale(1.0);
}

std::string SceneObjectInstanceAttributes::getObjectInfoHeaderInternal() const {
  return "Translation XYZ,Rotation W[XYZ],Motion Type,Shader Type,Uniform "
         "Scale,Non-Uniform Scale,Mass Scale,Translation Origin," +
         getSceneObjInstanceInfoHeaderInternal();
}

std::string SceneObjectInstanceAttributes::getObjectInfoInternal() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{}", getAsString("translation"),
      getAsString("rotation"), getMotionTypeName(getMotionType()),
      getShaderTypeName(getShaderType()), getAsString("uniform_scale"),
      getAsString("non_uniform_scale"), getAsString("mass_scale"),
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
  if (getTranslation() != Mn::Vector3()) {
    writeValueToJson("translation", jsonObj, allocator);
  }
  if (getTranslationOrigin() != SceneInstanceTranslationOrigin::Unknown) {
    writeValueToJson("translation_origin", jsonObj, allocator);
  }
  if (getRotation() != Mn::Quaternion(Mn::Math::IdentityInit)) {
    writeValueToJson("rotation", jsonObj, allocator);
  }
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
  if (getShaderType() != ObjectInstanceShaderType::Unspecified) {
    writeValueToJson("shader_type", jsonObj, allocator);
  }
  if (getUniformScale() != 1.0f) {
    writeValueToJson("uniform_scale", jsonObj, allocator);
  }
  if (getNonUniformScale() != Mn::Vector3(1.0, 1.0, 1.0)) {
    writeValueToJson("non_uniform_scale", jsonObj, allocator);
  }
  if (getMassScale() != 1.0) {
    writeValueToJson("mass_scale", jsonObj, allocator);
  }

  // take care of child class valeus, if any exist
  writeValuesToJsonInternal(jsonObj, allocator);

}  // SceneObjectInstanceAttributes::writeValuesToJson

SceneAOInstanceAttributes::SceneAOInstanceAttributes(const std::string& handle)
    : SceneObjectInstanceAttributes(handle, "SceneAOInstanceAttributes") {
  // set default auto clamp values (only used for articulated object)
  setAutoClampJointLimits(false);

  // Set the instance base type to be unspecified - if not set in instance json,
  // use ao_config value
  setBaseType(getAOBaseTypeName(ArticulatedObjectBaseType::Unspecified));
  // Set the instance source for the interia calculation to be unspecified - if
  // not set in instance json, use ao_config value
  setInertiaSource(
      getAOInertiaSourceName(ArticulatedObjectInertiaSource::Unspecified));
  // Set the instance link order to use as unspecified - if not set in instance
  // json, use ao_config value
  setLinkOrder(getAOLinkOrderName(ArticulatedObjectLinkOrder::Unspecified));
  // Set render mode to be unspecified - if not set in instance json, use
  // ao_config value
  setRenderMode(getAORenderModeName(ArticulatedObjectRenderMode::Unspecified));
}

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal()
    const {
  std::string infoHdr{"Base Type,Inertia Source,Link Order,Render Mode,"};
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

  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},{},{},{},",
                          getAOBaseTypeName(getBaseType()),
                          getAOInertiaSourceName(getInertiaSource()),
                          getAOLinkOrderName(getLinkOrder()),
                          getAORenderModeName(getRenderMode()));
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
  if (getBaseType() != ArticulatedObjectBaseType::Unspecified) {
    writeValueToJson("base_type", jsonObj, allocator);
  }
  if (getInertiaSource() != ArticulatedObjectInertiaSource::Unspecified) {
    writeValueToJson("inertia_source", jsonObj, allocator);
  }
  if (getLinkOrder() != ArticulatedObjectLinkOrder::Unspecified) {
    writeValueToJson("link_order", jsonObj, allocator);
  }
  if (getRenderMode() != ArticulatedObjectRenderMode::Unspecified) {
    writeValueToJson("render_mode", jsonObj, allocator);
  }

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
  pbrShaderRegionConfigHandles_ =
      editSubconfig<Configuration>("pbr_shader_region_configs");
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
  // Don't need to copy into, since pbrShaderRegionConfigHandles_ is not a
  // hierarchy
  pbrShaderRegionConfigHandles_ =
      editSubconfig<Configuration>("pbr_shader_region_configs");
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
  pbrShaderRegionConfigHandles_ =
      editSubconfig<Configuration>("pbr_shader_region_configs");
}

std::map<std::string, std::string>
SceneInstanceAttributes::getRegionPbrShaderAttributesHandles() const {
  // iterate through subconfig entries, casting appropriately and adding to
  // map if cast successful
  std::map<std::string, std::string> res{};
  int numValues = pbrShaderRegionConfigHandles_->getNumValues();
  if (numValues == 0) {
    return res;
  }
  // get begin/end pair of iterators for the values in the
  // pbrShaderRegionConfig
  auto pbrShdrRegionIter = pbrShaderRegionConfigHandles_->getValuesIterator();
  for (auto objIter = pbrShdrRegionIter.first;
       objIter != pbrShdrRegionIter.second; ++objIter) {
    // verify only string values
    if (objIter->second.getType() == core::config::ConfigStoredType::String) {
      res.emplace(
          std::make_pair(objIter->first, objIter->second.getAsString()));
    }
  }
  return res;
}  // SceneInstanceAttributes::getRegionPbrShaderAttributesHandles()

void SceneInstanceAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  if (getTranslationOrigin() != SceneInstanceTranslationOrigin::Unknown) {
    writeValueToJson("translation_origin", jsonObj, allocator);
  }
  writeValueToJson("default_lighting", jsonObj, allocator);
  writeValueToJson("navmesh_instance", jsonObj, allocator);
  writeValueToJson("semantic_scene_instance", jsonObj, allocator);
  writeValueToJson("default_pbr_shader_config", jsonObj, allocator);
}  // SceneInstanceAttributes::writeValuesToJson

void SceneInstanceAttributes::writeSubconfigsToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  // build list of JSON objs from subconfigs describing object instances and
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

  // save pbr_shader_region_configs subconfig
  io::JsonGenericValue pbrRegion =
      pbrShaderRegionConfigHandles_->writeToJsonObject(allocator);
  jsonObj.AddMember("pbr_shader_region_configs", pbrRegion, allocator);

  // iterate through other subconfigs using standard handling
  // do not resave ObjectInstances, AObjInstances, or any of the predefined
  // subconfigs.
  auto cfgIterPair = getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    if ((cfgIter->first == "object_instances") ||
        (cfgIter->first == "stage_instance") ||
        (cfgIter->first == "pbr_shader_region_configs") ||
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
    // Don't need to copy into, since pbrShaderRegionConfigHandles_ is not a
    // hierarchy, but a flat subconfig
    pbrShaderRegionConfigHandles_ =
        editSubconfig<Configuration>("pbr_shader_region_configs");
  }
  return *this;
}
SceneInstanceAttributes& SceneInstanceAttributes::operator=(
    SceneInstanceAttributes&& otr) noexcept {
  availableObjInstIDs_ = std::move(otr.availableObjInstIDs_);
  availableArtObjInstIDs_ = std::move(otr.availableArtObjInstIDs_);

  this->AbstractAttributes::operator=(static_cast<AbstractAttributes&&>(otr));

  objInstConfig_ = editSubconfig<Configuration>("object_instances");
  artObjInstConfig_ =
      editSubconfig<Configuration>("articulated_object_instances");
  pbrShaderRegionConfigHandles_ =
      editSubconfig<Configuration>("pbr_shader_region_configs");
  return *this;
}
std::string SceneInstanceAttributes::getObjectInfoInternal() const {
  // scene-specific info constants
  // default translation origin

  std::string res = Cr::Utility::formatString(
      "\nDefault Translation Origin,Default Lighting,Default PBR Shader "
      "Config,Navmesh Handle,Semantic "
      "Scene Descriptor Handle,\n{},{},{},{},{}\n",
      getTranslationOriginName(getTranslationOrigin()), getLightingHandle(),
      getDefaultPbrShaderAttributesHandle(), getNavmeshHandle(),
      getSemanticSceneHandle());

  // stage instance info
  const SceneObjectInstanceAttributes::cptr stageInstance = getStageInstance();
  Cr::Utility::formatInto(res, res.size(), "Stage Instance Info :\n{}\n{}\n",
                          stageInstance->getObjectInfoHeader(),
                          stageInstance->getObjectInfo());

  int iter = 0;

  // region-based PBR shader config info
  const auto& pbrShaderConfigRegionInfo = getRegionPbrShaderAttributesHandles();
  for (const auto& pbrCfg : pbrShaderConfigRegionInfo) {
    if (iter == 0) {
      ++iter;
      Cr::Utility::formatInto(
          res, res.size(),
          "Per Region PBR/IBL Shader Configuration Handles :\nRegion,Handle\n");
    }
    Cr::Utility::formatInto(res, res.size(), "{},{}\n", pbrCfg.first,
                            pbrCfg.second);
  }  // for pbrShaderConfigRegionInfo elements

  iter = 0;
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
