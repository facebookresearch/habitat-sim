// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneInstanceAttributes.h"
#include <utility>

#include "AbstractObjectAttributes.h"
#include "ArticulatedObjectAttributes.h"
#include "esp/physics/RigidBase.h"

namespace esp {
namespace metadata {
namespace attributes {

////////////////////////////////
// SceneObjectInstanceAttributes
SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle,
    const std::string& type)
    : AbstractAttributes(type, handle) {
  // default to unknown for object instances, to use attributes-specified
  // defaults
  initTranslated("shader_type",
                 getShaderTypeName(ObjectInstanceShaderType::Unspecified));

  // defaults to unknown/undefined
  initTranslated("motion_type",
                 getMotionTypeName(esp::physics::MotionType::UNDEFINED));
  // set to no rotation or translation
  init("rotation", Mn::Quaternion(Mn::Math::IdentityInit));
  init("translation", Mn::Vector3());
  // don't override attributes-specified visibility. - sets int value to
  // ID_UNDEFINED
  init("is_instance_visible", ID_UNDEFINED);
  // defaults to unknown so that obj instances use scene instance setting
  initTranslated(
      "translation_origin",
      getTranslationOriginName(SceneInstanceTranslationOrigin::Unknown));
  // set default multiplicative scaling values
  init("uniform_scale", 1.0);
  init("non_uniform_scale", Mn::Vector3{1.0, 1.0, 1.0});
  init("mass_scale", 1.0);
  init("apply_scale_to_mass", true);
}

SceneObjectInstanceAttributes::SceneObjectInstanceAttributes(
    const std::string& handle,
    const std::shared_ptr<AbstractObjectAttributes>& baseObjAttribs)
    : SceneObjectInstanceAttributes(handle) {
  // This constructor is for internally generated SceneObjectInstanceAttributes,
  // to correspond to an object that is being created programmatically and saved
  // to a scene instance.
  // Handle is set via init in base class, which would not be written out to
  // file if we did not explicitly set it.
  // NOTE : this will not call a virtual override
  // (SceneAOInstanceAttributes::setHandle) of AbstractAttributes::setHandle due
  // to virtual dispatch not being available in constructor
  setHandle(handle);
  // set appropriate fields from abstract object attributes
  // Not initialize, since these are not default values

  // Need to verify that the baseObjAttribs values are not defaults before we
  // set these values.

  if (!baseObjAttribs->isDefaultVal("shader_type")) {
    setShaderType(getShaderTypeName(baseObjAttribs->getShaderType()));
  }
  if (!baseObjAttribs->isDefaultVal("is_visible")) {
    setIsInstanceVisible(baseObjAttribs->getIsVisible());
  }

  // set nonuniform scale to match attributes scale
  if (!baseObjAttribs->isDefaultVal("scale")) {
    setNonUniformScale(baseObjAttribs->getScale());
  }
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
  writeValueToJson("translation", jsonObj, allocator);
  writeValueToJson("translation_origin", jsonObj, allocator);
  writeValueToJson("rotation", jsonObj, allocator);

  // map "is_instance_visible" to boolean only if not -1, otherwise don't save
  int visSet = getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // set JSON value based on visSet (0,1) as bool
    auto jsonVal = io::toJsonValue(static_cast<bool>(visSet), allocator);
    jsonObj.AddMember("is_instance_visible", jsonVal, allocator);
  }
  writeValueToJson("motion_type", jsonObj, allocator);
  writeValueToJson("shader_type", jsonObj, allocator);
  writeValueToJson("uniform_scale", jsonObj, allocator);
  writeValueToJson("non_uniform_scale", jsonObj, allocator);
  writeValueToJson("apply_scale_to_mass", jsonObj, allocator);
  writeValueToJson("mass_scale", jsonObj, allocator);
  // take care of child class values, if any exist
  writeValuesToJsonInternal(jsonObj, allocator);

}  // SceneObjectInstanceAttributes::writeValuesToJson

////////////////////////////////
// SceneAOInstanceAttributes

SceneAOInstanceAttributes::SceneAOInstanceAttributes(const std::string& handle)
    : SceneObjectInstanceAttributes(handle, "SceneAOInstanceAttributes") {
  // initialize default auto clamp values (only used for articulated object)
  init("auto_clamp_joint_limits", false);

  // Set the instance base type to be unspecified - if not set in instance json,
  // use ao_config value
  initTranslated("base_type",
                 getAOBaseTypeName(ArticulatedObjectBaseType::Unspecified));
  // Set the instance source for the inertia calculation to be unspecified - if
  // not set in instance json, use ao_config value
  initTranslated(
      "inertia_source",
      getAOInertiaSourceName(ArticulatedObjectInertiaSource::Unspecified));
  // Set the instance link order to use as unspecified - if not set in instance
  // json, use ao_config value
  initTranslated("link_order",
                 getAOLinkOrderName(ArticulatedObjectLinkOrder::Unspecified));
  // Set render mode to be unspecified - if not set in instance json, use
  // ao_config value
  initTranslated("render_mode",
                 getAORenderModeName(ArticulatedObjectRenderMode::Unspecified));
  editSubconfig<Configuration>("initial_joint_pose");
  editSubconfig<Configuration>("initial_joint_velocities");
}

SceneAOInstanceAttributes::SceneAOInstanceAttributes(
    const std::string& handle,
    const std::shared_ptr<ArticulatedObjectAttributes>& aObjAttribs)
    : SceneAOInstanceAttributes(handle) {
  // This constructor is for internally generated SceneAOInstanceAttributes, to
  // correspond to an object that is being created programmatically and saved to
  // a scene instance.
  // Handle is set via init in base class, which would not be written out to
  // file if we did not explicitly set it.
  // NOTE : this will not call a virtual override
  // (SceneAOInstanceAttributes::setHandle) of AbstractAttributes::setHandle due
  // to virtual dispatch not being available in constructor
  setHandle(handle);

  // Should not initialize these values but set them, since these are not
  // default values, but from an existing AO attributes.

  // Need to verify that the aObjAttribs values are not defaults before we
  // set these values.
  // Set shader type to use aObjAttribs value
  if (!aObjAttribs->isDefaultVal("shader_type")) {
    setShaderType(getShaderTypeName(aObjAttribs->getShaderType()));
  }
  if (!aObjAttribs->isDefaultVal("is_visible")) {
    setIsInstanceVisible(aObjAttribs->getIsVisible());
  }
  if (!aObjAttribs->isDefaultVal("base_type")) {
    // Set the instance base type to use aObjAttribs value
    setBaseType(getAOBaseTypeName(aObjAttribs->getBaseType()));
  }
  if (!aObjAttribs->isDefaultVal("inertia_source")) {
    // Set the instance source for the inertia calculation to use aObjAttribs
    // value
    setInertiaSource(getAOInertiaSourceName(aObjAttribs->getInertiaSource()));
  }
  if (!aObjAttribs->isDefaultVal("link_order")) {
    // Set the instance link order to use aObjAttribs value
    setLinkOrder(getAOLinkOrderName(aObjAttribs->getLinkOrder()));
  }
  if (!aObjAttribs->isDefaultVal("render_mode")) {
    // Set render mode to use aObjAttribs value
    setRenderMode(getAORenderModeName(aObjAttribs->getRenderMode()));
  }
}

std::string SceneAOInstanceAttributes::getSceneObjInstanceInfoHeaderInternal()
    const {
  std::string infoHdr{"Base Type,Inertia Source,Link Order,Render Mode,"};
  const auto initJointPose = getInitJointPose();
  for (uint32_t iter = 0; iter < initJointPose.size(); ++iter) {
    Cr::Utility::formatInto(infoHdr, infoHdr.size(), "Init Pose {},", iter);
  }
  const auto initJointVel = getInitJointVelocities();
  for (uint32_t iter = 0; iter < initJointPose.size(); ++iter) {
    Cr::Utility::formatInto(infoHdr, infoHdr.size(), "Init Vel {},", iter);
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
  const auto initJointPose = getInitJointPose();
  for (const auto& it : initJointPose) {
    Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},",
                            std::to_string(it));
  }
  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "],[");
  const auto initJointVel = getInitJointVelocities();
  for (const auto& it : initJointVel) {
    Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "{},",
                            std::to_string(it));
  }
  Cr::Utility::formatInto(initPoseStr, initPoseStr.size(), "]");

  return initPoseStr;
}  // SceneAOInstanceAttributes::getSceneObjInstanceInfoInternal()

void SceneAOInstanceAttributes::writeValuesToJsonInternal(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("base_type", jsonObj, allocator);
  writeValueToJson("inertia_source", jsonObj, allocator);
  writeValueToJson("link_order", jsonObj, allocator);
  writeValueToJson("render_mode", jsonObj, allocator);
  writeValueToJson("auto_clamp_joint_limits", jsonObj, allocator);

}  // SceneAOInstanceAttributes::writeValuesToJsonInternal

////////////////////////////////
// SceneInstanceAttributes

SceneInstanceAttributes::SceneInstanceAttributes(const std::string& handle)
    : AbstractAttributes("SceneInstanceAttributes", handle) {
  // defaults to no lights
  init("default_lighting", NO_LIGHT_KEY);
  // defaults to asset local
  init("translation_origin",
       getTranslationOriginName(SceneInstanceTranslationOrigin::AssetLocal));
  init("navmesh_instance", "");
  init("semantic_scene_instance", "");
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
    : AbstractAttributes(std::move(static_cast<AbstractAttributes&&>(otr))),
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
    if (objIter->second.getType() == core::config::ConfigValType::String) {
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
