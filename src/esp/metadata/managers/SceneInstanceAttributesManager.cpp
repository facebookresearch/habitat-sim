// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneInstanceAttributesManager.h"

#include <Corrade/Utility/FormatStl.h>

#include <utility>
#include "esp/physics/RigidBase.h"

#include "esp/io/Json.h"

namespace esp {
namespace metadata {

using attributes::SceneAOInstanceAttributes;
using attributes::SceneInstanceAttributes;
using attributes::SceneObjectInstanceAttributes;

namespace managers {

SceneInstanceAttributes::ptr SceneInstanceAttributesManager::createObject(
    const std::string& sceneInstanceHandle,
    bool registerTemplate) {
  std::string msg;
  SceneInstanceAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      sceneInstanceHandle, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << msg << " Scene Instance Attributes created"
        << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // SceneInstanceAttributesManager::createObject

SceneInstanceAttributes::ptr
SceneInstanceAttributesManager::initNewObjectInternal(
    const std::string& sceneInstanceHandle,
    bool) {
  SceneInstanceAttributes::ptr newAttributes =
      this->constructFromDefault(sceneInstanceHandle);
  if (nullptr == newAttributes) {
    newAttributes = SceneInstanceAttributes::create(sceneInstanceHandle);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);
  // Set the baseline default before json is processed.
  newAttributes->setDefaultPbrShaderAttributesHandle(
      defaultPbrShaderAttributesHandle_);

  // any internal default configuration here
  return newAttributes;
}  // SceneInstanceAttributesManager::initNewObjectInternal

void SceneInstanceAttributesManager::setValsFromJSONDoc(
    SceneInstanceAttributes::ptr attribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string attribsDispName = attribs->getSimplifiedHandle();
  // Check for translation origin.  Default to unknown.
  attribs->setTranslationOrigin(getTranslationOriginVal(jsonConfig));

  // Check for stage instance existence in scene instance
  io::JsonGenericValue::ConstMemberIterator stageJSONIter =
      jsonConfig.FindMember("stage_instance");
  if (stageJSONIter != jsonConfig.MemberEnd()) {
    if (stageJSONIter->value.IsObject()) {
      attribs->setStageInstanceAttrs(
          createInstanceAttributesFromJSON(stageJSONIter->value));
    } else {
      // stage instance exists but is not a valid JSON Object
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Stage instance issue in Scene Instance `" << attribsDispName
          << "`: JSON cell `stage_instance` is not a valid JSON object.";
    }
  } else {
    // No stage_instance specified in SceneInstance configuration.
    // We expect a scene instance to be present always, except for the default
    // Scene Dataset that is empty.
    if (attribsDispName == "default_attributes") {
      // Default attributes is empty
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "No Stage Instance specified in Default Scene Instance, so "
             "setting empty/NONE Stage as Stage instance.";
      SceneObjectInstanceAttributes::ptr instanceAttrs =
          createEmptyInstanceAttributes("");
      // Set to use none stage
      instanceAttrs->setHandle("NONE");
      attribs->setStageInstanceAttrs(instanceAttrs);
    } else {
      // no stage instance exists in Scene Instance config JSON. This should not
      // happen and would indicate an error in the dataset.
      ESP_CHECK(false,
                "No JSON cell `stage_instance` specified in Scene Instance `"
                    << Mn::Debug::nospace << attribsDispName
                    << Mn::Debug::nospace
                    << "` so no Stage can be created for this Scene.");
    }
  }

  // Check for object instances existence
  io::JsonGenericValue::ConstMemberIterator objJSONIter =
      jsonConfig.FindMember("object_instances");
  if (objJSONIter != jsonConfig.MemberEnd()) {
    // object_instances tag exists
    if (objJSONIter->value.IsArray()) {
      const auto& objectArray = objJSONIter->value;
      for (rapidjson::SizeType i = 0; i < objectArray.Size(); ++i) {
        const auto& objCell = objectArray[i];
        if (objCell.IsObject()) {
          attribs->addObjectInstanceAttrs(
              createInstanceAttributesFromJSON(objCell));
        } else {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "Object instance issue in Scene Instance `" << attribsDispName
              << "` at idx : " << i
              << " : JSON cell within `object_instances` array is not a "
                 "valid JSON object, so skipping entry.";
        }
      }
    } else {
      // object_instances tag exists but is not an array. should warn (perhaps
      // error?)
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Object instances issue in Scene Instance `" << attribsDispName
          << "`: JSON cell `object_instances` is not a valid JSON "
             "array, so no object instances loaded.";
    }
  } else {
    // No object_instances tag exists in scene instance. Not necessarily a bad
    // thing, not all datasets have objects
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No Objects specified in Scene Instance `" << attribsDispName
        << "`: JSON cell with tag `object_instances` does not exist in Scene "
           "Instance.";
  }

  // Check for articulated object instances existence
  io::JsonGenericValue::ConstMemberIterator artObjJSONIter =
      jsonConfig.FindMember("articulated_object_instances");
  if (artObjJSONIter != jsonConfig.MemberEnd()) {
    // articulated_object_instances tag exists
    if (artObjJSONIter->value.IsArray()) {
      const auto& articulatedObjArray = artObjJSONIter->value;
      for (rapidjson::SizeType i = 0; i < articulatedObjArray.Size(); ++i) {
        const auto& artObjCell = articulatedObjArray[i];

        if (artObjCell.IsObject()) {
          attribs->addArticulatedObjectInstanceAttrs(
              createAOInstanceAttributesFromJSON(artObjCell));
        } else {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "Articulated Object specification error in Scene Instance `"
              << attribsDispName << "` at idx : " << i
              << ": JSON cell within `articulated_object_instances` array is "
                 "not a valid JSON object, so skipping entry.";
        }
      }
    } else {
      // articulated_object_instances tag exists but is not an array. should
      // warn (perhaps error?)
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Articulated Object instances issue in Scene "
             "InstanceScene Instance `"
          << attribsDispName
          << "`: JSON cell `articulated_object_instances` is not a valid JSON "
             "array, so no articulated object instances loaded.";
    }
  } else {
    // No articulated_object_instances tag exists in scene instance. Not
    // necessarily a bad thing, not all datasets have articulated objects
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No Articulated Objects specified for Scene Instance `"
        << attribsDispName
        << "`: JSON cell with tag `articulated_object_instances` does not "
           "exist in Scene Instance.";
  }

  // Check for PBR/IBL shader region-based configuration specifications
  // existence.

  io::JsonGenericValue::ConstMemberIterator pbrShaderRegionJSONIter =
      jsonConfig.FindMember("pbr_shader_region_configs");
  if ((pbrShaderRegionJSONIter != jsonConfig.MemberEnd()) &&
      (pbrShaderRegionJSONIter->value.IsObject())) {
    const auto& articulatedObjArray = artObjJSONIter->value;

    // pbr_shader_region_configs tag exists, and should be an object, holding
    // unique region names and the handle to the PbrShaderAttributes to use for
    // that region.

    // Tag should have the format of an array of key-value
    // pairs, where the key is some region identifier and the value is a
    // string representing the PbrShaderAttributes to use, as specified in the
    // PbrShaderAttributesManager.
    const auto& pbrShaderRegionHandles = pbrShaderRegionJSONIter->value;
    int count = 0;
    // iterate through objects
    for (rapidjson::Value::ConstMemberIterator it =
             pbrShaderRegionHandles.MemberBegin();
         it != pbrShaderRegionHandles.MemberEnd(); ++it) {
      // create attributes and set its name to be the tag in the JSON for the
      // individual light
      const std::string region = it->name.GetString();
      const std::string pbrHandle = it->value.GetString();
      attribs->addRegionPbrShaderAttributesHandle(region, pbrHandle);
    }

  } else {
    // No pbr_shader_region_configs tag exists in scene instance. Not
    // necessarily a bad thing, not all datasets have specified PBR/IBL
    // configs that deviate from the default.
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No Region-based PPR/IBL Shader configurations specified for Scene "
           "Instance `"
        << attribsDispName
        << "`: JSON cell with tag `pbr_shader_region_configs` does not exist "
           "in Scene Instance.";
  }

  std::string dfltShaderConfig = "";
  // Check for default shader config. This will be used for the PBR/IBL shading
  // of all objects and stages not specifically covered by any region-based
  // configs, if they exist.
  if (io::readMember<std::string>(jsonConfig, "default_pbr_shader_config",
                                  dfltShaderConfig)) {
    // Set the default PbrShaderAttributes handle to use for all Pbr rendering.
    attribs->setDefaultPbrShaderAttributesHandle(dfltShaderConfig);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No default_pbr_shader_config specified for Scene Instance `"
        << attribsDispName << "`.";
  }

  std::string dfltLighting = "";
  // Check for lighting instances
  if (io::readMember<std::string>(jsonConfig, "default_lighting",
                                  dfltLighting)) {
    // if "default lighting" is specified in scene json set value.
    attribs->setLightingHandle(dfltLighting);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No default_lighting specified for Scene Instance `"
        << attribsDispName << "`.";
  }

  std::string navmeshName = "";
  if (io::readMember<std::string>(jsonConfig, "navmesh_instance",
                                  navmeshName)) {
    // if "navmesh_instance" is specified in scene json set value.
    attribs->setNavmeshHandle(navmeshName);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No navmesh_instance specified for Scene Instance `"
        << attribsDispName << "`.";
  }

  std::string semanticDesc = "";
  // Check for Semantic Scene specification
  if (io::readMember<std::string>(jsonConfig, "semantic_scene_instance",
                                  semanticDesc)) {
    // if "semantic scene instance" is specified in scene json set value.
    attribs->setSemanticSceneHandle(semanticDesc);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No semantic_scene_instance specified for Scene Instance `"
        << attribsDispName << "`.";
  }
  // check for user defined attributes
  this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // SceneInstanceAttributesManager::setValsFromJSONDoc

SceneObjectInstanceAttributes::ptr
SceneInstanceAttributesManager::createInstanceAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SceneObjectInstanceAttributes::ptr instanceAttrs =
      createEmptyInstanceAttributes("");
  // populate attributes
  this->setAbstractObjectAttributesFromJson(instanceAttrs, jCell);
  return instanceAttrs;
}  // SceneInstanceAttributesManager::createInstanceAttributesFromJSON

SceneAOInstanceAttributes::ptr
SceneInstanceAttributesManager::createAOInstanceAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SceneAOInstanceAttributes::ptr instanceAttrs =
      createEmptyAOInstanceAttributes("");
  // populate attributes
  this->setAbstractObjectAttributesFromJson(instanceAttrs, jCell);

  // only used for articulated objects
  // fixed base
  // TODO remove this setter once datasets are updated and we only use
  // enum-backed values in scene instances
  io::jsonIntoSetter<bool>(
      jCell, "fixed_base", [instanceAttrs](bool fixed_base) {
        instanceAttrs->setBaseType(fixed_base ? "fixed" : "free");
      });

  // render mode
  this->setEnumStringFromJsonDoc(jCell, "render_mode", "AORenderModesMap", true,
                                 attributes::AORenderModesMap,
                                 [instanceAttrs](const std::string& val) {
                                   instanceAttrs->setRenderMode(val);
                                 });

  // base type
  this->setEnumStringFromJsonDoc(jCell, "base_type", "AOBaseTypeMap", true,
                                 attributes::AOBaseTypeMap,
                                 [instanceAttrs](const std::string& val) {
                                   instanceAttrs->setBaseType(val);
                                 });

  // inertia source
  this->setEnumStringFromJsonDoc(jCell, "inertia_source", "AOInertiaSourceMap",
                                 true, attributes::AOInertiaSourceMap,
                                 [instanceAttrs](const std::string& val) {
                                   instanceAttrs->setInertiaSource(val);
                                 });

  // link order
  this->setEnumStringFromJsonDoc(jCell, "link_order", "AOLinkOrderMap", true,
                                 attributes::AOLinkOrderMap,
                                 [instanceAttrs](const std::string& val) {
                                   instanceAttrs->setLinkOrder(val);
                                 });

  // only used for articulated objects
  // auto clamp joint limits
  io::jsonIntoSetter<bool>(
      jCell, "auto_clamp_joint_limits",
      [instanceAttrs](bool auto_clamp_joint_limits) {
        instanceAttrs->setAutoClampJointLimits(auto_clamp_joint_limits);
      });

  // only used for articulated objects
  // initial joint pose
  io::JsonGenericValue::ConstMemberIterator jntPoseJSONIter =
      jCell.FindMember("initial_joint_pose");
  if (jntPoseJSONIter != jCell.MemberEnd()) {
    if (jntPoseJSONIter->value.IsArray()) {
      std::vector<float> poseRes;
      // read values into vector
      io::readMember<float>(jCell, "initial_joint_pose", poseRes);
      int i = 0;
      for (const float& v : poseRes) {
        const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
        instanceAttrs->addInitJointPoseVal(key, v);
      }

    } else if (jntPoseJSONIter->value.IsObject()) {
      // load values into map
      io::readMember<std::map<std::string, float>>(
          jCell, "initial_joint_pose", instanceAttrs->copyIntoInitJointPose());
    } else {
      ESP_WARNING() << ": Unknown format for "
                       "initial_joint_pose specified for instance"
                    << instanceAttrs->getHandle()
                    << "in Scene Instance File, so no values are set.";
    }
  }
  // only used for articulated objects
  // initial joint velocities
  io::JsonGenericValue::ConstMemberIterator jntVelJSONIter =
      jCell.FindMember("initial_joint_velocities");
  if (jntVelJSONIter != jCell.MemberEnd()) {
    if (jntVelJSONIter->value.IsArray()) {
      std::vector<float> poseRes;
      // read values into vector
      io::readMember<float>(jCell, "initial_joint_velocities", poseRes);
      int i = 0;
      for (const float& v : poseRes) {
        const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
        instanceAttrs->addInitJointVelocityVal(key, v);
      }

    } else if (jntVelJSONIter->value.IsObject()) {
      // load values into map
      io::readMember<std::map<std::string, float>>(
          jCell, "initial_joint_velocities",
          instanceAttrs->copyIntoInitJointVelocities());
    } else {
      ESP_WARNING() << ": Unknown format for "
                       "initial_joint_velocities specified for instance"
                    << instanceAttrs->getHandle()
                    << "in Scene Instance File, so no values are set.";
    }
  }
  return instanceAttrs;

}  // SceneInstanceAttributesManager::createAOInstanceAttributesFromJSON

void SceneInstanceAttributesManager::setAbstractObjectAttributesFromJson(
    const attributes::SceneObjectInstanceAttributes::ptr& instanceAttrs,
    const io::JsonGenericValue& jCell) {
  // template handle describing stage/object instance
  io::jsonIntoConstSetter<std::string>(
      jCell, "template_name",
      [instanceAttrs](const std::string& template_name) {
        instanceAttrs->setHandle(template_name);
      });

  // Check for translation origin override for a particular instance.  Default
  // to unknown, which will mean use scene instance-level default.
  instanceAttrs->setTranslationOrigin(getTranslationOriginVal(jCell));

  // set specified shader type value.  May be Unspecified, which means the
  // default value specified in the stage or object attributes will be used.
  // instanceAttrs->setShaderType(getShaderTypeFromJsonDoc(jCell));

  // shader type
  this->setEnumStringFromJsonDoc(jCell, "shader_type", "ShaderTypeNamesMap",
                                 true, attributes::ShaderTypeNamesMap,
                                 [instanceAttrs](const std::string& val) {
                                   instanceAttrs->setShaderType(val);
                                 });

  // motion type of object.  Ignored for stage.
  std::string tmpVal = "";
  if (io::readMember<std::string>(jCell, "motion_type", tmpVal)) {
    // motion type tag was found, perform check - first convert to lowercase
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    auto found = attributes::MotionTypeNamesMap.find(strToLookFor);
    if (found != attributes::MotionTypeNamesMap.end()) {
      // only set value if specified in json
      instanceAttrs->setMotionType(strToLookFor);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << ": motion_type value in json  : `" << tmpVal << "`|`"
          << strToLookFor
          << "` does not map to a valid physics::MotionType value, so not "
             "setting instance motion type value.";
    }
  }

  // translation from origin
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jCell, "translation",
      [instanceAttrs](const Magnum::Vector3& translation) {
        instanceAttrs->setTranslation(translation);
      });

  // orientation TODO : support euler angles too?
  io::jsonIntoConstSetter<Magnum::Quaternion>(
      jCell, "rotation", [instanceAttrs](const Magnum::Quaternion& rotation) {
        instanceAttrs->setRotation(rotation);
      });

  // uniform scaling for instance
  io::jsonIntoSetter<double>(jCell, "uniform_scale",
                             [instanceAttrs](double uniform_scale) {
                               instanceAttrs->setUniformScale(uniform_scale);
                             });

  // non-uniform scaling for instance
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jCell, "non_uniform_scale",
      [instanceAttrs](const Magnum::Vector3& non_uniform_scale) {
        instanceAttrs->setNonUniformScale(non_uniform_scale);
      });

  // whether particular instance is visible or not - only modify if actually
  // present in instance json
  io::jsonIntoSetter<bool>(
      jCell, "is_instance_visible", [instanceAttrs](bool is_instance_visible) {
        instanceAttrs->setIsInstanceVisible(is_instance_visible);
      });

  // mass scaling for instance
  io::jsonIntoSetter<double>(jCell, "mass_scale",
                             [instanceAttrs](double mass_scale) {
                               instanceAttrs->setMassScale(mass_scale);
                             });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(instanceAttrs, jCell);

}  // SceneInstanceAttributesManager::setAbstractObjectAttributesFromJson

std::string SceneInstanceAttributesManager::getTranslationOriginVal(
    const io::JsonGenericValue& jsonDoc) const {
  // Check for translation origin.  Default to unknown.
  std::string transOrigin = getTranslationOriginName(
      attributes::SceneInstanceTranslationOrigin::Unknown);
  std::string tmpTransOriginVal = "";
  if (io::readMember<std::string>(jsonDoc, "translation_origin",
                                  tmpTransOriginVal)) {
    // translation_origin tag was found, perform check - first convert to
    // lowercase
    std::string strToLookFor =
        Cr::Utility::String::lowercase(tmpTransOriginVal);
    auto found = attributes::InstanceTranslationOriginMap.find(strToLookFor);
    if (found != attributes::InstanceTranslationOriginMap.end()) {
      transOrigin = std::move(tmpTransOriginVal);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << ": translation_origin value in json :`" << tmpTransOriginVal
          << "`|`" << strToLookFor
          << "` does not map to a valid SceneInstanceTranslationOrigin "
             "value, so defaulting translation origin to "
             "SceneInstanceTranslationOrigin::Unknown.";
    }
  }
  return transOrigin;
}  // SceneInstanceAttributesManager::getTranslationOriginVal

int SceneInstanceAttributesManager::registerObjectFinalize(
    SceneInstanceAttributes::ptr sceneInstanceAttributes,
    const std::string& sceneInstanceAttributesHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by sceneInstanceAttributesHandle, or the next
  // available ID if not found.
  int datasetTemplateID = this->addObjectToLibrary(
      std::move(sceneInstanceAttributes), sceneInstanceAttributesHandle);
  return datasetTemplateID;
}  // SceneInstanceAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
