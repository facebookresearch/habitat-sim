// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributesManager.h"

#include <Corrade/Utility/FormatStl.h>
#include "esp/metadata/MetadataUtils.h"
#include "esp/physics/RigidBase.h"

#include "esp/io/Json.h"

namespace esp {
namespace metadata {

using attributes::SceneAOInstanceAttributes;
using attributes::SceneAttributes;
using attributes::SceneObjectInstanceAttributes;

namespace managers {

SceneAttributes::ptr SceneAttributesManager::createObject(
    const std::string& sceneInstanceHandle,
    bool registerTemplate) {
  std::string msg;
  SceneAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      sceneInstanceHandle, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG() << msg << "scene instance attributes created"
                << (registerTemplate ? "and registered." : ".");
  }
  return attrs;
}  // SceneAttributesManager::createObject

SceneAttributes::ptr SceneAttributesManager::initNewObjectInternal(
    const std::string& sceneInstanceHandle,
    bool) {
  SceneAttributes::ptr newAttributes =
      this->constructFromDefault(sceneInstanceHandle);
  if (nullptr == newAttributes) {
    newAttributes = SceneAttributes::create(sceneInstanceHandle);
  }
  // attempt to set source directory if exists
  this->setFileDirectoryFromHandle(newAttributes);

  // any internal default configuration here
  return newAttributes;
}  // SceneAttributesManager::initNewObjectInternal

void SceneAttributesManager::setValsFromJSONDoc(
    SceneAttributes::ptr attribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string attribsDispName = attribs->getSimplifiedHandle();
  // Check for translation origin.  Default to unknown.
  attribs->setTranslationOrigin(getTranslationOriginVal(jsonConfig));

  // Check for stage instance existence
  if ((jsonConfig.HasMember("stage_instance")) &&
      (jsonConfig["stage_instance"].IsObject())) {
    attribs->setStageInstance(
        createInstanceAttributesFromJSON(jsonConfig["stage_instance"]));
  } else {
    ESP_WARNING() << "No Stage specified for scene" << attribsDispName
                  << ", or specification error.";
  }

  // Check for object instances existence
  if (jsonConfig.HasMember("object_instances")) {
    if (jsonConfig["object_instances"].IsArray()) {
      const auto& objectArray = jsonConfig["object_instances"];
      for (rapidjson::SizeType i = 0; i < objectArray.Size(); i++) {
        const auto& objCell = objectArray[i];
        if (objCell.IsObject()) {
          attribs->addObjectInstance(createInstanceAttributesFromJSON(objCell));
        } else {
          ESP_WARNING() << "Object specification error in scene"
                        << attribsDispName << "at idx :" << i << ".";
        }
      }
    } else {
      ESP_WARNING() << "No Objects specified for scene" << attribsDispName
                    << ", or specification error.";
    }
  }

  // Check for articulated object instances existence
  if ((jsonConfig.HasMember("articulated_object_instances")) &&
      (jsonConfig["articulated_object_instances"].IsArray())) {
    const auto& articulatedObjArray =
        jsonConfig["articulated_object_instances"];
    for (rapidjson::SizeType i = 0; i < articulatedObjArray.Size(); i++) {
      const auto& artObjCell = articulatedObjArray[i];

      if (artObjCell.IsObject()) {
        attribs->addArticulatedObjectInstance(
            createAOInstanceAttributesFromJSON(artObjCell));
      } else {
        ESP_WARNING() << "Articulated Object specification error in scene"
                      << attribsDispName << "at idx :" << i << ".";
      }
    }
  } else {
    ESP_WARNING() << "No Articulated Objects specified for scene"
                  << attribsDispName << ", or specification error.";
  }

  std::string dfltLighting = "";
  if (io::readMember<std::string>(jsonConfig, "default_lighting",
                                  dfltLighting)) {
    // if "default lighting" is specified in scene json set value.
    attribs->setLightingHandle(dfltLighting);
  } else {
    ESP_WARNING() << "No default_lighting specified for scene"
                  << attribsDispName << ".";
  }

  std::string navmeshName = "";
  if (io::readMember<std::string>(jsonConfig, "navmesh_instance",
                                  navmeshName)) {
    // if "navmesh_instance" is specified in scene json set value.
    attribs->setNavmeshHandle(navmeshName);
  } else {
    ESP_WARNING() << "No navmesh_instance specified for scene"
                  << attribsDispName << ".";
  }

  std::string semanticDesc = "";
  if (io::readMember<std::string>(jsonConfig, "semantic_scene_instance",
                                  semanticDesc)) {
    // if "semantic scene instance" is specified in scene json set value.
    attribs->setSemanticSceneHandle(semanticDesc);
  } else {
    ESP_WARNING() << "No semantic_scene_instance specified for scene"
                  << attribsDispName << ".";
  }
  // check for user defined attributes
  this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // SceneAttributesManager::setValsFromJSONDoc

SceneObjectInstanceAttributes::ptr
SceneAttributesManager::createInstanceAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SceneObjectInstanceAttributes::ptr instanceAttrs =
      createEmptyInstanceAttributes("");
  // populate attributes
  this->loadAbstractObjectAttributesFromJson(instanceAttrs, jCell);
  return instanceAttrs;
}  // SceneAttributesManager::createInstanceAttributesFromJSON

SceneAOInstanceAttributes::ptr
SceneAttributesManager::createAOInstanceAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SceneAOInstanceAttributes::ptr instanceAttrs =
      createEmptyAOInstanceAttributes("");
  // populate attributes
  this->loadAbstractObjectAttributesFromJson(instanceAttrs, jCell);

  // only used for articulated objects
  // fixed base
  io::jsonIntoSetter<bool>(jCell, "fixed_base",
                           [instanceAttrs](bool fixed_base) {
                             instanceAttrs->setFixedBase(fixed_base);
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
  if (jCell.HasMember("initial_joint_pose")) {
    if (jCell["initial_joint_pose"].IsArray()) {
      std::vector<float> poseRes;
      // read values into vector
      io::readMember<float>(jCell, "initial_joint_pose", poseRes);
      int i = 0;
      for (const float& v : poseRes) {
        const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
        instanceAttrs->addInitJointPoseVal(key, v);
      }

    } else if (jCell["initial_joint_pose"].IsObject()) {
      // load values into map
      io::readMember<std::map<std::string, float>>(
          jCell, "initial_joint_pose", instanceAttrs->copyIntoInitJointPose());
    } else {
      ESP_WARNING()
          << "SceneAttributesManager::"
             "createAOInstanceAttributesFromJSON : Unknown format for "
             "initial_joint_pose specified for instance"
          << instanceAttrs->getHandle()
          << "in Scene Instance File, so no values are set.";
    }
  }
  // only used for articulated objects
  // initial joint velocities
  if (jCell.HasMember("initial_joint_velocities")) {
    if (jCell["initial_joint_velocities"].IsArray()) {
      std::vector<float> poseRes;
      // read values into vector
      io::readMember<float>(jCell, "initial_joint_velocities", poseRes);
      int i = 0;
      for (const float& v : poseRes) {
        const std::string key = Cr::Utility::formatString("joint_{:.02d}", i++);
        instanceAttrs->addInitJointVelocityVal(key, v);
      }

    } else if (jCell["initial_joint_velocities"].IsObject()) {
      // load values into map
      io::readMember<std::map<std::string, float>>(
          jCell, "initial_joint_velocities",
          instanceAttrs->copyIntoInitJointVelocities());
    } else {
      ESP_WARNING()
          << "SceneAttributesManager::"
             "createAOInstanceAttributesFromJSON : Unknown format for "
             "initial_joint_velocities specified for instance"
          << instanceAttrs->getHandle()
          << "in Scene Instance File, so no values are set.";
    }
  }
  return instanceAttrs;

}  // SceneAttributesManager::createAOInstanceAttributesFromJSON

void SceneAttributesManager::loadAbstractObjectAttributesFromJson(
    const attributes::SceneObjectInstanceAttributes::ptr& instanceAttrs,
    const io::JsonGenericValue& jCell) const {
  // template handle describing stage/object instance
  io::jsonIntoConstSetter<std::string>(
      jCell, "template_name",
      [instanceAttrs](const std::string& template_name) {
        instanceAttrs->setHandle(template_name);
      });

  // Check for translation origin override for a particular instance.  Default
  // to unknown, which will mean use scene instance-level default.
  instanceAttrs->setTranslationOrigin(getTranslationOriginVal(jCell));

  // set specified shader type value.  May be Unknown, which means the default
  // value specified in the stage or object attributes will be used.
  instanceAttrs->setShaderType(getShaderTypeFromJsonDoc(jCell));

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
      ESP_WARNING()
          << "::createInstanceAttributesFromJSON : motion_type value "
             "in json  : `"
          << tmpVal << "|" << strToLookFor
          << "` does not map to a valid physics::MotionType value, so "
             "not setting instance motion type value.";
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
  // mass scaling for instance
  io::jsonIntoSetter<double>(jCell, "mass_scale",
                             [instanceAttrs](double mass_scale) {
                               instanceAttrs->setMassScale(mass_scale);
                             });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(instanceAttrs, jCell);

}  // SceneAttributesManager::loadAbstractObjectAttributesFromJson

std::string SceneAttributesManager::getTranslationOriginVal(
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
      ESP_WARNING()
          << "::getTranslationOriginVal : translation_origin value in json  "
             ": `"
          << tmpTransOriginVal << "|" << strToLookFor
          << "` does not map to a valid "
             "SceneInstanceTranslationOrigin value, so defaulting "
             "translation origin to SceneInstanceTranslationOrigin::Unknown.";
    }
  }
  return transOrigin;
}  // SceneAttributesManager::getTranslationOriginVal

int SceneAttributesManager::registerObjectFinalize(
    SceneAttributes::ptr sceneAttributes,
    const std::string& sceneAttributesHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by sceneAttributesHandle, or the next available ID
  // if not found.
  int datasetTemplateID =
      this->addObjectToLibrary(sceneAttributes, sceneAttributesHandle);
  return datasetTemplateID;
}  // SceneAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
