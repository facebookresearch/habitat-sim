// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneAttributesManager.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

using std::placeholders::_1;

namespace esp {
namespace metadata {

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
    LOG(INFO) << msg << " scene instance attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // SceneAttributesManager::createObject

SceneAttributes::ptr SceneAttributesManager::initNewObjectInternal(
    const std::string& sceneInstanceHandle) {
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
  const std::string attribsName = attribs->getHandle();
  // Check for stage instance existance
  if ((jsonConfig.HasMember("stage instance")) &&
      (jsonConfig["stage instance"].IsObject())) {
    attribs->setStageInstance(
        createInstanceAttributesFromJSON(jsonConfig["stage instance"]));
  } else {
    LOG(WARNING) << "SceneAttributesManager::setValsFromJSONDoc : No Stage "
                    "specified for scene "
                 << attribsName << ", or specification error.";
  }
  // Check for object instances existance
  if ((jsonConfig.HasMember("object instances")) &&
      (jsonConfig["object instances"].IsArray())) {
    const auto& objectArray = jsonConfig["object instances"];
    for (rapidjson::SizeType i = 0; i < objectArray.Size(); i++) {
      const auto& objCell = objectArray[i];
      if (objCell.IsObject()) {
        attribs->addObjectInstance(createInstanceAttributesFromJSON(objCell));
      } else {
        LOG(WARNING) << "SceneAttributesManager::setValsFromJSONDoc : Object "
                        "specification error in scene "
                     << attribsName << " at idx : " << i << ".";
      }
    }
  } else {
    LOG(WARNING) << "SceneAttributesManager::setValsFromJSONDoc : No Objects "
                    "specified for scene "
                 << attribsName << ", or specification error.";
  }
  std::string dfltLighting = "";
  if (io::jsonIntoVal<std::string>(jsonConfig, "default lighting",
                                   dfltLighting)) {
    // if "default lighting" is specified in scene json set value.
    attribs->setLightingHandle(dfltLighting);
  } else {
    LOG(WARNING)
        << "SceneAttributesManager::setValsFromJSONDoc : No default lighting "
           "specified for scene "
        << attribsName << ".";
  }

  std::string navmeshName = "";
  if (io::jsonIntoVal<std::string>(jsonConfig, "navmesh instance",
                                   navmeshName)) {
    // if "navmesh instance" is specified in scene json set value.
    attribs->setNavmeshHandle(navmeshName);
  } else {
    LOG(WARNING) << "SceneAttributesManager::setValsFromJSONDoc : No navmesh "
                    "specified for scene "
                 << attribsName << ".";
  }

  std::string semanticDesc = "";
  if (io::jsonIntoVal<std::string>(jsonConfig, "semantic scene instance",
                                   semanticDesc)) {
    // if "semantic scene instance" is specified in scene json set value.
    attribs->setSemanticSceneHandle(semanticDesc);
  } else {
    LOG(WARNING) << "SceneAttributesManager::setValsFromJSONDoc : No Semantic "
                    "Scene Description specified for scene "
                 << attribsName << ".";
  }
}  // SceneAttributesManager::setValsFromJSONDoc

SceneObjectInstanceAttributes::ptr
SceneAttributesManager::createInstanceAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SceneObjectInstanceAttributes::ptr instanceAttrs =
      SceneObjectInstanceAttributes::create("");
  // template handle describing stage/object instance
  io::jsonIntoConstSetter<std::string>(
      jCell, "template handle",
      std::bind(&SceneObjectInstanceAttributes::setHandle, instanceAttrs, _1));

  // motion type of object.  Ignored for stage.  TODO : veify is valid motion
  // type using standard mechanism of static map comparison.
  io::jsonIntoConstSetter<std::string>(
      jCell, "motiontype",
      std::bind(&SceneObjectInstanceAttributes::setMotionType, instanceAttrs,
                _1));

  // translation from origin
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jCell, "translation",
      std::bind(&SceneObjectInstanceAttributes::setTranslation, instanceAttrs,
                _1));

  // orientation TODO : support euler angles too?
  io::jsonIntoConstSetter<Magnum::Quaternion>(
      jCell, "rotation",
      std::bind(&SceneObjectInstanceAttributes::setRotation, instanceAttrs,
                _1));

  return instanceAttrs;

}  // SceneAttributesManager::createInstanceAttributesFromJSON

int SceneAttributesManager::registerObjectFinalize(
    SceneAttributes::ptr sceneAttributes,
    const std::string& sceneAttributesHandle) {
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
