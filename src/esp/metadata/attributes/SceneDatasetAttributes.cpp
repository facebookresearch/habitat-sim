// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneDatasetAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

SceneDatasetAttributes::SceneDatasetAttributes(
    const std::string& datasetName,
    const managers::PhysicsAttributesManager::ptr& physAttrMgr)
    : AbstractAttributes("SceneDatasetAttributes", datasetName) {
  assetAttributesManager_ = managers::AssetAttributesManager::create();
  lightLayoutAttributesManager_ =
      managers::LightLayoutAttributesManager::create();
  objectAttributesManager_ = managers::ObjectAttributesManager::create();
  objectAttributesManager_->setAssetAttributesManager(assetAttributesManager_);
  sceneAttributesManager_ = managers::SceneAttributesManager::create();
  stageAttributesManager_ = managers::StageAttributesManager::create(
      objectAttributesManager_, physAttrMgr);
}  // ctor

bool SceneDatasetAttributes::addNewSceneInstanceToDataset(
    const attributes::SceneAttributes::ptr& sceneInstance) {
  // info display message prefix
  std::string infoPrefix("Dataset : '");
  infoPrefix.append(getSimplifiedHandle()).append("' : ");

  const std::string sceneInstanceName = sceneInstance->getHandle();
  // verify stage in sceneInstance (required) exists in SceneDatasetAttributes,
  // and if not, add it.
  const auto& stageInstance = sceneInstance->getStageInstance();
  const std::string stageHandle = stageInstance->getHandle();
  const std::string fullStageName =
      getFullAttrNameFromStr(stageHandle, stageAttributesManager_);
  if (fullStageName == "") {
    ESP_DEBUG()
        << infoPrefix << "Stage Attributes '" << Mn::Debug::nospace
        << stageHandle << Mn::Debug::nospace
        << "' specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    stageAttributesManager_->createObject(stageHandle, true);
  } else {
    ESP_DEBUG() << infoPrefix << "Stage Attributes '" << Mn::Debug::nospace
                << stageHandle << Mn::Debug::nospace
                << "' specified in Scene Attributes exists in dataset library.";
  }

  // verify each object in sceneInstance exists in SceneDatasetAttributes
  auto objectInstances = sceneInstance->getObjectInstances();
  for (const auto& objInstance : objectInstances) {
    const std::string objHandle = objInstance->getHandle();
    const std::string fullObjHandle =
        getFullAttrNameFromStr(objHandle, objectAttributesManager_);
    if (fullObjHandle == "") {
      ESP_DEBUG() << infoPrefix << "Object Attributes '" << Mn::Debug::nospace
                  << objHandle << Mn::Debug::nospace
                  << "' specified in Scene Attributes but does not exist in "
                     "dataset, so creating.";
      objectAttributesManager_->createObject(objHandle, true);
    } else {
      ESP_DEBUG()
          << infoPrefix << "Object Attributes '" << Mn::Debug::nospace
          << objHandle << Mn::Debug::nospace
          << "' specified in Scene Attributes exists in dataset library.";
    }
  }

  // verify lighting
  std::string lightHandle = sceneInstance->getLightingHandle();
  // if lighthandle is empty, give handle same name as scene instance
  // should only be empty if not specified in scene instance config, or if scene
  // instance config is synthesized (i.e. when Simulator::reconfigure is called
  // with SimulatorConfiguration::activeSceneName being a stage)
  if (lightHandle.length() == 0) {
    lightHandle = sceneInstanceName;
  }

  const std::string fullLightLayoutAttrName =
      getFullAttrNameFromStr(lightHandle, lightLayoutAttributesManager_);
  if (fullLightLayoutAttrName == "") {
    ESP_DEBUG()
        << infoPrefix << "Lighting Layout Attributes '" << Mn::Debug::nospace
        << lightHandle << Mn::Debug::nospace
        << "' specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    lightLayoutAttributesManager_->createObject(lightHandle, true);
  } else {
    ESP_DEBUG() << infoPrefix << "Lighting Layout Attributes" << lightHandle
                << "specified in Scene Attributes exists in dataset library.";
  }

  const std::string fullSceneInstanceName =
      getFullAttrNameFromStr(sceneInstanceName, sceneAttributesManager_);

  // add scene attributes to scene attributes manager
  if (fullSceneInstanceName == "") {
    ESP_DEBUG() << infoPrefix << "Scene Attributes" << sceneInstanceName
                << "does not exist in dataset so adding.";
    sceneAttributesManager_->registerObject(sceneInstance);
  }
  return true;
}  // SceneDatasetAttributes::addSceneInstanceToDataset

std::pair<std::string, std::string> SceneDatasetAttributes::addNewValToMap(
    const std::string& key,
    const std::string& path,
    bool overwrite,
    std::map<std::string, std::string>& map,
    const std::string& descString) {
  // see if key is in map - if so, this means key is pointing to a
  // different value if not overwrite, modify key and add, returning entry.
  std::string newKey(key);
  auto mapSearch = map.find(newKey);
  if (mapSearch != map.end()) {
    // found key, mapsearch points to existing entry.
    // check if existing entry is desired entry
    if (mapSearch->second == path) {
      // appropriate entry found, return pair
      return *mapSearch;
    } else {
      // key found, points to a different entry
      if (!overwrite) {
        // modify existing key to be legal value
        int iter = 0;
        std::stringstream ss("");
        do {
          ss.str("");
          ss << key << "_" << iter++;
          newKey = ss.str();
        } while (map.count(newKey) > 0);
        ESP_WARNING()
            << descString << ": Provided key '" << Mn::Debug::nospace << key
            << Mn::Debug::nospace
            << "' already references a different value in "
               "map. Modifying key to be"
            << newKey
            << ". Set overwrite to true to overwrite existing entries.";
      } else {  // overwrite entry
        ESP_WARNING() << descString
                      << ": Warning : Overwriting existing map entry"
                      << map.at(newKey) << "at key" << newKey << "with value"
                      << path << ".";
      }  // overwrite or not
    }    // found entry is desired or not
  }      // key is found
  map[newKey] = path;
  return *(map.find(newKey));
}  // SceneDatasetAttributes::addNewValToMap

esp::gfx::LightSetup SceneDatasetAttributes::getNamedLightSetup(
    const std::string& lightSetupName) {
  auto lightLayoutAttrName = getLightSetupFullHandle(lightSetupName);
  if (lightLayoutAttrName == NO_LIGHT_KEY) {
    return esp::gfx::LightSetup{};
  }
  return lightLayoutAttributesManager_->createLightSetupFromAttributes(
      lightLayoutAttrName);

}  // SceneDatasetAttributes::getNamedLightSetup

attributes::StageAttributes::ptr
SceneDatasetAttributes::getNamedStageAttributesCopy(
    const std::string& stageAttrName) {
  // do a substring search to find actual stage attributes and return first
  // attributes found; if does not exist, name will be empty. return nullptr
  auto fullStageName =
      getFullAttrNameFromStr(stageAttrName, stageAttributesManager_);
  // fullStageName will be empty if not found
  if (fullStageName == "") {
    return nullptr;
  }
  return stageAttributesManager_->getObjectCopyByHandle(fullStageName);
}  // SceneDatasetAttributes::getNamedStageAttributesCopy

attributes::ObjectAttributes::ptr
SceneDatasetAttributes::getNamedObjectAttributesCopy(
    const std::string& objAttrName) {
  // do a substring search to find actual object attributes and return first
  // attributes found; if does not exist, name will be empty. return nullptr
  auto fullObjName =
      getFullAttrNameFromStr(objAttrName, objectAttributesManager_);
  // fullObjName will be empty if not found
  if (fullObjName == "") {
    return nullptr;
  }
  return objectAttributesManager_->getObjectCopyByHandle(fullObjName);
}  // SceneDatasetAttributes::getNamedObjectAttributesCopy

std::string SceneDatasetAttributes::getObjectInfoInternal() const {
  // provide a summary for all info for this scene dataset
  std::string res = "\n";
  // scene instances

  std::vector<std::string> sceneInstAttrInfoAra =
      sceneAttributesManager_->getObjectInfoStrings();
  res.append("\nScene Instances : ")
      .append(std::to_string(sceneInstAttrInfoAra.size() - 1))
      .append(" loaded.\n");

  for (const std::string& s : sceneInstAttrInfoAra) {
    res.append(s).append(1, '\n');
  }

  // stages
  std::vector<std::string> stageAttrInfoAra =
      stageAttributesManager_->getObjectInfoStrings();
  res.append("\nStage Templates : ")
      .append(std::to_string(stageAttrInfoAra.size() - 1))
      .append(" loaded.\n");
  for (const std::string& s : stageAttrInfoAra) {
    res.append(s).append(1, '\n');
  }

  // objects
  std::vector<std::string> objAttrInfoAra =
      objectAttributesManager_->getObjectInfoStrings();
  res.append("\nObject Templates : ")
      .append(std::to_string(objAttrInfoAra.size() - 1))
      .append(" loaded.\n");
  for (const std::string& s : objAttrInfoAra) {
    res.append(s).append(1, '\n');
  }

  // articulated objects
  res.append("\nArticulated Object Models : ")
      .append(std::to_string(articulatedObjPaths.size()))
      .append(" loaded.\n");

  for (const auto& item : articulatedObjPaths) {
    res.append(item.first).append(1, ',').append(item.second).append(1, '\n');
  }

  // lights
  std::vector<std::string> lightAttrInfoAra =
      lightLayoutAttributesManager_->getObjectInfoStrings();
  res.append("\nLighting Configurations : ")
      .append(std::to_string(lightAttrInfoAra.size() - 1))
      .append(" loaded.\n");
  for (const std::string& s : lightAttrInfoAra) {
    res.append(s).append(1, '\n');
  }

  // prims
  std::vector<std::string> primAttrInfoAra =
      assetAttributesManager_->getObjectInfoStrings();
  res.append("\nPrimitives Templates : ")
      .append(std::to_string(primAttrInfoAra.size() - 1))
      .append(" loaded.\n");
  for (const std::string& s : primAttrInfoAra) {
    res.append(s).append(1, '\n');
  }

  // navmesh
  res.append("\nNavmeshes : ")
      .append(std::to_string(navmeshMap_.size()))
      .append(" loaded.\n");
  for (const auto& item : navmeshMap_) {
    res.append(item.first).append(1, ',').append(item.second).append(1, '\n');
  }

  // SSD entries
  res.append("\nSemantic Scene Descriptors : ")
      .append(std::to_string(semanticSceneDescrMap_.size()))
      .append(" loaded.\n");
  for (const auto& item : semanticSceneDescrMap_) {
    res.append(item.first).append(1, ',').append(item.second).append(1, '\n');
  }

  return res;
}

std::string SceneDatasetAttributes::getDatasetSummaryHeader() {
  return "Dataset Name, Scene Instance Templates, Stage Templates, Object "
         "Templates, Articulated Object Paths, Lighting Templates, Primitive "
         "Templates, Navmesh Entries, Semantic Scene Descriptor Entries,";
}

std::string SceneDatasetAttributes::getDatasetSummary() const {
  std::string res{
      getSimplifiedHandle()
          .append(1, ',')
          .append(std::to_string(sceneAttributesManager_->getNumObjects()))
          .append(1, ',')
          .append(std::to_string(stageAttributesManager_->getNumObjects()))
          .append(1, ',')
          .append(std::to_string(objectAttributesManager_->getNumObjects()))
          .append(1, ',')
          .append(std::to_string(articulatedObjPaths.size()))
          .append(1, ',')
          .append(
              std::to_string(lightLayoutAttributesManager_->getNumObjects()))
          .append(1, ',')
          .append(std::to_string(assetAttributesManager_->getNumObjects()))
          .append(1, ',')
          .append(std::to_string(navmeshMap_.size()))
          .append(1, ',')
          .append(std::to_string(semanticSceneDescrMap_.size()))
          .append(1, ',')};
  return res;

}  // SceneDatasetAttributes::getDatasetSummary

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
