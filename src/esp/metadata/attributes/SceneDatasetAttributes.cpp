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
  const std::string datasetName = getHandle();
  const std::string sceneInstanceName = sceneInstance->getHandle();
  // verify stage in sceneInstance (required) exists in SceneDatasetAttributes,
  // and if not, add it.
  const auto& stageInstance = sceneInstance->getStageInstance();
  const std::string stageHandle = stageInstance->getHandle();
  // info display message prefix
  const std::string infoPrefix(
      "SceneDatasetAttributes::addNewSceneInstanceToDataset : Dataset : '" +
      datasetName + "' :");
  if (!stageAttributesManager_->getObjectLibHasHandle(stageHandle)) {
    LOG(INFO)
        << infoPrefix << " Stage Attributes '" << stageHandle
        << "' specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    stageAttributesManager_->createObject(stageHandle, true);
  } else {
    LOG(INFO) << infoPrefix << " Stage Attributes '" << stageHandle
              << "' specified in Scene Attributes exists in dataset library.";
  }

  // verify each object in sceneInstance exists in SceneDatasetAttributes
  auto objectInstances = sceneInstance->getObjectInstances();
  for (const auto& objInstance : objectInstances) {
    const std::string objHandle = objInstance->getHandle();
    if (!objectAttributesManager_->getObjectLibHasHandle(objHandle)) {
      LOG(INFO) << infoPrefix << " Object Attributes '" << objHandle
                << "' specified in Scene Attributes but does not exist in "
                   "dataset, so creating.";
      objectAttributesManager_->createObject(objHandle, true);
    } else {
      LOG(INFO) << infoPrefix << " Object Attributes '" << objHandle
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
  if (!lightLayoutAttributesManager_->getObjectLibHasHandle(lightHandle)) {
    LOG(INFO)
        << infoPrefix << "Lighting Layout Attributes '" << lightHandle
        << "' specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    lightLayoutAttributesManager_->createObject(lightHandle, true);
  } else {
    LOG(INFO) << infoPrefix << " Lighting Layout Attributes " << lightHandle
              << " specified in Scene Attributes exists in dataset library.";
  }

  // add scene attributes to scene attributes manager
  if (!sceneAttributesManager_->getObjectLibHasHandle(sceneInstanceName)) {
    LOG(INFO) << infoPrefix << " Scene Attributes " << sceneInstanceName
              << " does not exist in dataset so adding.";
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
  // different value if not overwrite, modify key and add, returning
  std::string newKey(key);
  auto mapSearch = map.find(newKey);
  if (mapSearch != map.end()) {
    // found key, mapsearch points to existing entry.
    // check if existing entry is desired entry
    if (mapSearch->second.compare(path) == 0) {
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
        LOG(WARNING)
            << descString << " : Provided key '" << key
            << "' already references a different value in "
               "map. Modifying key to be "
            << newKey
            << ". Set overwrite to true to overwrite existing entries.";
      } else {  // overwrite entry
        LOG(WARNING) << descString
                     << " : Warning : Overwriting existing map entry "
                     << map.at(newKey) << " at key " << newKey << " with value "
                     << path << ".";
      }  // overwrite or not
    }    // found entry is desired or not
  }      // key is found
  map[newKey] = path;
  return *(map.find(newKey));
}  // SceneDatasetAttributes::addNewValToMap

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
