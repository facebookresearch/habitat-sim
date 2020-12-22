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
  if (!stageAttributesManager_->getObjectLibHasHandle(stageHandle)) {
    LOG(INFO)
        << "SceneDatasetAttributes::addNewSceneInstanceToDataset : Dataset : "
        << datasetName << " : Stage Attributes " << stageHandle
        << " specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    stageAttributesManager_->createObject(stageHandle, true);
  }

  // verify each object in sceneInstance exists in SceneDatasetAttributes
  auto objectInstances = sceneInstance->getObjectInstances();
  for (const auto& objInstance : objectInstances) {
    const std::string objHandle = objInstance->getHandle();
    if (!objectAttributesManager_->getObjectLibHasHandle(objHandle)) {
      LOG(INFO)
          << "SceneDatasetAttributes::addNewSceneInstanceToDataset : Dataset : "
          << datasetName << " : Object Attributes " << objHandle
          << " specified in Scene Attributes but does not exist in dataset, so "
             "creating.";
      objectAttributesManager_->createObject(objHandle, true);
    }
  }

  // verify lighting
  const std::string lightHandle = sceneInstance->getLightingHandle();
  if (!lightLayoutAttributesManager_->getObjectLibHasHandle(lightHandle)) {
    LOG(INFO)
        << "SceneDatasetAttributes::addNewSceneInstanceToDataset : Dataset : "
        << datasetName << " : Lighting Layout Attributes " << lightHandle
        << " specified in Scene Attributes but does not exist in dataset, so "
           "creating.";
    lightLayoutAttributesManager_->createObject(lightHandle, true);
  }

  // add scene attributes to scene attributes manager
  if (!sceneAttributesManager_->getObjectLibHasHandle(sceneInstanceName)) {
    LOG(INFO)
        << "SceneDatasetAttributes::addNewSceneInstanceToDataset : Dataset : "
        << datasetName << " : Scene Attributes " << sceneInstanceName
        << " does not exist in dataset so adding.";
    sceneAttributesManager_->registerObject(sceneInstance);
  }
  return true;
}  // SceneDatasetAttributes::addSceneInstanceToDataset

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
