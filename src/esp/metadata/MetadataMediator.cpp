// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataMediator.h"

namespace esp {
namespace metadata {
void MetadataMediator::buildAttributesManagers() {
  physicsAttributesManager_ = managers::PhysicsAttributesManager::create();
  sceneDatasetAttributesManager_ =
      managers::SceneDatasetAttributesManager::create(
          physicsAttributesManager_);
  // create blank default attributes manager
  createDataset(activeSceneDataset_);
}  // MetadataMediator::buildAttributesManagers

bool MetadataMediator::createDataset(const std::string& sceneDatasetName,
                                     bool overwrite) {
  // see if exists
  bool exists =
      sceneDatasetAttributesManager_->getObjectLibHasHandle(sceneDatasetName);
  if (exists) {
    // check if not overwrite and exists already
    if (!overwrite) {
      LOG(WARNING) << "MetadataMediator::createDataset : Dataset "
                   << sceneDatasetName
                   << " already exists.  To reload and overwrite existing "
                      "data, set overwrite to true. Aborting.";
      return false;
    }
    // overwrite specified, make sure not locked
    sceneDatasetAttributesManager_->setLock(sceneDatasetName, false);
  }
  // by here dataset either does not exist or exists but is unlocked.
  auto datasetAttribs =
      sceneDatasetAttributesManager_->createObject(sceneDatasetName, true);
  if (nullptr == datasetAttribs) {
    // not created, do not set name
    LOG(WARNING) << "MetadataMediator::createDataset : Unknown dataset "
                 << sceneDatasetName
                 << " does not exist and is not able to be created.  Aborting.";
    return false;
  }
  // if not null then successfully created
  LOG(INFO) << "MetadataMediator::createDataset : Dataset " << sceneDatasetName
            << " successfully created.";
  return true;
}  // MetadataMediator::createDataset

bool MetadataMediator::setActiveSceneDatasetName(
    const std::string& sceneDatasetName) {
  // first check if dataset exists, if so then set default
  if (sceneDatasetAttributesManager_->getObjectLibHasHandle(sceneDatasetName)) {
    LOG(INFO)
        << "MetadataMediator::setActiveSceneDatasetName : Old active dataset "
        << activeSceneDataset_ << " changed to " << sceneDatasetName
        << " successfully.";
    activeSceneDataset_ = sceneDatasetName;
    return true;
  }
  // if does not exist, attempt to create it
  bool success = createDataset(sceneDatasetName);
  // if successfully created, set default name to access dataset attributes in
  // SceneDatasetAttributesManager
  if (success) {
    activeSceneDataset_ = sceneDatasetName;
  }
  return success;
}  // namespace metadata

}  // namespace metadata
}  // namespace esp
