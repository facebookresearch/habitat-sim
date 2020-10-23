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
  createDataset(activeDataset_);
}  // MetadataMediator::buildAttributesManagers

bool MetadataMediator::createDataset(const std::string& datasetName,
                                     bool overwrite) {
  // see if exists
  bool exists =
      sceneDatasetAttributesManager_->getObjectLibHasHandle(datasetName);
  if (exists) {
    // check if not overwrite and exists already
    if (!overwrite) {
      LOG(WARNING) << "MetadataMediator::setActiveDataset : Dataset "
                   << datasetName
                   << " already exists.  To reload and overwrite existing "
                      "data, set overwrite to true. Aborting.";
      return false;
    }
    // overwrite specified, make sure not locked
    sceneDatasetAttributesManager_->setLock(datasetName, false);
  }
  // by here dataset either does not exist or exists but is unlocked.
  auto datasetAttribs =
      sceneDatasetAttributesManager_->createObject(datasetName, true);
  if (nullptr == datasetAttribs) {
    // not created, do not set name
    LOG(WARNING) << "MetadataMediator::createDataset : Unknown dataset "
                 << datasetName
                 << " does not exist and is not able to be created.  Aborting.";
    return false;
  }
  // if not null then successfully created
  LOG(INFO) << "MetadataMediator::createDataset : Dataset " << datasetName
            << " successfully created.";
  return true;
}  // MetadataMediator::createDataset

bool MetadataMediator::setActiveDatasetName(const std::string& datasetName) {
  // first check if dataset exists, if so then set default
  if (sceneDatasetAttributesManager_->getObjectLibHasHandle(datasetName)) {
    LOG(INFO) << "MetadataMediator::setActiveDatasetName : Old active dataset "
              << activeDataset_ << " changed to " << datasetName
              << " successfully.";
    activeDataset_ = datasetName;
    return true;
  }
  // if does not exist, attempt to create it
  bool success = createDataset(datasetName);
  // if successfully created, set default name to access dataset attributes in
  // SceneDatasetAttributesManager
  if (success) {
    activeDataset_ = datasetName;
  }
  return success;
}  // namespace metadata

}  // namespace metadata
}  // namespace esp
