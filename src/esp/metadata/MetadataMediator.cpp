// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataMediator.h"

namespace esp {
namespace metadata {

MetadataMediator::MetadataMediator(
    const std::string& _activeSceneDataset,
    const std::string& _physicsManagerAttributesPath)
    : activeSceneDataset_(_activeSceneDataset),
      currPhysicsManagerAttributes_(_physicsManagerAttributesPath) {
  buildAttributesManagers();
}  // MetadataMediator ctor
void MetadataMediator::buildAttributesManagers() {
  physicsAttributesManager_ = managers::PhysicsAttributesManager::create();

  sceneDatasetAttributesManager_ =
      managers::SceneDatasetAttributesManager::create(
          physicsAttributesManager_);

  // create scene dataset attributes
  createSceneDataset(activeSceneDataset_);
  // create physics attributes and pass on to sceneDatasetAttributesManager
  setCurrPhysicsAttributesHandle(currPhysicsManagerAttributes_);

}  // MetadataMediator::buildAttributesManagers

bool MetadataMediator::createPhysicsManagerAttributes(
    const std::string& _physicsManagerAttributesPath) {
  bool exists = physicsAttributesManager_->getObjectLibHasHandle(
      _physicsManagerAttributesPath);
  if (!exists) {
    auto physAttrs = physicsAttributesManager_->createObject(
        _physicsManagerAttributesPath, true);

    if (nullptr == physAttrs) {
      // not created, do not set name
      LOG(WARNING)
          << "MetadataMediator::createPhysicsManagerAttributes : Unknown  "
             "physics manager configuration file : "
          << physAttrs
          << " does not exist and is not able to be created.  Aborting.";
      return false;
    }
  }  // if dne then create
  return true;
}  // MetadataMediator::createPhysicsManagerAttributes

bool MetadataMediator::createSceneDataset(const std::string& sceneDatasetName,
                                          bool overwrite) {
  // see if exists
  bool exists =
      sceneDatasetAttributesManager_->getObjectLibHasHandle(sceneDatasetName);
  if (exists) {
    // check if not overwrite and exists already
    if (!overwrite) {
      LOG(WARNING) << "MetadataMediator::createSceneDataset : Scene Dataset "
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
    LOG(WARNING) << "MetadataMediator::createSceneDataset : Unknown dataset "
                 << sceneDatasetName
                 << " does not exist and is not able to be created.  Aborting.";
    return false;
  }
  // if not null then successfully created
  LOG(INFO) << "MetadataMediator::createSceneDataset : Dataset "
            << sceneDatasetName << " successfully created.";
  return true;
}  // MetadataMediator::createSceneDataset

bool MetadataMediator::setCurrPhysicsAttributesHandle(
    const std::string& _physicsManagerAttributesPath) {
  // first check if physics manager attributes exists, if so then set as current
  if (physicsAttributesManager_->getObjectLibHasHandle(
          _physicsManagerAttributesPath)) {
    LOG(INFO) << "MetadataMediator::setCurrPhysicsAttributesHandle : Old "
                 "physics manager attributes "
              << currPhysicsManagerAttributes_ << " changed to "
              << _physicsManagerAttributesPath << " successfully.";
    currPhysicsManagerAttributes_ = _physicsManagerAttributesPath;
    sceneDatasetAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        currPhysicsManagerAttributes_);
    return true;
  }
  bool success = createPhysicsManagerAttributes(_physicsManagerAttributesPath);
  // if successfully created, set default name to physics manager attributes in
  // SceneDatasetAttributesManager
  if (success) {
    currPhysicsManagerAttributes_ = _physicsManagerAttributesPath;
    sceneDatasetAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        currPhysicsManagerAttributes_);
    /// setCurrPhysicsManagerAttributesHandle
  }
  return success;

}  // MetadataMediator::setCurrPhysicsAttributesHandle

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
  bool success = createSceneDataset(sceneDatasetName);
  // if successfully created, set default name to access dataset attributes in
  // SceneDatasetAttributesManager
  if (success) {
    activeSceneDataset_ = sceneDatasetName;
  }
  return success;
}  // MetadataMediator::setActiveSceneDatasetName

}  // namespace metadata
}  // namespace esp
