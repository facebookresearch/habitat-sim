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

attributes::SceneAttributes::ptr MetadataMediator::getSceneAttributesByName(
    const std::string& sceneName) {
  // get current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
  // this should never happen
  if (nullptr == datasetAttr) {
    LOG(ERROR) << "MetadataMediator::getSceneAttributesByName : No dataset "
                  "specified/exists.";
    return nullptr;
  }
  // directory to look for attributes for this dataset
  const std::string dsDir = datasetAttr->getFileDirectory();
  // get appropriate attr managers for the current dataset
  managers::SceneAttributesManager::ptr dsSceneAttrMgr =
      datasetAttr->getSceneAttributesManager();
  managers::StageAttributesManager::ptr dsStageAttrMgr =
      datasetAttr->getStageAttributesManager();

  attributes::SceneAttributes::ptr sceneAttributes = nullptr;
  // sceneName can legally match any one of the following conditions :

  if (dsSceneAttrMgr->getObjectLibHasHandle(sceneName)) {
    // 1.  Existing, registered SceneAttributes in current active dataset.
    //    In this case the SceneAttributes is returned.
    LOG(INFO) << "MetadataMediator::getSceneAttributesByName : Query dataset : "
              << activeSceneDataset_
              << " for SceneAttributes named : " << sceneName << " successful.";
    sceneAttributes = dsSceneAttrMgr->getObjectCopyByHandle(sceneName);
  } else {
    const std::string sceneFilenameCandidate =
        dsSceneAttrMgr->getFormattedJSONFileName(sceneName);

    if (dsSceneAttrMgr->isValidFileName(sceneFilenameCandidate)) {
      // 2.  Existing, valid SceneAttributes file on disk, but not in dataset.
      //    If this is the case, then the SceneAttributes should be loaded,
      //    registered, added to the dataset and returned.
      LOG(INFO) << "MetadataMediator::getSceneAttributesByName : Dataset : "
                << activeSceneDataset_
                << " does not reference a SceneAttributes named  " << sceneName
                << " but a SceneAttributes config was found on disk, so "
                   "loading.  Will be named :"
                << sceneFilenameCandidate;
      sceneAttributes = dsSceneAttrMgr->createObjectFromJSONFile(
          sceneFilenameCandidate, true);
    } else if (dsStageAttrMgr->getObjectLibHasHandle(sceneName)) {
      // 3.  Existing, registered StageAttributes in current active dataset.
      //    In this case, a SceneAttributes is created amd registered using
      //    sceneName, referencing the StageAttributes of the same name; This
      //    sceneAttributes is returned.
      LOG(INFO) << "MetadataMediator::getSceneAttributesByName : Dataset : "
                << activeSceneDataset_
                << " has no SceneAttributes named : " << sceneName
                << " but StageAttributes found, so constructing "
                   "SceneAttributes with same name and adding to Dataset.";
      // create a new SceneAttributes, and give it a
      // SceneObjectInstanceAttributes for the stage with the same name.
      sceneAttributes = makeSceneAndReferenceStage(dsSceneAttrMgr, sceneName);

    } else {
      // 4.  Existing stage config/asset on disk, but not in current dataset.
      //    In this case, a stage attributes is loaded and registered, then
      //    added to current dataset, and then 3. is performed.
      LOG(INFO)
          << "MetadataMediator::getSceneAttributesByName : Dataset : "
          << activeSceneDataset_
          << " has no preloaded SceneAttributes or StageAttributes named : "
          << sceneName
          << " so loading/creating a new StageAttributes with this "
             "name, and then create a SceneAttributes with the same name "
             "that referneces this stage.";
      // create and register stage
      auto stageAttributes = dsStageAttrMgr->createObject(sceneName, true);
      // create a new SceneAttributes, and give it a
      // SceneObjectInstanceAttributes for the stage with the same name.
      sceneAttributes = makeSceneAndReferenceStage(dsSceneAttrMgr, sceneName);
    }
  }
  // make sure that all attributes referenced in scene attributes are loaded in
  // dataset, as well as the scene attributes itself.
  datasetAttr->addNewSceneInstanceToDataset(sceneAttributes);

  return sceneAttributes;

}  // MetadataMediator::getSceneAttributesByName

attributes::SceneAttributes::ptr MetadataMediator::makeSceneAndReferenceStage(
    const managers::SceneAttributesManager::ptr dsSceneAttrMgr,
    const std::string& sceneName) {
  // create scene attributes with passed name
  attributes::SceneAttributes::ptr sceneAttributes =
      dsSceneAttrMgr->createDefaultObject(sceneName, false);
  // create stage instance and set its name
  sceneAttributes->setStageInstance(
      dsSceneAttrMgr->createEmptyInstanceAttributes(sceneName));
  // register object
  dsSceneAttrMgr->registerObject(sceneAttributes);
  return sceneAttributes;
}

}  // namespace metadata
}  // namespace esp
