// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataMediator.h"

namespace esp {
namespace metadata {

MetadataMediator::MetadataMediator(const sim::SimulatorConfiguration& cfg)
    : activeSceneDataset_(cfg.sceneDatasetConfigFile),
      currPhysicsManagerAttributes_(cfg.physicsConfigFile),
      simConfig_(cfg) {
  buildAttributesManagers();
  setSimulatorConfiguration(simConfig_);
}  // MetadataMediator ctor (SimulatorConfiguration)

void MetadataMediator::buildAttributesManagers() {
  physicsAttributesManager_ = managers::PhysicsAttributesManager::create();

  sceneDatasetAttributesManager_ =
      managers::SceneDatasetAttributesManager::create(
          physicsAttributesManager_);

  // should always have default dataset
  createSceneDataset("default");
  // should always have default physicsManagerAttributesPath
  createPhysicsManagerAttributes(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH);
  // after this setSimulatorConfiguration will be called
}  // MetadataMediator::buildAttributesManagers

bool MetadataMediator::setSimulatorConfiguration(
    const sim::SimulatorConfiguration& cfg) {
  simConfig_ = cfg;

  // set current active dataset name - if unchanged, does nothing
  bool success = setActiveSceneDatasetName(simConfig_.sceneDatasetConfigFile);
  if (!success) {
    // something failed about setting up active scene dataset
    LOG(ERROR) << "MetadataMediator::setSimulatorConfiguration : Some error "
                  "prevented current scene dataset name to be changed to "
               << simConfig_.sceneDatasetConfigFile;
    return false;
  }

  // set active physics attributes handle - if unchanged, does nothing
  success = setCurrPhysicsAttributesHandle(simConfig_.physicsConfigFile);
  if (!success) {
    // something failed about setting up physics attributes
    LOG(ERROR) << "MetadataMediator::setSimulatorConfiguration : Some error "
                  "prevented current physics attributes to "
               << simConfig_.physicsConfigFile;
    return false;
  }

  // get a ref to current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
  // pass relevant config values to the current dataset
  if (datasetAttr != nullptr) {
    datasetAttr->setCurrCfgVals(simConfig_.sceneLightSetup,
                                simConfig_.frustumCulling);
  } else {
    LOG(ERROR) << "MetadataMediator::setSimulatorConfiguration : No active "
                  "dataset exists or has been specified. Aborting";
    return false;
  }
  LOG(INFO) << "MetadataMediator::setSimulatorConfiguration : Set new "
               "simulator config for scene/stage : "
            << simConfig_.activeSceneName
            << " and dataset : " << simConfig_.sceneDatasetConfigFile
            << " which "
            << (activeSceneDataset_.compare(
                    simConfig_.sceneDatasetConfigFile) == 0
                    ? "is currently active dataset."
                    : "is NOT active dataset (THIS IS PROBABLY AN ERROR.)");
  return true;
}  // MetadataMediator::setSimulatorConfiguration

bool MetadataMediator::createPhysicsManagerAttributes(
    const std::string& _physicsManagerAttributesPath) {
  bool exists = physicsAttributesManager_->getObjectLibHasHandle(
      _physicsManagerAttributesPath);
  if (!exists) {
    auto physAttrs = physicsAttributesManager_->createObject(
        _physicsManagerAttributesPath, true);

    if (physAttrs == nullptr) {
      // something failed during creation process.
      LOG(WARNING)
          << "MetadataMediator::createPhysicsManagerAttributes : Unknown  "
             "physics manager configuration file : "
          << _physicsManagerAttributesPath
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
  if (datasetAttribs == nullptr) {
    // not created, do not set name
    LOG(WARNING) << "MetadataMediator::createSceneDataset : Unknown dataset "
                 << sceneDatasetName
                 << " does not exist and is not able to be created.  Aborting.";
    return false;
  }
  // if not null then successfully created
  LOG(INFO) << "MetadataMediator::createSceneDataset : Dataset "
            << sceneDatasetName << " successfully created.";
  // lock dataset to prevent accidental deletion
  sceneDatasetAttributesManager_->setLock(sceneDatasetName, true);
  return true;
}  // MetadataMediator::createSceneDataset

bool MetadataMediator::removeSceneDataset(const std::string& sceneDatasetName) {
  // First check if SceneDatasetAttributes exists
  if (!sceneDatasetAttributesManager_->getObjectLibHasHandle(
          sceneDatasetName)) {
    // DNE, do nothing
    LOG(INFO)
        << "MetadataMediator::removeSceneDataset : SceneDatasetAttributes "
        << sceneDatasetName << " does not exist. Aborting.";
    return false;
  }

  // Next check if is current activeSceneDataset_, and if so skip with warning
  if (sceneDatasetName.compare(activeSceneDataset_)) {
    LOG(WARNING) << "MetadataMediator::removeSceneDataset : Cannot remove "
                    "active SceneDatasetAttributes "
                 << sceneDatasetName
                 << ".  Switch to another dataset before removing.";
    return false;
  }

  // Now force unlock and remove requested SceneDatasetAttributes- there should
  // be no SceneDatasetAttributes set to undeletable
  sceneDatasetAttributesManager_->setLock(sceneDatasetName, false);
  auto delDataset =
      sceneDatasetAttributesManager_->removeObjectByHandle(sceneDatasetName);
  // if failed here, probably means SceneDatasetAttributes was set to
  // undeletable, return message and false
  if (delDataset == nullptr) {
    LOG(WARNING)
        << "MetadataMediator::removeSceneDataset : SceneDatasetAttributes "
        << sceneDatasetName << " unable to be deleted. Aborting.";
    return false;
  }
  // Should always have a default dataset.
  if (sceneDatasetName.compare("default")) {
    // removing default dataset should still create another, empty, default
    // dataset.
    createSceneDataset("default");
  }
  LOG(INFO) << "MetadataMediator::removeSceneDataset : SceneDatasetAttributes "
            << sceneDatasetName << " successfully removed.";
  return true;

}  // MetadataMediator::removeSceneDataset

bool MetadataMediator::setCurrPhysicsAttributesHandle(
    const std::string& _physicsManagerAttributesPath) {
  // first check if physics manager attributes exists, if so then set as current
  if (physicsAttributesManager_->getObjectLibHasHandle(
          _physicsManagerAttributesPath)) {
    if (currPhysicsManagerAttributes_.compare(_physicsManagerAttributesPath) !=
        0) {
      LOG(INFO) << "MetadataMediator::setCurrPhysicsAttributesHandle : Old "
                   "physics manager attributes "
                << currPhysicsManagerAttributes_ << " changed to "
                << _physicsManagerAttributesPath << " successfully.";
      currPhysicsManagerAttributes_ = _physicsManagerAttributesPath;
    }
    sceneDatasetAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        currPhysicsManagerAttributes_);
    return true;
  }
  // if this handle does not exist, create the attributes for it.
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
  // first check if dataset exists/is loaded, if so then set as default
  if (sceneDatasetAttributesManager_->getObjectLibHasHandle(sceneDatasetName)) {
    if (activeSceneDataset_.compare(sceneDatasetName) != 0) {
      LOG(INFO)
          << "MetadataMediator::setActiveSceneDatasetName : Old active dataset "
          << activeSceneDataset_ << " changed to " << sceneDatasetName
          << " successfully.";
      activeSceneDataset_ = sceneDatasetName;
    }
    return true;
  }
  // if does not exist, attempt to create it
  LOG(INFO) << "MetadataMediator::setActiveSceneDatasetName : Attempting to "
               "create new dataset "
            << sceneDatasetName;
  bool success = createSceneDataset(sceneDatasetName);
  // if successfully created, set default name to access dataset attributes in
  // SceneDatasetAttributesManager
  if (success) {
    activeSceneDataset_ = sceneDatasetName;
  }
  LOG(INFO) << "MetadataMediator::setActiveSceneDatasetName : Attempt to "
               "create new dataset "
            << sceneDatasetName << " "
            << (success ? " succeeded." : " failed.");
  return success;
}  // MetadataMediator::setActiveSceneDatasetName

attributes::SceneAttributes::ptr MetadataMediator::getSceneAttributesByName(
    const std::string& sceneName) {
  // get current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
  // this should never happen
  if (datasetAttr == nullptr) {
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
  // get list of scene attributes handles that contain sceneName as a substring
  auto sceneList = dsSceneAttrMgr->getObjectHandlesBySubstring(sceneName);
  // sceneName can legally match any one of the following conditions :
  if (sceneList.size() > 0) {
    // 1.  Existing, registered SceneAttributes in current active dataset.
    //    In this case the SceneAttributes is returned.
    LOG(INFO) << "MetadataMediator::getSceneAttributesByName : Query dataset : "
              << activeSceneDataset_
              << " for SceneAttributes named : " << sceneName << " yields "
              << sceneList.size() << " candidates.  Using " << sceneList[0]
              << ".";
    sceneAttributes = dsSceneAttrMgr->getObjectCopyByHandle(sceneList[0]);
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
                << " but a SceneAttributes config named "
                << sceneFilenameCandidate
                << " was found on disk, so "
                   "loading.";
      sceneAttributes = dsSceneAttrMgr->createObjectFromJSONFile(
          sceneFilenameCandidate, true);
    } else {
      // get list of stage attributes handles that contain sceneName as a
      // substring
      auto stageList = dsStageAttrMgr->getObjectHandlesBySubstring(sceneName);
      if (stageList.size() > 0) {
        // 3.  Existing, registered StageAttributes in current active dataset.
        //    In this case, a SceneAttributes is created amd registered using
        //    sceneName, referencing the StageAttributes of the same name; This
        //    sceneAttributes is returned.
        LOG(INFO) << "MetadataMediator::getSceneAttributesByName : No existing "
                     "scene instance attributes containing name "
                  << sceneName << " found in Dataset : " << activeSceneDataset_
                  << " but " << stageList.size()
                  << " StageAttributes found.  Using " << stageList[0]
                  << " as stage and to construct a SceneAttributes with same "
                     "name that will be added to Dataset.";
        // create a new SceneAttributes, and give it a
        // SceneObjectInstanceAttributes for the stage with the same name.
        sceneAttributes = makeSceneAndReferenceStage(
            datasetAttr, dsStageAttrMgr->getObjectByHandle(stageList[0]),
            dsSceneAttrMgr, sceneName);

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
               "name, and then creating a SceneAttributes with the same name "
               "that references this stage.";
        // create and register stage
        auto stageAttributes = dsStageAttrMgr->createObject(sceneName, true);
        // create a new SceneAttributes, and give it a
        // SceneObjectInstanceAttributes for the stage with the same name.
        sceneAttributes = makeSceneAndReferenceStage(
            datasetAttr, stageAttributes, dsSceneAttrMgr, sceneName);
      }
    }
  }
  // make sure that all stage, object and lighting attributes referenced in
  // scene attributes are loaded in dataset, as well as the scene attributes
  // itself.
  datasetAttr->addNewSceneInstanceToDataset(sceneAttributes);

  return sceneAttributes;

}  // MetadataMediator::getSceneAttributesByName

attributes::SceneAttributes::ptr MetadataMediator::makeSceneAndReferenceStage(
    const attributes::SceneDatasetAttributes::ptr& datasetAttr,
    const attributes::StageAttributes::ptr& stageAttributes,
    const managers::SceneAttributesManager::ptr& dsSceneAttrMgr,
    const std::string& sceneName) {
  // create scene attributes with passed name
  attributes::SceneAttributes::ptr sceneAttributes =
      dsSceneAttrMgr->createDefaultObject(sceneName, false);
  // create stage instance attributes and set its name (from stage attributes)
  sceneAttributes->setStageInstance(
      dsSceneAttrMgr->createEmptyInstanceAttributes(
          stageAttributes->getHandle()));

  // The following is to manage stage files that have navmesh and semantic scene
  // descriptor ("house file") handles in them. This mechanism has been
  // deprecated, but in order to provide backwards compatibility, we are going
  // to support these values here when we synthesize a non-existing scene
  // instance attributes only.

  // add a ref to the navmesh path from the stage attributes to scene
  // attributes, giving it an appropriately obvious name.  This entails adding
  // the path itself to the dataset, if it does not already exist there, keyed
  // by the ref that the scene attributes will use.
  std::pair<std::string, std::string> navmeshEntry =
      datasetAttr->addNavmeshPathEntry(
          sceneName, stageAttributes->getNavmeshAssetHandle(), false);
  // navmeshEntry holds the navmesh key-value in the dataset to use by this
  // scene instance.  NOTE : the key may have changed from what was passed if a
  // collision occurred with same key but different value, so we need to add
  // this key to the scene instance attributes.
  sceneAttributes->setNavmeshHandle(navmeshEntry.first);

  // add a ref to semantic scene descriptor ("house file") from stage attributes
  // to scene attributes, giving it an appropriately obvious name.  This entails
  // adding the path itself to the dataset, if it does not already exist there,
  // keyed by the ref that the scene attributes will use.
  std::pair<std::string, std::string> ssdEntry =
      datasetAttr->addSemanticSceneDescrPathEntry(
          sceneName, stageAttributes->getHouseFilename(), false);
  // ssdEntry holds the ssd key in the dataset to use by this scene instance.
  // NOTE : the key may have changed from what was passed if a collision
  // occurred with same key but different value, so we need to add this key to
  // the scene instance attributes.
  sceneAttributes->setSemanticSceneHandle(ssdEntry.first);

  // register SceneAttributes object
  dsSceneAttrMgr->registerObject(sceneAttributes);
  return sceneAttributes;
}  // MetadataMediator::makeSceneAndReferenceStage

}  // namespace metadata
}  // namespace esp
