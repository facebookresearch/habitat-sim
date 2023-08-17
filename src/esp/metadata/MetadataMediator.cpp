// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MetadataMediator.h"

namespace esp {
namespace metadata {

MetadataMediator::MetadataMediator(const sim::SimulatorConfiguration& cfg) {
  buildAttributesManagers();
  // sets simConfig_, activeSceneDataset_ and currPhysicsManagerAttributes_
  // based on config
  setSimulatorConfiguration(cfg);
}  // MetadataMediator ctor (SimulatorConfiguration)

void MetadataMediator::buildAttributesManagers() {
  physicsAttributesManager_ = managers::PhysicsAttributesManager::create();

  pbrShaderAttributesManager_ = managers::PbrShaderAttributesManager::create();

  sceneDatasetAttributesManager_ =
      managers::SceneDatasetAttributesManager::create(
          physicsAttributesManager_, pbrShaderAttributesManager_);

  // should always have default dataset, but this is managed by MM instead of
  // made undeletable in SceneDatasetManager, so that it can be easily "reset"
  // by deleting and remaking.
  createSceneDataset("default");
  // should always have default physicsManagerAttributesPath
  bool success =
      createPhysicsManagerAttributes(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH);
  // Lock this so it is always available
  if (success) {
    physicsAttributesManager_->setLock(ESP_DEFAULT_PHYSICS_CONFIG_REL_PATH,
                                       true);
  }
  // should always have default PBRConfig
  success = createPbrAttributes(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH);
  // Lock this so it is always available
  pbrShaderAttributesManager_->setLock(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH,
                                       true);
  // set as default and current
  if (success) {
    setCurrDefaultPbrAttributesHandle(ESP_DEFAULT_PBRSHADER_CONFIG_REL_PATH);
  }
  // after this setSimulatorConfiguration will be called
}  // MetadataMediator::buildAttributesManagers

bool MetadataMediator::setSimulatorConfiguration(
    const sim::SimulatorConfiguration& cfg) {
  simConfig_ = cfg;

  // set current active dataset name - if unchanged, does nothing
  ESP_CHECK(setActiveSceneDatasetName(simConfig_.sceneDatasetConfigFile),
            // something failed about setting up active scene dataset
            "Some error prevented currently active Scene Dataset `"
                << Mn::Debug::nospace << activeSceneDataset_
                << Mn::Debug::nospace << "` being changed to `"
                << Mn::Debug::nospace << simConfig_.sceneDatasetConfigFile
                << Mn::Debug::nospace
                << "` as requested in the Simulator Configuration so aborting. "
                   "Verify requested Scene Dataset file name in Simulator "
                   "Configuration is correct.");
  // set active physics attributes handle - if unchanged, does nothing
  ESP_CHECK(setCurrPhysicsAttributesHandle(simConfig_.physicsConfigFile),
            // something failed about setting up physics attributes
            "Some error prevented changing current Physics Attributes to `"
                << Mn::Debug::nospace << simConfig_.physicsConfigFile
                << Mn::Debug::nospace
                << "` as requested in the Simulator Configuration so aborting. "
                   "Verify requested physics config filename in Simulator "
                   "Configuration is correct.");

  // get a ref to current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
  // pass relevant config values to the current dataset
  if (datasetAttr != nullptr) {
    datasetAttr->setCurrCfgVals(simConfig_.sceneLightSetupKey,
                                simConfig_.frustumCulling);
  } else {
    // No active dataset exists, so fail. Always should have an active dataset,
    // even if it is default.
    ESP_CHECK(false,
              "No active dataset exists or has been specified. Aborting");
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Set new simulator config for scene/stage : `"
      << simConfig_.activeSceneName << "` and dataset : `"
      << simConfig_.sceneDatasetConfigFile << "` which "
      << (activeSceneDataset_ == simConfig_.sceneDatasetConfigFile
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
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Unknown PhysicsManager Attributes configuration file : `"
          << _physicsManagerAttributesPath
          << "` does not exist and no PhysicsManager Attributes is able to be "
             "created for it, so creation failed.";
      return false;
    }
  }  // if dne then create
  return true;
}  // MetadataMediator::createPhysicsManagerAttributes

bool MetadataMediator::createPbrAttributes(const std::string& _pbrConfigPath) {
  bool exists =
      pbrShaderAttributesManager_->getObjectLibHasHandle(_pbrConfigPath);
  if (!exists) {
    auto pbrConfigs =
        pbrShaderAttributesManager_->createObject(_pbrConfigPath, true);
    if (pbrConfigs == nullptr) {
      // something failed during creation process.
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Unknown PBR/IBL shader configuration file : `" << _pbrConfigPath
          << "` does not exist and no PBRShaderConfiguration is able to be "
             "created for it, so creation failed.";
      return false;
    }
  }  // if dne then create
  return true;
}  // MetadataMediator::createPbrAttributes

bool MetadataMediator::createSceneDataset(const std::string& sceneDatasetName,
                                          bool overwrite) {
  // see if exists
  bool exists = sceneDatasetExists(sceneDatasetName);
  if (exists) {
    // check if not overwrite and exists already
    if (!overwrite) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Scene Dataset `" << sceneDatasetName
          << "` already exists. To reload and overwrite existing "
             "data, set overwrite to true. Using existing Scene Dataset.";
      // Treat as if created, since it exists.
      return true;
    }
    // overwrite specified, make sure not locked
    sceneDatasetAttributesManager_->setLock(sceneDatasetName, false);
  }
  // by here dataset either does not exist or exists but is unlocked and
  // overwrite is specified. attempt to create new/overwrite
  auto datasetAttribs =
      sceneDatasetAttributesManager_->createObject(sceneDatasetName, true);
  // Failure here means some catastrophic error attempting to create the dataset
  // attributes. Should fail code
  ESP_CHECK(datasetAttribs,
            "Scene Dataset `"
                << Mn::Debug::nospace << sceneDatasetName << Mn::Debug::nospace
                << "` does not exist and is unable to be "
                   "created using that name. Verify the given file name.");

  // if not null then successfully created
  // lock dataset to prevent accidental deletion
  // should only be removable via MetadataMediator::removeSceneDataset.
  sceneDatasetAttributesManager_->setLock(sceneDatasetName, true);
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "Dataset `" << sceneDatasetName
      << "` successfully created and locked.";

  return true;
}  // MetadataMediator::createSceneDataset

bool MetadataMediator::removeSceneDataset(const std::string& sceneDatasetName) {
  // First check if SceneDatasetAttributes exists
  if (!sceneDatasetExists(sceneDatasetName)) {
    // DNE, do nothing
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "SceneDatasetAttributes `" << sceneDatasetName
        << "` does not exist, so it cannot be removed.";

    return true;
  }

  // Next check if is current activeSceneDataset_, and if so skip with warning
  if (sceneDatasetName == activeSceneDataset_) {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "Cannot remove currently active SceneDatasetAttributes `"
        << sceneDatasetName << "`. Switch to another dataset before removing.";
    return false;
  }

  // Now force unlock and remove requested SceneDatasetAttributes- there should
  // be no SceneDatasetAttributes set to undeletable
  sceneDatasetAttributesManager_->setLock(sceneDatasetName, false);
  auto delDataset =
      sceneDatasetAttributesManager_->removeObjectByHandle(sceneDatasetName);
  // if failed here, probably means SceneDatasetAttributes was set to
  // undeletable, give warning and return false
  if (delDataset == nullptr) {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "SceneDatasetAttributes `" << sceneDatasetName
        << "` unable to be deleted, probably due to it being marked "
           "undeletable in manager, so aborting dataset delete.";
    return false;
  }
  // Should always have a default dataset. Use this process to remove extraneous
  // configs in default Scene Dataset
  if (sceneDatasetName == "default") {
    // removing default dataset should still create another, empty, default
    // dataset.
    createSceneDataset("default", true);
  }
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "SceneDatasetAttributes `" << sceneDatasetName
      << "` successfully removed.";
  return true;

}  // MetadataMediator::removeSceneDataset

bool MetadataMediator::setCurrPhysicsAttributesHandle(
    const std::string& _physicsManagerAttributesPath) {
  // first check if physics manager attributes exists, if so then set as current
  if (physicsAttributesManager_->getObjectLibHasHandle(
          _physicsManagerAttributesPath)) {
    if (currPhysicsManagerAttributes_ != _physicsManagerAttributesPath) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Old physics manager attributes `" << currPhysicsManagerAttributes_
          << "` changed to `" << _physicsManagerAttributesPath
          << "` successfully.";
      currPhysicsManagerAttributes_ = _physicsManagerAttributesPath;
    }
    sceneDatasetAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        currPhysicsManagerAttributes_);
    return true;
  }
  // if this handle does not exist, create the attributes for it.
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Attempting to create new physics manager attributes `"
      << _physicsManagerAttributesPath << "`.";
  bool success = createPhysicsManagerAttributes(_physicsManagerAttributesPath);
  // if successfully created, set default name to physics manager attributes in
  // SceneDatasetAttributesManager
  if (success) {
    currPhysicsManagerAttributes_ = _physicsManagerAttributesPath;
    sceneDatasetAttributesManager_->setCurrPhysicsManagerAttributesHandle(
        currPhysicsManagerAttributes_);
    /// setCurrPhysicsManagerAttributesHandle
  }
  ESP_DEBUG() << "Attempt to create new  Physics Manager Attributes from `"
              << _physicsManagerAttributesPath << "`"
              << (success ? " succeeded." : " failed.")
              << "Currently active Physics Manager Attributes :"
              << currPhysicsManagerAttributes_;
  return success;

}  // MetadataMediator::setCurrPhysicsAttributesHandle

bool MetadataMediator::setCurrDefaultPbrAttributesHandle(
    const std::string& pbrShaderAttributesPath) {
  // first check if  Pbr/Ibl shader attributes exists, if so then set as current
  if (pbrShaderAttributesManager_->getObjectLibHasHandle(
          pbrShaderAttributesPath)) {
    if (currDefaultPbrAttributesHandle_ != pbrShaderAttributesPath) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Old default Pbr/Ibl shader attributes `"
          << currDefaultPbrAttributesHandle_ << "` changed to `"
          << pbrShaderAttributesPath << "` successfully.";
      currDefaultPbrAttributesHandle_ = pbrShaderAttributesPath;
    }
    sceneDatasetAttributesManager_->setDefaultPbrShaderAttributesHandle(
        currDefaultPbrAttributesHandle_);
    return true;
  }
  // if this handle does not exist, create the attributes for it.
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Attempting to create new default Pbr/Ibl shader attributes `"
      << pbrShaderAttributesPath << "`.";
  bool success = createPbrAttributes(pbrShaderAttributesPath);
  // if successfully created, set default name to  Pbr/Ibl shader attributes in
  // SceneDatasetAttributesManager
  if (success) {
    currDefaultPbrAttributesHandle_ = pbrShaderAttributesPath;
    sceneDatasetAttributesManager_->setDefaultPbrShaderAttributesHandle(
        currDefaultPbrAttributesHandle_);
    /// setDefaultPbrShaderAttributesHandle
  }
  ESP_DEBUG()
      << "Attempt to create new default Pbr/Ibl shader attributes from `"
      << pbrShaderAttributesPath << "`"
      << (success ? " succeeded." : " failed.")
      << "Currently active default Pbr/Ibl shader attributes :"
      << currDefaultPbrAttributesHandle_;
  return success;

}  // MetadataMediator::setCurrDefaultPbrAttributesHandle

bool MetadataMediator::setActiveSceneDatasetName(
    const std::string& sceneDatasetName) {
  // first check if dataset exists/is loaded, if so then set as default
  if (sceneDatasetExists(sceneDatasetName)) {
    if (activeSceneDataset_ != sceneDatasetName) {
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Previous active Scene Dataset `" << activeSceneDataset_
          << "` changed to `" << sceneDatasetName << "` successfully.";
      activeSceneDataset_ = sceneDatasetName;
    }
    return true;
  }
  // if does not exist, attempt to load/create it
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Attempting to create new dataset `" << sceneDatasetName << "`.";
  bool success = createSceneDataset(sceneDatasetName);
  // if successfully created, set default name to access dataset attributes in
  // SceneDatasetAttributesManager
  if (success) {
    activeSceneDataset_ = sceneDatasetName;
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Attempt to load/create new Scene Dataset from `" << sceneDatasetName
      << "`" << (success ? " succeeded." : " failed. ")
      << "Currently active Scene Dataset : `" << activeSceneDataset_ << "`.";
  return success;
}  // MetadataMediator::setActiveSceneDatasetName

attributes::SceneInstanceAttributes::ptr
MetadataMediator::getSceneInstanceAttributesByName(
    const std::string& sceneName) {
  // get current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();

  // this should never happen, and would indicate someone did an end-run around
  // management code and inappropriately deleted the dataset without resetting
  // the activeSceneDataset_ tag.
  ESP_CHECK(datasetAttr, "No active Scene Dataset specified/exists.");

  // directory to look for attributes for this dataset
  const std::string dsDir = datasetAttr->getFileDirectory();
  // get appropriate attr managers for the current dataset
  managers::SceneInstanceAttributesManager::ptr dsSceneAttrMgr =
      datasetAttr->getSceneInstanceAttributesManager();
  managers::StageAttributesManager::ptr dsStageAttrMgr =
      datasetAttr->getStageAttributesManager();

  attributes::SceneInstanceAttributes::ptr sceneInstanceAttributes = nullptr;
  // get list of scene attributes handles that contain sceneName as a substring
  auto sceneList = dsSceneAttrMgr->getObjectHandlesBySubstring(sceneName);
  // sceneName can legally match any one of the following conditions :
  if (!sceneList.empty()) {
    // 1. Existing, registered SceneInstanceAttributes in current active
    // dataset.
    //    In this case the SceneInstanceAttributes is returned.
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Querying Scene Dataset : `" << activeSceneDataset_
        << "` for Scene Instance Attributes named : `" << sceneName
        << "` yielded " << sceneList.size() << " candidates. Using `"
        << sceneList[0] << "` (first candidate).";
    sceneInstanceAttributes =
        dsSceneAttrMgr->getObjectCopyByHandle(sceneList[0]);
  } else {
    const std::string sceneFilenameCandidate =
        dsSceneAttrMgr->getFormattedJSONFileName(sceneName);

    if (Cr::Utility::Path::exists(sceneFilenameCandidate)) {
      // 2. Existing, valid SceneInstanceAttributes file on disk, but not in
      // dataset.
      //    If this is the case, then the SceneInstanceAttributes should be
      //    loaded, registered, added to the dataset and returned.
      ESP_DEBUG(Mn::Debug::Flag::NoSpace)
          << "Scene Dataset : `" << activeSceneDataset_
          << "` does not reference a Scene Instance Attributes named `"
          << sceneName << "` but a Scene Instance Attributes config named `"
          << sceneFilenameCandidate
          << "` was found on disk, so this will be loaded and added to the "
             "currently active Scene Dataset.";
      sceneInstanceAttributes = dsSceneAttrMgr->createObjectFromJSONFile(
          sceneFilenameCandidate, true);
    } else {
      // get list of stage attributes handles that contain sceneName as a
      // substring
      auto stageList = dsStageAttrMgr->getObjectHandlesBySubstring(sceneName);
      if (!stageList.empty()) {
        // 3. Existing, registered StageAttributes in current active dataset.
        //    In this case, a SceneInstanceAttributes is created amd registered
        //    using sceneName, referencing the StageAttributes of the same name;
        //    This sceneInstanceAttributes is returned.
        ESP_DEBUG(Mn::Debug::Flag::NoSpace)
            << "Scene Dataset : `" << activeSceneDataset_
            << "` does not reference a Scene Instance Attributes named `"
            << sceneName << "` but `" << stageList.size()
            << " Stage Attributes were found with a similar name. Using `"
            << stageList[0]
            << "` (first entry) as stage and will construct a Scene Instance "
               "Attributes with the same name that will be added to the "
               "currently active Scene Dataset.";
        // create a new SceneInstanceAttributes, and give it a
        // SceneObjectInstanceAttributes for the stage with the same name.
        sceneInstanceAttributes = makeSceneAndReferenceStage(
            datasetAttr, dsStageAttrMgr->getObjectByHandle(stageList[0]),
            dsSceneAttrMgr, sceneName);

      } else {
        // 4. Either existing stage config/asset on disk, but not in current
        // dataset, or no stage config/asset exists with passed name.
        //    In this case, a stage attributes is loaded/created and registered,
        //    then added to current dataset, and then 3. is performed.
        ESP_DEBUG(Mn::Debug::Flag::NoSpace)
            << "Scene Dataset : `" << activeSceneDataset_
            << "` does not reference a Scene Instance Attributes or Stage "
               "Attributes named `"
            << sceneName
            << "` so loading/creating a new Stage Attributes with this "
               "name, and then creating a Scene Instance Attributes with "
               "the same name that references this stage, which will be added "
               "to the currently active Scene Dataset.";
        // create and register stage attributes
        auto stageAttributes = dsStageAttrMgr->createObject(sceneName, true);
        // create a new SceneInstanceAttributes, and give it a
        // SceneObjectInstanceAttributes for the stage with the same name.
        sceneInstanceAttributes = makeSceneAndReferenceStage(
            datasetAttr, stageAttributes, dsSceneAttrMgr, sceneName);
      }
    }
  }
  // make sure that all stage, object and lighting attributes referenced in
  // scene attributes are loaded in dataset, as well as the scene attributes
  // itself.
  datasetAttr->addNewSceneInstanceToDataset(sceneInstanceAttributes);

  return sceneInstanceAttributes;

}  // MetadataMediator::getSceneInstanceAttributesByName

attributes::SceneInstanceAttributes::ptr
MetadataMediator::makeSceneAndReferenceStage(
    const attributes::SceneDatasetAttributes::ptr& datasetAttr,
    const attributes::StageAttributes::ptr& stageAttributes,
    const managers::SceneInstanceAttributesManager::ptr& dsSceneAttrMgr,
    const std::string& sceneName) {
  // Verify that a Scene Dataset (possibly default) exists.
  ESP_CHECK(
      datasetAttr,
      "No Scene Dataset Attributes exists for currently active Scene Dataset `"
          << Mn::Debug::nospace << activeSceneDataset_ << Mn::Debug::nospace
          << "` from which to load scene '" << Mn::Debug::nospace << sceneName
          << Mn::Debug::nospace
          << "'. Likely the Scene Dataset Configuration file requested was not "
             "found and so an attempt to create a new, empty Scene Dataset was "
             "made and failed somehow. Verify the Scene Dataset Configuration "
             "file name used.");

  // Verify that the SceneInstanceAttributesManager exists. This should never
  // fire - there should always be a SceneInstanceAttributesManager
  ESP_CHECK(
      dsSceneAttrMgr,
      "No Scene Instance Attributes Manager was created for Scene Dataset `"
          << Mn::Debug::nospace << activeSceneDataset_ << Mn::Debug::nospace
          << "` to load scene '" << Mn::Debug::nospace << sceneName
          << Mn::Debug::nospace
          << "'. This would only occur due to an internal Habitat-Sim error.");

  // Verify that a StageAttributes exists in SceneInstance for requested Scene
  ESP_CHECK(
      stageAttributes,
      "No Stage Attributes exists for requested scene '"
          << Mn::Debug::nospace << sceneName << Mn::Debug::nospace
          << "' in currently specified Scene Dataset `" << Mn::Debug::nospace
          << activeSceneDataset_ << Mn::Debug::nospace
          << "`. Likely cause is either the Scene Dataset Configuration "
             "requested was not found, or the paths to the various dataset "
             "component configuration files (i.e. "
             "stage, object, scene instance, etc.) specified in the Scene "
             "Dataset Configuration file cannot be found. Verify the paths and "
             "filenames for the desired Scene Dataset Configuration file and "
             "the paths specified within the file.");

  // create empty SceneInstanceAttributes with passed name
  attributes::SceneInstanceAttributes::ptr sceneInstanceAttributes =
      dsSceneAttrMgr->createDefaultObject(sceneName, false);
  // Verify that the requested SceneInstanceAttributes is created successfully.
  // This should never fail- we're either building a SceneInstanceAttributes as
  // a copy of a previously verified Scene Dataset default or we're creating a
  // new SceneInstanceAttributes via its constructor.
  ESP_CHECK(sceneInstanceAttributes,
            "In currently specified Scene Dataset `"
                << Mn::Debug::nospace << activeSceneDataset_
                << Mn::Debug::nospace
                << "`, Scene Instance Attributes creation failed for scene '"
                << Mn::Debug::nospace << sceneName << Mn::Debug::nospace << ""
                << "'. This would occur due to an internal Habitat-Sim error.");

  // create stage instance attributes and set its name (from stage attributes)
  sceneInstanceAttributes->setStageInstanceAttrs(
      dsSceneAttrMgr->createEmptyInstanceAttributes(
          stageAttributes->getHandle()));

  // The following is to manage stage files that have navmesh and semantic scene
  // descriptor ("house file") handles in them. This mechanism has been
  // deprecated, but in order to provide backwards compatibility, we are going
  // to support these values here when we synthesize a non-existing scene
  // instance attributes only.

  // add a ref to the navmesh path from the stage attributes to scene
  // attributes, giving it an appropriately obvious name. This entails adding
  // the path itself to the dataset, if it does not already exist there, keyed
  // by the ref that the scene attributes will use.
  std::pair<std::string, std::string> navmeshEntry =
      datasetAttr->addNavmeshPathEntry(
          sceneName, stageAttributes->getNavmeshAssetHandle(), false);
  // navmeshEntry holds the navmesh key-value in the dataset to use by this
  // scene instance. NOTE : the key may have changed from what was passed if a
  // collision occurred with same key but different value, so we need to add
  // this key to the Scene Instance Attributes.
  sceneInstanceAttributes->setNavmeshHandle(navmeshEntry.first);

  // add a ref to semantic scene descriptor ("house file") from stage attributes
  // to scene attributes, giving it an appropriately obvious name. This entails
  // adding the path itself to the dataset, if it does not already exist there,
  // keyed by the ref that the scene attributes will use.
  std::pair<std::string, std::string> ssdEntry =
      datasetAttr->addSemanticSceneDescrPathEntry(
          sceneName, stageAttributes->getSemanticDescriptorFilename(), false);
  // ssdEntry holds the ssd key in the dataset to use by this scene instance.
  // NOTE : the key may have changed from what was passed if a collision
  // occurred with same key but different value, so we need to add this key to
  // the Scene Instance Attributes.
  sceneInstanceAttributes->setSemanticSceneHandle(ssdEntry.first);

  // register SceneInstanceAttributes object
  dsSceneAttrMgr->registerObject(sceneInstanceAttributes);
  return sceneInstanceAttributes;
}  // MetadataMediator::makeSceneAndReferenceStage

std::shared_ptr<esp::core::config::Configuration>
MetadataMediator::getSceneInstanceUserConfiguration(
    const std::string& curSceneName) {
  // get current dataset attributes
  attributes::SceneDatasetAttributes::ptr datasetAttr = getActiveDSAttribs();
  // this should never happen
  if (datasetAttr == nullptr) {
    ESP_ERROR() << "No active dataset specified/exists, so aborting User "
                   "Configuration retrieval.";
    return nullptr;
  }
  // get scene instance attribute manager
  managers::SceneInstanceAttributesManager::ptr dsSceneAttrMgr =
      datasetAttr->getSceneInstanceAttributesManager();
  // get list of scene attributes handles that contain sceneName as a substring
  auto sceneList = dsSceneAttrMgr->getObjectHandlesBySubstring(curSceneName);
  // returned list of scene names must not be empty, otherwise display error
  // message and return nullptr
  if (!sceneList.empty()) {
    // Scene instance exists with given name, registered SceneInstanceAttributes
    // in current active dataset.
    //    In this case the SceneInstanceAttributes is returned.
    ESP_DEBUG() << "Query dataset :" << activeSceneDataset_
                << "for SceneInstanceAttributes named :" << curSceneName
                << "yields" << sceneList.size() << "candidates. Using"
                << sceneList[0] << Mn::Debug::nospace << ".";
    return dsSceneAttrMgr->getObjectCopyByHandle(sceneList[0])
        ->getUserConfiguration();
  }
  ESP_ERROR() << "No scene instance specified/exists with name" << curSceneName
              << ", so Aborting.";
  return nullptr;

}  // MetadataMediator::getSceneInstanceUserConfiguration

std::string MetadataMediator::getFilePathForHandle(
    const std::string& assetHandle,
    const std::map<std::string, std::string>& assetMapping,
    const std::string& msgString) {
  std::map<std::string, std::string>::const_iterator mapIter =
      assetMapping.find(assetHandle);
  if (mapIter == assetMapping.end()) {
    ESP_WARNING() << msgString << ": Unable to find file path for"
                  << assetHandle << ", so returning empty string.";
    return "";
  }
  return mapIter->second;
}  // MetadataMediator::getFilePathForHandle

std::string MetadataMediator::getDatasetsOverview() const {
  // reserve space for info strings for all scene datasets
  std::vector<std::string> sceneDatasetHandles =
      sceneDatasetAttributesManager_->getObjectHandlesBySubstring("");
  std::string res =
      "Datasets : \n" +
      attributes::SceneDatasetAttributes::getDatasetSummaryHeader() + "\n";
  for (const std::string& handle : sceneDatasetHandles) {
    res += sceneDatasetAttributesManager_->getObjectByHandle(handle)
               ->getDatasetSummary();
    res += '\n';
  }
  // Get summary of Pbr/Ibl shader configurations available
  std::vector<std::string> pbrShaderConfigHandles =
      pbrShaderAttributesManager_->getObjectHandlesBySubstring("");
  int numPbrConfigs = pbrShaderConfigHandles.size();
  if (numPbrConfigs > 0) {
    Cr::Utility::formatInto(
        res, res.length(),
        "\nPbr/IBL Shader configurations available to all datasets :\n{}\n",
        numPbrConfigs);
  }
  return res;
}  // MetadataMediator::getDatasetNames

std::string MetadataMediator::createDatasetReport(
    const std::string& sceneDataset) const {
  attributes::SceneDatasetAttributes::ptr ds;
  if (sceneDataset == "") {
    ds = sceneDatasetAttributesManager_->getObjectByHandle(activeSceneDataset_);

  } else if (sceneDatasetAttributesManager_->getObjectLibHasHandle(
                 sceneDataset)) {
    ds = sceneDatasetAttributesManager_->getObjectByHandle(sceneDataset);
  } else {
    // unknown dataset
    ESP_ERROR() << "Dataset" << sceneDataset
                << "is not found in the MetadataMediator, so unable to create "
                   "Dataset Report.";
    return "Requested SceneDataset `" + sceneDataset + "` unknown.";
  }
  return Corrade::Utility::formatString(
      "Scene Dataset {}\n{}\n", ds->getObjectInfoHeader(), ds->getObjectInfo());
}  // MetadataMediator::const std::string MetadataMediator::createDatasetReport(

}  // namespace metadata
}  // namespace esp
