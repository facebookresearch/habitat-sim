// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneDatasetAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {
using esp::core::managedContainers::ManagedObjectAccess;

SceneDatasetAttributes::SceneDatasetAttributes(
    const std::string& datasetName,
    const managers::PhysicsAttributesManager::ptr& physAttrMgr)
    : AbstractAttributes("SceneDatasetAttributes", datasetName) {
  assetAttributesManager_ = managers::AssetAttributesManager::create();
  lightLayoutAttributesManager_ =
      managers::LightLayoutAttributesManager::create();
  artObjAttributesManager_ = managers::AOAttributesManager::create();
  objectAttributesManager_ = managers::ObjectAttributesManager::create();
  objectAttributesManager_->setAssetAttributesManager(assetAttributesManager_);
  sceneInstanceAttributesManager_ =
      managers::SceneInstanceAttributesManager::create();
  semanticAttributesManager_ = managers::SemanticAttributesManager::create();
  stageAttributesManager_ =
      managers::StageAttributesManager::create(physAttrMgr);
  stageAttributesManager_->setAssetAttributesManager(assetAttributesManager_);
  // Use PBR as default materials.Override this in SceneDataset config
  setDefaultMaterialIsPBR(true);
}  // ctor

bool SceneDatasetAttributes::addNewSceneInstanceToDataset(
    const attributes::SceneInstanceAttributes::ptr& sceneInstance) {
  // info display message prefix
  const std::string infoPrefix = Cr::Utility::formatString(
      "Dataset : '{}' : ", this->getSimplifiedHandle());

  const std::string sceneInstanceName = sceneInstance->getHandle();
  // verify stage in sceneInstance (required) exists in SceneDatasetAttributes,
  // and if not, add it.
  const auto& stageInstance = sceneInstance->getStageInstance();
  const std::string stageHandle = stageInstance->getHandle();
  // Check if present and if not create default.
  if (stageAttributesManager_->getFullAttrNameFromStr(stageHandle).empty()) {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << infoPrefix << "Stage Attributes '" << stageHandle
        << "' specified in Scene Attributes but does not exist in dataset, "
           "so creating default.";
    stageAttributesManager_->createObject(stageHandle, true);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << infoPrefix << "Stage Attributes '" << stageHandle
        << "' specified in Scene Attributes exists in dataset library.";
  }

  // verify each object in sceneInstance exists in SceneDatasetAttributes
  auto objectInstances = sceneInstance->getObjectInstances();
  for (const auto& objInstance : objectInstances) {
    const std::string objHandle = objInstance->getHandle();
    if (objectAttributesManager_->getFullAttrNameFromStr(objHandle).empty()) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << infoPrefix << "Object Attributes '" << objHandle
          << "' specified in Scene Attributes but does not exist in dataset, "
             "so creating default.";

      objectAttributesManager_->createObject(objHandle, true);
    } else {
      ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
          << infoPrefix << "Object Attributes '" << objHandle
          << "' specified in Scene Attributes exists in dataset library.";
    }
  }
  // verify each articulated object in sceneInstance exists in
  // SceneDatasetAttributes
  auto artObjectInstances = sceneInstance->getArticulatedObjectInstances();
  for (const auto& artObjInstance : artObjectInstances) {
    const std::string artObjHandle = artObjInstance->getHandle();

    // Check if present and if not create default.
    if (artObjAttributesManager_->getFullAttrNameFromStr(artObjHandle)
            .empty()) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << infoPrefix << "Articulated Object Attributes '" << artObjHandle
          << "' specified in Scene Attributes but does not exist in dataset, "
             "so creating default.";
      artObjAttributesManager_->createObject(artObjHandle, true);
    } else {
      ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
          << infoPrefix << "Articulated Object Attributes '" << artObjHandle
          << "' specified in Scene Attributes exists in dataset library.";
    }
  }

  // verify lighting
  std::string lightHandle = sceneInstance->getLightingHandle();
  // if lighthandle is empty, give handle same name as scene instance
  // should only be empty if not specified in scene instance config, or if scene
  // instance config is synthesized (i.e. when Simulator::reconfigure is called
  // with SimulatorConfiguration::activeSceneName being a stage)
  if (lightHandle.empty()) {
    lightHandle = sceneInstanceName;
  }
  // Check if lighting attributes specified is present and if not create
  // default.
  if (lightLayoutAttributesManager_->getFullAttrNameFromStr(lightHandle) ==
      "") {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << infoPrefix << "Lighting Layout Attributes '" << lightHandle
        << "' specified in Scene Attributes but does not exist in dataset, "
           "so creating default.";
    lightLayoutAttributesManager_->createObject(lightHandle, true);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << infoPrefix << "Lighting Layout Attributes '" << lightHandle
        << "' specified in Scene Attributes exists in dataset library.";
  }

  const std::string fullSceneInstanceName =
      sceneInstanceAttributesManager_->getFullAttrNameFromStr(
          sceneInstanceName);
  // add scene attributes to scene attributes manager
  if (fullSceneInstanceName.empty()) {
    ESP_VERY_VERBOSE() << infoPrefix << "Scene Attributes" << sceneInstanceName
                       << "does not exist in dataset so adding.";
    sceneInstanceAttributesManager_->registerObject(sceneInstance);
  }

  return true;
}  // SceneDatasetAttributes::addSceneInstanceToDataset

void SceneDatasetAttributes::createSemanticAttribsFromDS(
    const std::string& semanticHandle,
    const std::string& dbgSourceAttribs) {
  const std::string infoPrefix =
      Cr::Utility::formatString("Dataset : `{}` : Semantic Attributes `{}`",
                                this->getSimplifiedHandle(), semanticHandle);
  // Check if exists or build a new SemanticAttributes with the passed Stage's
  // or Scene Dataset Config's semantic data set/file
  // Should match -exactly-
  if (!semanticAttributesManager_->getObjectLibHasHandle(semanticHandle)) {
    // DNE, create a new one
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << infoPrefix
        << "does not exists; A new one will be created and saved.";
    semanticAttributesManager_->createObject(semanticHandle, true);
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << infoPrefix << " specified in " << dbgSourceAttribs
        << " already exists in dataset library. ";
  }
}  // SceneDatasetAttributes::createSemanticAttribsFromDS

std::string SceneDatasetAttributes::addSemanticSceneDescrPathEntry(
    const std::string& semanticHandle,
    const attributes::StageAttributes::ptr& stageAttributes) {
  const auto stageSSDFilename =
      stageAttributes->getSemanticDescriptorFilename();
  const auto stageSemanticAssetFilename =
      stageAttributes->getSemanticAssetHandle();
  bool setSemanticAssetData = (stageSemanticAssetFilename != "");
  bool setSSDFilename = (stageSSDFilename != "");
  // create a semantic attributes if DNE with given handle
  this->createSemanticAttribsFromDS(semanticHandle, "Stage Attributes");

  // Get actual object and set semantic data if appropriate
  auto semanticAttr =
      semanticAttributesManager_->getObjectByHandle(semanticHandle);

  if (setSemanticAssetData && semanticAttr->getSemanticAssetHandle().empty()) {
    // asset handle specified, get all stage-specified data
    semanticAttr->setSemanticAssetHandle(stageSemanticAssetFilename);
    semanticAttr->setSemanticAssetFullPath(
        stageAttributes->getSemanticAssetFullPath());
    semanticAttr->setSemanticAssetTypeEnum(
        stageAttributes->getSemanticAssetType());
    semanticAttr->setSemanticOrientUp(stageAttributes->getSemanticOrientUp());
    semanticAttr->setSemanticOrientFront(
        stageAttributes->getSemanticOrientFront());
  }
  if (setSSDFilename && semanticAttr->getSemanticDescriptorFilename().empty()) {
    // scene descriptor filename specified in stage, set in semantic
    // attributes.
    semanticAttr->setSemanticDescriptorFilename(stageSSDFilename);
    semanticAttr->setSemanticDescriptorFullPath(
        stageAttributes->getSemanticDescriptorFullPath());
  }
  // Save changes and make sure appropriate filtering happens
  semanticAttributesManager_->registerObject(semanticAttr, semanticHandle);
  return semanticAttr->getHandle();
}  // SceneDatasetAttributes::addSemanticSceneDescrPathEntry

void SceneDatasetAttributes::setSemanticAttrSSDFilenames(
    const std::map<std::string, std::string>& semanticPathnameMap) {
  for (const auto& entry : semanticPathnameMap) {
    const std::string semanticHandle = entry.first;
    const std::string ssdFilename = entry.second;

    // create a semantic attributes if DNE with given handle
    this->createSemanticAttribsFromDS(semanticHandle,
                                      "Dataset Config Map Entry");
    // Get actual object and set semantic data if appropriate
    auto semanticAttr =
        semanticAttributesManager_->getObjectByHandle(semanticHandle);
    semanticAttr->setSemanticDescriptorFilename(ssdFilename);
    // Save changes and make sure appropriate filtering happens
    semanticAttributesManager_->registerObject(semanticAttr, semanticHandle);
  }

}  // SceneDatasetAttributes::setSemanticAttrSSDFilenames

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
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << descString << " : Provided key '" << key
            << "' already references a different value in map. Modifying key "
               "to be '"
            << newKey
            << "'. Set overwrite to true to overwrite existing entries.";
      } else {  // overwrite entry
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << descString << " : Overwriting existing map entry "
            << map.at(newKey) << " at key '" << newKey << "' with value'"
            << path << "'.";
      }  // overwrite or not
    }    // found entry is desired or not
  }      // key is found
  map[newKey] = path;
  return *(map.find(newKey));
}  // SceneDatasetAttributes::addNewValToMap

esp::gfx::LightSetup SceneDatasetAttributes::getNamedLightSetup(
    const std::string& lightSetupName) {
  auto lightLayoutAttrName = getLightSetupFullHandle(lightSetupName);
  // lightLayoutAttrName == NO_LIGHT_KEY and DEFAULT_LIGHTING_KEY
  // handled in lightLayoutAttributesManager_
  return lightLayoutAttributesManager_->createLightSetupFromAttributes(
      lightLayoutAttrName);

}  // SceneDatasetAttributes::getNamedLightSetup

attributes::StageAttributes::ptr
SceneDatasetAttributes::getNamedStageAttributesCopy(
    const std::string& stageAttrName) {
  // do a substring search to find actual stage attributes and return first
  // attributes found; if does not exist, name will be empty. return nullptr
  auto fullStageName =
      stageAttributesManager_->getFullAttrNameFromStr(stageAttrName);
  // fullStageName will be empty if not found
  if (fullStageName.empty()) {
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
      objectAttributesManager_->getFullAttrNameFromStr(objAttrName);
  // fullObjName will be empty if not found
  if (fullObjName.empty()) {
    return nullptr;
  }
  return objectAttributesManager_->getObjectCopyByHandle(fullObjName);
}  // SceneDatasetAttributes::getNamedObjectAttributesCopy

attributes::ArticulatedObjectAttributes::ptr
SceneDatasetAttributes::getNamedArticulatedObjectAttributesCopy(
    const std::string& artObjAttrName) {
  // do a substring search to find actual object attributes and return first
  // attributes found; if does not exist, name will be empty. return nullptr
  auto fullArtObjName =
      artObjAttributesManager_->getFullAttrNameFromStr(artObjAttrName);
  // fullObjName will be empty if not found
  if (fullArtObjName.empty()) {
    return nullptr;
  }
  return artObjAttributesManager_->getObjectCopyByHandle(fullArtObjName);
}  // SceneDatasetAttributes::getNamedArticulatedObjectAttributesCopy

namespace {

std::string concatStrings(const std::string& header,
                          const std::vector<std::string>& vec) {
  // subtract 1 for header row
  std::string res = Cr::Utility::formatString("\n{} : {} available.\n", header,
                                              vec.size() - 1);
  for (const std::string& s : vec) {
    Cr::Utility::formatInto(res, res.size(), "{}\n", s);
  }
  return res;
}

std::string concatStrings(const std::string& header,
                          const std::map<std::string, std::string>& map) {
  std::string res =
      Cr::Utility::formatString("\n{} : {} available.\n", header, map.size());
  for (const auto& item : map) {
    Cr::Utility::formatInto(res, res.size(), "{}, {}\n", item.first,
                            item.second);
  }
  return res;
}

}  // namespace

std::string SceneDatasetAttributes::getObjectInfoInternal() const {
  // provide a summary for all info for this scene dataset
  // scene instances
  std::string res =
      concatStrings("Scene Instances",
                    sceneInstanceAttributesManager_->getObjectInfoStrings());

  // stages
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Stage Templates",
                    stageAttributesManager_->getObjectInfoStrings()));

  // objects
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Object Templates",
                    objectAttributesManager_->getObjectInfoStrings()));

  // articulated objects
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Articulated Object Templates",
                    artObjAttributesManager_->getObjectInfoStrings()));

  // lights
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Lighting Configurations",
                    lightLayoutAttributesManager_->getObjectInfoStrings()));

  // prims
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Primitives Templates",
                    assetAttributesManager_->getObjectInfoStrings()));

  // navmesh
  Cr::Utility::formatInto(res, res.size(), "{}",
                          concatStrings("Navmeshes", navmeshMap_));
  // SSD entries
  Cr::Utility::formatInto(
      res, res.size(), "{}",
      concatStrings("Semantic Scene Descriptor Templates",
                    semanticAttributesManager_->getObjectInfoStrings()));

  return res;
}

std::string SceneDatasetAttributes::getDatasetSummaryHeader() {
  return "Dataset Name,Scene Instance Templates,Stage Templates,Object "
         "Templates,Articulated Object Paths,Lighting Templates,Primitive "
         "Templates,Navmesh Entries,Semantic Scene Descriptor Templates,";
}

std::string SceneDatasetAttributes::getDatasetSummary() const {
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{},{},", getSimplifiedHandle(),
      sceneInstanceAttributesManager_->getNumObjects(),
      stageAttributesManager_->getNumObjects(),
      objectAttributesManager_->getNumObjects(),
      artObjAttributesManager_->getNumObjects(),
      lightLayoutAttributesManager_->getNumObjects(),
      assetAttributesManager_->getNumObjects(), navmeshMap_.size(),
      semanticAttributesManager_->getNumObjects());
}  // namespace attributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
