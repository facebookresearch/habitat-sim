// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetAttributesManager.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace metadata {

using attributes::DatasetAttributes;
namespace managers {
DatasetAttributes::ptr DatasetAttributesManager::createObject(
    const std::string& datasetHandle,
    bool registerTemplate) {
  std::string msg;
  DatasetAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      datasetHandle, msg, registerTemplate);

  if (nullptr != attrs) {
    LOG(INFO) << msg << " dataset attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // DatasetAttributesManager::createObject

DatasetAttributes::ptr DatasetAttributesManager::initNewObjectInternal(
    const std::string& datasetFilename) {
  DatasetAttributes::ptr newAttributes = this->constructFromDefault();
  if (nullptr == newAttributes) {
    newAttributes =
        DatasetAttributes::create(datasetFilename, physicsAttributesManager_);
  }
  // attempt to set source directory if exists
  this->setFileDirectoryFromHandle(newAttributes);

  // set the handle of the physics manager that is used for this newly-made
  // dataset
  newAttributes->setPhysicsManagerHandle(physicsManagerAttributesHandle_);
  // any internal default configuration here
  return newAttributes;
}  // DatasetAttributesManager::initNewObjectInternal

void DatasetAttributesManager::setValsFromJSONDoc(
    attributes::DatasetAttributes::ptr dsAttribs,
    const io::JsonGenericValue& jsonConfig) {
  // process stages
  readDatasetJSONCell(dsAttribs, "stages", jsonConfig,
                      dsAttribs->getStageAttributesManager());

  // process objects
  readDatasetJSONCell(dsAttribs, "objects", jsonConfig,
                      dsAttribs->getObjectAttributesManager());

  // process light setups - implement handling light setups TODO
  // readDatasetJSONCell(dsAttribs,"light setups", jsonConfig,
  //                     dsAttribs->getLightsAttributesManager());

  // process scene instances - implement handling scene instances TODO
  // readDatasetJSONCell(dsAttribs,"scene instances", jsonConfig,
  //                     dsAttribs->getSceneInstanceManager());

  // process navmesh instances
  if (jsonConfig.HasMember("navmesh instances")) {
    if (!jsonConfig["navmesh instances"].IsObject()) {
      dispCellConfigError("navmesh instances");
    } else {
      const auto& jCell = jsonConfig["navmesh instances"];
      // implement handling navmesh instances TODO
    }
  }

  // process semantic scene descriptor instances
  if (jsonConfig.HasMember("semantic scene descriptor instances")) {
    if (!jsonConfig["semantic scene descriptor instances"].IsObject()) {
      dispCellConfigError("semantic scene descriptor instances");
    } else {
      const auto& jCell = jsonConfig["semantic scene descriptor instances"];
      // implement handling semantic scene descriptor instances TODO
    }
  }

}  // DatasetAttributesManager::setValsFromJSONDoc

// using type deduction
template <typename U>
void DatasetAttributesManager::readDatasetJSONCell(
    attributes::DatasetAttributes::ptr dsAttribs,
    const char* tag,
    const io::JsonGenericValue& jsonConfig,
    const U& attrMgr) {
  if (jsonConfig.HasMember(tag)) {
    if (!jsonConfig[tag].IsObject()) {
      dispCellConfigError(tag);
    } else {
      const auto& jCell = jsonConfig[tag];
      // process JSON jCell here - this cell potentially holds :
      // 1. "default attributes" : a single attributes default of the specified
      // type.
      if (jCell.HasMember("default attributes")) {
        if (!jCell["default attributes"].IsObject()) {
          LOG(WARNING) << "DatasetAttributesManager::readDatasetJSONCell : \""
                       << tag
                       << ".default attributes\" cell in JSON config unable to "
                          "be parsed to set default attributes so skipping.";
        } else {
          // load attributes as default from file, do not register
          auto attr = attrMgr->buildObjectFromJSONDoc(
              "default attributes", jCell["default attributes"]);
          if (nullptr == attr) {
            LOG(WARNING) << "DatasetAttributesManager::readDatasetJSONCell : \""
                         << tag
                         << ".default attributes\" cell failed to successfully "
                            "create an attributes, so skipping.";
          } else {
            // set attributes as defaultObject_ in attrMgr.
            attrMgr->setDefaultObject(attr);
            LOG(INFO)
                << "DatasetAttributesManager::readDatasetJSONCell : \"" << tag
                << ".default attributes\" set in Attributes Manager from JSON.";
          }
        }  // if is an object
      }    // if has default attributes cell

      // 2. "paths" an array of paths to search for appropriately typed config
      // files.
      if (jCell.HasMember("paths")) {
        if (!jCell["paths"].IsArray()) {
          LOG(WARNING) << "DatasetAttributesManager::readDatasetJSONCell : \""
                       << tag
                       << ".paths\" cell in JSON config unable to be parsed as "
                          "an array to determine search paths so skipping.";
        } else {
          std::string configDirectory = dsAttribs->getFileDirectory();
          const auto& paths = jsonConfig["paths"];
          attrMgr->buildCfgPathsFromJSONAndLoad(configDirectory, paths);
        }  // if is array
      }    // if has paths cell
      // 3. "configs" : an array of json cells defining customizations to
      // existing attributes.
      if (jCell.HasMember("configs")) {
        if (!jCell["configs"].IsArray()) {
          LOG(WARNING) << "DatasetAttributesManager::readDatasetJSONCell : \""
                       << tag
                       << ".configs\" cell in JSON config unable to be parsed "
                          "as an array to determine search paths so skipping.";

        } else {
        }
      }  // if has configs cell
    }
  }
}  // DatasetAttributesManager::readDatasetJSONCell

int DatasetAttributesManager::registerObjectFinalize(
    attributes::DatasetAttributes::ptr datasetAttributes,
    const std::string& datasetAttributesHandle) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by datasetAttributesHandle, or the next available ID
  // if not found.
  int datasetTemplateID =
      this->addObjectToLibrary(datasetAttributes, datasetAttributesHandle);
  return datasetTemplateID;
}  // DatasetAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
