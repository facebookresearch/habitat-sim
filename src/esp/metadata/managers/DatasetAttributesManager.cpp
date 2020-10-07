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

  // set the handle of the physics manager that is used for this newly-made
  // dataset
  newAttributes->setPhysicsManagerHandle(physicsManagerAttributesHandle_);
  // any internal default configuration here
  return newAttributes;
}  // DatasetAttributes::initNewObjectInternal

void DatasetAttributesManager::setValsFromJSONDoc(
    attributes::DatasetAttributes::ptr datasetAttributes,
    const io::JsonGenericValue& jsonConfig) {
  // get dataset managers to handle loading
  // auto assetMgrs = datasetAttributes->getAssetAttributesManager();
  const ObjectAttributesManager::ptr objectMgrs =
      datasetAttributes->getObjectAttributesManager();
  const StageAttributesManager::ptr stageMgrs =
      datasetAttributes->getStageAttributesManager();

  // process stages
  readDatasetJSONCell("stages", jsonConfig,
                      datasetAttributes->getStageAttributesManager());
  // process objects
  readDatasetJSONCell("objects", jsonConfig,
                      datasetAttributes->getObjectAttributesManager());

}  // DatasetAttributesManager::setValsFromJSONDoc

// using type deduction
template <typename U>
void DatasetAttributesManager::readDatasetJSONCell(
    const char* tag,
    const io::JsonGenericValue& jsonConfig,
    const U& attrMgr) {
  if (jsonConfig.HasMember(tag)) {
    if (!jsonConfig[tag].IsObject()) {
      LOG(WARNING) << "DatasetAttributesManager::setValsFromJSONDoc : "
                      "\"Stages\" cell in JSON config not appropriately "
                      "configured. Skipping.";
      return;
    } else {
      const io::JsonGenericValue& jsonCell = jsonConfig[tag];
      // process JSON here
    }
  }  // if has tag else
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
