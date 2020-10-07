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
  DatasetAttributes::ptr attrs;
  std::string msg;
  if (this->isValidFileName(datasetHandle)) {
    // check if datasetHandle corresponds to an actual file descriptor
    // this method lives in class template.
    attrs = this->template createObjectFromFile<io::JsonDocument>(
        datasetHandle, registerTemplate);
    msg = "File (" + datasetHandle + ") Based";
  } else {
    // if name is not file descriptor, return default attributes.
    attrs = this->template createObjectFromFile<io::JsonDocument>(
        datasetHandle, registerTemplate);
    msg = "File (" + datasetHandle + ") not found so new, default";
  }

  if (nullptr != attrs) {
    LOG(INFO) << msg << " dataset attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // DatasetAttributesManager::createObject

DatasetAttributes::ptr DatasetAttributesManager::initNewObjectInternal(
    const std::string& datasetFilename) {
  auto datasetAttributes =
      DatasetAttributes::create(datasetFilename, physicsAttributesManager_);
  // set the handle of the physics manager that is used for this newly-made
  // dataset
  datasetAttributes->setPhysicsManagerHandle(physicsManagerAttributesHandle_);
  // any internal default configuration here
  return datasetAttributes;
}  // DatasetAttributes::initNewObjectInternal

DatasetAttributes::ptr DatasetAttributesManager::loadFromJSONDoc(
    const std::string& templateName,
    const io::JsonDocument& jsonConfig) {
  DatasetAttributes::ptr datasetAttributes =
      this->initNewObjectInternal(templateName);
  // get dataset managers to handle loading
  auto assetMgrs = datasetAttributes->getAssetAttributesManager();
  auto objectMgrs = datasetAttributes->getObjectAttributesManager();
  auto stageMgrs = datasetAttributes->getStageAttributesManager();

  // TODO add code to read dataset_config json

  return datasetAttributes;
}  // DatasetAttributesManager::loadFromJSONDoc

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
