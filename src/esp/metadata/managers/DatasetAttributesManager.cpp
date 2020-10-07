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
  // dataset root directory to build paths from
  std::string dsDir = dsAttribs->getFileDirectory();
  // process stages
  readDatasetJSONCell(dsDir, "stages", jsonConfig,
                      dsAttribs->getStageAttributesManager());

  // process objects
  readDatasetJSONCell(dsDir, "objects", jsonConfig,
                      dsAttribs->getObjectAttributesManager());

  // process light setups - implement handling light setups TODO
  // readDatasetJSONCell(dsDir,"light setups", jsonConfig,
  //                     dsAttribs->getLightsAttributesManager());

  // process scene instances - implement handling scene instances TODO
  // readDatasetJSONCell(dsDir,"scene instances", jsonConfig,
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
    const std::string& dsDir,
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
          const auto& paths = jCell["paths"];
          attrMgr->buildCfgPathsFromJSONAndLoad(dsDir, paths);
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
          const auto& configsAra = jCell["configs"];
          for (rapidjson::SizeType i = 0; i < configsAra.Size(); i++) {
            const auto& configCell = configsAra[i];
            readDatasetConfigsJSONCell(dsDir, tag, configCell, attrMgr);
          }  // for each cell in configs array
        }    // if is array
      }      // if has configs cell
    }        // if cell is an object
  }          // if cell exists
}  // DatasetAttributesManager::readDatasetJSONCell

template <typename U>
void DatasetAttributesManager::readDatasetConfigsJSONCell(
    const std::string& dsDir,
    const char* tag,
    const io::JsonGenericValue& jCell,
    const U& attrMgr) {
  // every cell within configs array must have an attributes tag
  if ((!jCell.HasMember("attributes")) || (!jCell["attributes"].IsObject())) {
    LOG(WARNING)
        << "DatasetAttributesManager::readDatasetConfigsJSONCell : \"" << tag
        << ".configs\" cell element in JSON config lacks required data to "
           "construct configuration override (an attributes tag and data "
           "describing the overrides is not found), so skipping.";
    return;
  }  // attributes must be specified.

  // every cell within configs array will have at least one of ("original
  // file","template handle")
  bool validCell = false;
  bool origFileNameSpecified = false;
  bool newNameSpecified = false;
  std::string originalFile = "";
  std::string newTemplateHandle = "";
  // try to find original attributes file name
  if (io::jsonIntoVal<std::string>(jCell, "original file", originalFile)) {
    originalFile = Cr::Utility::Directory::join(dsDir, originalFile);
    if (!this->isValidFileName(originalFile)) {
      LOG(WARNING)
          << "DatasetAttributesManager::readDatasetConfigsJSONCell : \"" << tag
          << ".configs\" cell element in JSON config specified source file : "
          << originalFile << " which cannot be found, so skipping.";
      return;
    }
    validCell = true;
    origFileNameSpecified = true;
  }
  // try to find new attributes template name
  if (io::jsonIntoVal<std::string>(jCell, "template handle",
                                   newTemplateHandle)) {
    validCell = true;
    newNameSpecified = true;
  }
  // if neither handle is specified, cell will fail
  if (!validCell) {
    LOG(WARNING)
        << "DatasetAttributesManager::readDatasetConfigsJSONCell : \"" << tag
        << ".configs\" cell element in JSON config lacks required data to "
           "construct configuration override (either an original file or a "
           "template handle must be provided) so skipping.";
    return;
  }
  // set registration handle
  std::string regHandle = (newNameSpecified ? newTemplateHandle : originalFile);

  // if has original file but no template handle, will retrieve/load template
  // with given name, modify it, and save it with original name.  If template
  // handle is given it will save it with this handle instead.  If no original
  // file is given, will create new default template and save it with template
  // handle.

  // create attributes using original file name if specified, otherwise, create
  // from default and set new template handle upon registration.
  if (origFileNameSpecified) {
    // get object if exists, else create object.  By here originalFile is known
    // to be legitimate directory
    auto oldAttr = attrMgr->getObjectCopyByHandle(originalFile);
    if (nullptr == oldAttr) {
      oldAttr = attrMgr->createObject(originalFile);
      if (nullptr == oldAttr) {
        LOG(WARNING)
            << "DatasetAttributesManager::readDatasetConfigsJSONCell : \""
            << tag << ".configs\" cell element's original file ("
            << originalFile
            << ") failed to successfully create a base attributes to modify, "
               "so skipping.";
        return;
      }
      // old object is available now. Modify it using json tag data
      attrMgr->setValsFromJSONDoc(oldAttr, jCell["attributes"]);
      // register object
      attrMgr->registerObject(oldAttr, regHandle);
    } else {
      // load attributes as default from file, do not register
      auto attr = attrMgr->buildObjectFromJSONDoc(newTemplateHandle,
                                                  jCell["attributes"]);
      if (nullptr == attr) {
        LOG(WARNING)
            << "DatasetAttributesManager::readDatasetConfigsJSONCell : \""
            << tag
            << ".configs\" cell element failed to successfully create an "
               "attributes, so skipping.";
        return;
      }
      // register new object
      attrMgr->registerObject(attr, regHandle);
    }
  }  // if original filename was specified
}  // DatasetAttributesManager::readDatasetConfigsJSONCell

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
