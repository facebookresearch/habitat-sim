// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneDatasetAttributesManager.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace esp {
namespace metadata {

using attributes::SceneDatasetAttributes;
namespace managers {
SceneDatasetAttributes::ptr SceneDatasetAttributesManager::createObject(
    const std::string& datasetHandle,
    bool registerTemplate) {
  std::string msg;
  SceneDatasetAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      datasetHandle, msg, registerTemplate);

  if (nullptr != attrs) {
    LOG(INFO) << msg << " dataset attributes created"
              << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // SceneDatasetAttributesManager::createObject

SceneDatasetAttributes::ptr
SceneDatasetAttributesManager::initNewObjectInternal(
    const std::string& datasetFilename,
    CORRADE_UNUSED bool builtFromConfig) {
  SceneDatasetAttributes::ptr newAttributes =
      this->constructFromDefault(datasetFilename);
  if (nullptr == newAttributes) {
    newAttributes = SceneDatasetAttributes::create(datasetFilename,
                                                   physicsAttributesManager_);
  }
  // attempt to set source directory if exists
  this->setFileDirectoryFromHandle(newAttributes);

  // set the handle of the physics manager that is used for this newly-made
  // dataset
  newAttributes->setPhysicsManagerHandle(physicsManagerAttributesHandle_);
  // any internal default configuration here
  return newAttributes;
}  // SceneDatasetAttributesManager::initNewObjectInternal

void SceneDatasetAttributesManager::setValsFromJSONDoc(
    attributes::SceneDatasetAttributes::ptr dsAttribs,
    const io::JsonGenericValue& jsonConfig) {
  // dataset root directory to build paths from
  std::string dsDir = dsAttribs->getFileDirectory();
  // process stages
  readDatasetJSONCell(dsDir, "stages", jsonConfig,
                      dsAttribs->getStageAttributesManager());

  // process objects
  readDatasetJSONCell(dsDir, "objects", jsonConfig,
                      dsAttribs->getObjectAttributesManager());

  // process light setups - implement handling light setups
  readDatasetJSONCell(dsDir, "light_setups", jsonConfig,
                      dsAttribs->getLightLayoutAttributesManager());

  // process scene instances - implement handling scene instances
  readDatasetJSONCell(dsDir, "scene_instances", jsonConfig,
                      dsAttribs->getSceneAttributesManager());

  // process navmesh instances
  loadAndValidateMap(dsDir, "navmesh_instances", jsonConfig,
                     dsAttribs->editNavmeshMap());

  // process semantic scene descriptor instances
  loadAndValidateMap(dsDir, "semantic_scene_descriptor_instances", jsonConfig,
                     dsAttribs->editSemanticSceneDescrMap());

}  // SceneDatasetAttributesManager::setValsFromJSONDoc

void SceneDatasetAttributesManager::loadAndValidateMap(
    const std::string& dsDir,
    const std::string& jsonTag,
    const io::JsonGenericValue& jsonConfig,
    std::map<std::string, std::string>& map) {
  // load values into map
  io::readMember<std::map<std::string, std::string>>(jsonConfig,
                                                     jsonTag.c_str(), map);

  // now verify that all entries in map exist.  If not replace entry with
  // dsDir-prepended entry
  for (std::pair<const std::string, std::string>& entry : map) {
    const std::string loc = entry.second;
    if (!Cr::Utility::Directory::exists(loc)) {
      std::string newLoc = Cr::Utility::Directory::join(dsDir, loc);
      if (!Cr::Utility::Directory::exists(newLoc)) {
        LOG(WARNING) << "SceneDatasetAttributesManager::loadAndValidateMap : "
                     << jsonTag << " Value : " << loc
                     << " not found on disk as absolute path or relative to "
                     << dsDir;
      } else {
        // replace value with dataset-augmented absolute path
        map[entry.first] = newLoc;
      }
    }  // loc does not exist
  }    // for each loc
}  // SceneDatasetAttributesManager::loadAndValidateMap

// using type deduction
template <typename U>
void SceneDatasetAttributesManager::readDatasetJSONCell(
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
      // 1. "default_attributes" : a single attributes default of the specified
      // type.
      if (jCell.HasMember("default_attributes")) {
        if (!jCell["default_attributes"].IsObject()) {
          LOG(WARNING)
              << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
              << tag
              << ".default attributes\" cell in JSON config unable to "
                 "be parsed to set default attributes so skipping.";
        } else {
          // load attributes as default from file, do not register
          auto attr = attrMgr->buildObjectFromJSONDoc(
              "default_attributes", jCell["default_attributes"]);
          if (nullptr == attr) {
            LOG(WARNING)
                << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
                << tag
                << ".default attributes\" cell failed to successfully "
                   "create an attributes, so skipping.";
          } else {
            // set attributes as defaultObject_ in attrMgr.
            attrMgr->setDefaultObject(attr);
            LOG(INFO)
                << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
                << tag
                << ".default attributes\" set in Attributes Manager from JSON.";
          }
        }  // if is an object
      }    // if has default attributes cell

      // 2. "paths" an array of paths to search for appropriately typed config
      // files.
      if (jCell.HasMember("paths")) {
        if (!jCell["paths"].IsObject()) {
          LOG(WARNING)
              << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
              << tag
              << ".paths\" cell in JSON config unable to be parsed as "
                 "an array to determine search paths so skipping.";
        } else {
          const auto& pathsObj = jCell["paths"];
          if (pathsObj.HasMember(".json")) {
            if (!pathsObj[".json"].IsArray()) {
              LOG(WARNING)
                  << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
                  << tag
                  << ".paths.\".json\"\" cell in JSON config unable to be "
                     "parsed as "
                     "an array to determine search paths for json config so "
                     "skipping.";
            } else {
              const auto& paths = pathsObj[".json"];
              attrMgr->buildCfgPathsFromJSONAndLoad(dsDir, paths);
            }
          }  // if has member ".json"
             // TODO support other extention tags
        }    // if paths cell is an object
      }      // if has paths cell
      // 3. "configs" : an array of json cells defining customizations to
      // existing attributes.
      if (jCell.HasMember("configs")) {
        if (!jCell["configs"].IsArray()) {
          LOG(WARNING)
              << "SceneDatasetAttributesManager::readDatasetJSONCell : \""
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
}  // SceneDatasetAttributesManager::readDatasetJSONCell

template <typename U>
void SceneDatasetAttributesManager::readDatasetConfigsJSONCell(
    const std::string& dsDir,
    const char* tag,
    const io::JsonGenericValue& jCell,
    const U& attrMgr) {
  // every cell within configs array must have an attributes tag
  if ((!jCell.HasMember("attributes")) || (!jCell["attributes"].IsObject())) {
    LOG(WARNING)
        << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : \""
        << tag
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
  std::string origObjHandle = "";
  std::string newTemplateHandle = "";
  std::string newTemplateSrcDir = "";
  // try to find original file name for attributes
  if (io::readMember<std::string>(jCell, "original_file", originalFile)) {
    // verify that a template with this field as the original file was loaded.
    std::vector<std::string> handles =
        attrMgr->getObjectHandlesBySubstring(originalFile, true);
    if (handles.size() == 0) {
      LOG(WARNING)
          << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : \""
          << tag
          << ".configs\" cell element in JSON config specified source file : "
          << originalFile << " which cannot be found, so skipping.";
      return;
    }
    origObjHandle = handles[0];
    validCell = true;
    origFileNameSpecified = true;
  }

  // try to find new template name for attributes
  if (io::readMember<std::string>(jCell, "template_handle",
                                  newTemplateHandle)) {
    // if a new template handle has been specified, then this is a valid
    // configuration cell only if either an original to copy from or a source
    // directory for this template's new assets is specified.
    validCell = true;
    newNameSpecified = true;
  }
  // if neither handle is specified, cell will fail
  if (!validCell) {
    LOG(WARNING)
        << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : \""
        << tag
        << ".configs\" cell element in JSON config lacks required data to "
           "construct configuration override (either an original_file or a "
           "template_handle must be provided) so skipping.";
    return;
  }
  // set registration handle
  std::string regHandle =
      (newNameSpecified ? newTemplateHandle : origObjHandle);

  // if has original file but no template handle, will retrieve/load template
  // with given name, modify it, and save it with original name.  If template
  // handle is given it will save it with this handle instead.  If no original
  // file is given, will create new default template and save it with template
  // handle.

  // create attributes using original file name if specified, otherwise, create
  // from default and set new template handle upon registration.
  if (origFileNameSpecified) {
    // get copy of object if exists, else create object.  By here origObjHandle
    // is known to be legitimate file
    auto attr = attrMgr->getObjectCopyByHandle(origObjHandle);
    if (nullptr == attr) {
      LOG(WARNING)
          << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : "
          << attrMgr->getObjectType() << " : Attempting to make a copy of "
          << origObjHandle
          << " failing so creating and registering a new object.";
      attr = attrMgr->createObject(origObjHandle, true);
      if (nullptr == attr) {
        LOG(WARNING)
            << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : \""
            << tag << ".configs\" cell element's original file ("
            << originalFile
            << ") failed to successfully create a base attributes to modify, "
               "so skipping.";
        return;
      }
    }  // create copy/new object using old object file name
    // set copied object's handle based on registration handle
    attr->setHandle(regHandle);
    // object is available now. Modify it using json tag data
    attrMgr->setValsFromJSONDoc(attr, jCell["attributes"]);
    // register object
    attrMgr->registerObject(attr, regHandle);
  } else {  // orig file name not specified, create a new object
    // create a default object
    auto attr = attrMgr->createDefaultObject(newTemplateHandle, false);
    // if null then failed for some reason to create a new default object.
    if (nullptr == attr) {
      LOG(WARNING)
          << "SceneDatasetAttributesManager::readDatasetConfigsJSONCell : \""
          << tag
          << ".configs\" cell element failed to successfully create an "
             "attributes, so skipping.";
      return;
    }
    // set attributes' setFileDirectory :  use dataset directory, since all
    // assets will be named relative to this.
    attr->setFileDirectory(dsDir);

    // object is available now. Modify it using json tag data
    attrMgr->setValsFromJSONDoc(attr, jCell["attributes"]);
    // register object
    attrMgr->registerObject(attr, regHandle);
  }  // if original filename was specified else
}  // SceneDatasetAttributesManager::readDatasetConfigsJSONCell

int SceneDatasetAttributesManager::registerObjectFinalize(
    attributes::SceneDatasetAttributes::ptr SceneDatasetAttributes,
    const std::string& SceneDatasetAttributesHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by SceneDatasetAttributesHandle, or the next available
  // ID if not found.
  int datasetTemplateID = this->addObjectToLibrary(
      SceneDatasetAttributes, SceneDatasetAttributesHandle);
  return datasetTemplateID;
}  // SceneDatasetAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
