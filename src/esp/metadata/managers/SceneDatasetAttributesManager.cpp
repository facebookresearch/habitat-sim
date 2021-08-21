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

SceneDatasetAttributesManager::SceneDatasetAttributesManager(
    PhysicsAttributesManager::ptr physicsAttributesMgr)
    : AttributesManager<attributes::SceneDatasetAttributes,
                        core::ManagedObjectAccess::Share>::
          AttributesManager("Dataset", "scene_dataset_config.json"),
      physicsAttributesManager_(std::move(physicsAttributesMgr)) {
  // build this manager's copy ctor map
  this->copyConstructorMap_["SceneDatasetAttributes"] =
      &SceneDatasetAttributesManager::createObjectCopy<
          attributes::SceneDatasetAttributes>;
}  // SceneDatasetAttributesManager ctor

SceneDatasetAttributes::ptr SceneDatasetAttributesManager::createObject(
    const std::string& datasetHandle,
    bool registerTemplate) {
  std::string msg;
  SceneDatasetAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      datasetHandle, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG() << msg << "dataset attributes created"
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

  // process articulated objects
  // TODO Need to construct manager to consume readDatasetJSONCell
  // All of the below will be replaced by readDatasetJSONCell call
  const char* tag = "articulated_objects";
  if (jsonConfig.HasMember(tag)) {
    namespace Dir = Cr::Utility::Directory;
    if (!jsonConfig[tag].IsObject()) {
      ESP_WARNING()
          << "\"" << tag
          << "\" cell in JSON config not appropriately configured. Skipping.";
    } else {
      const auto& jCell = jsonConfig[tag];
      if (jCell.HasMember("paths")) {
        if (!jCell["paths"].IsObject()) {
          ESP_WARNING()
              << "(Articulated Object) : \"" << tag
              << ".paths\" cell in JSON config unable to be parsed as "
                 "a JSON object to determine search paths so skipping.";
        } else {
          const auto& pathsObj = jCell["paths"];
          bool pathsWarn = false;
          std::string pathsWarnType = "";
          const char* urdfPathExt = ".urdf";
          if (pathsObj.HasMember(urdfPathExt)) {
            if (!pathsObj[urdfPathExt].IsArray()) {
              pathsWarn = true;
              pathsWarnType = urdfPathExt;
            } else {
              const auto& aoPathsObj = pathsObj[urdfPathExt];
              //** replaces call to buildCfgPathsFromJSONAndLoad in AOManager
              // for each entry in ao paths array object
              for (rapidjson::SizeType i = 0; i < aoPathsObj.Size(); ++i) {
                if (!aoPathsObj[i].IsString()) {
                  ESP_ERROR() << "(Articulated Object) : Invalid path "
                                 "value in file path array element @ idx"
                              << i << ". Skipping.";
                  continue;
                }
                // aoPathsObj entry is a string, assumed to be relative to the
                // directory where the ds attribs resides
                std::string absolutePath =
                    Dir::join(dsDir, aoPathsObj[i].GetString());

                // getting a list of all directories that match possible glob
                // wildcards
                std::vector<std::string> globPaths = io::globDirs(absolutePath);
                if (globPaths.size() > 0) {
                  std::vector<std::string> aoFilePaths;
                  // iterate through every entry
                  for (const auto& globPath : globPaths) {
                    // load all object templates available as configs in
                    // absolutePath
                    ESP_WARNING()
                        << "(Articulated Object) : Glob path result for"
                        << absolutePath << ":" << globPath;
                    // each globPath entry represents real unique entry on disk

                    //****replaces call to loadAllConfigsFromPath in AOManager

                    // Check if directory
                    const bool dirExists = Dir::isDirectory(globPath);
                    if (dirExists) {
                      ESP_DEBUG()
                          << "(Articulated Object) : "
                             "Parsing articulated object library directory: " +
                                 globPath;
                      for (auto& file :
                           Dir::list(globPath, Dir::Flag::SortAscending)) {
                        std::string absoluteSubfilePath =
                            Dir::join(globPath, file);
                        if (Cr::Utility::String::endsWith(absoluteSubfilePath,
                                                          urdfPathExt)) {
                          aoFilePaths.push_back(absoluteSubfilePath);
                        }
                      }
                    } else if (Cr::Utility::String::endsWith(globPath,
                                                             urdfPathExt)) {
                      aoFilePaths.push_back(globPath);
                    } else {  // neither a directory or a file
                      ESP_WARNING() << "(Articulated Object) : Parsing "
                                       "articulated objects  : Cannot find"
                                    << globPath
                                    << "as sub directory or as config file. "
                                       "Aborting parse.";
                      continue;
                    }  // if dirExists else
                       //**//** replaces call to loadAllFileBasedTemplates
                  }    // for each glob path

                  // check if any exist, may be more than 1 since may have
                  // traversing a subdirectory
                  if (aoFilePaths.size() > 0) {
                    std::string ao_dir = Dir::path(aoFilePaths[0]);
                    ESP_DEBUG() << "(Articulated Object) : Loading"
                                << aoFilePaths.size() << "" << this->objectType_
                                << "templates found in" << ao_dir;
                    for (int i = 0; i < aoFilePaths.size(); ++i) {
                      auto aoModelFileName = aoFilePaths[i];
                      ESP_DEBUG() << "(Articulated Object) : "
                                     "Found Articulated Object Model file :"
                                  << aoModelFileName;

                      // set k-v pairs here.
                      auto key =
                          Corrade::Utility::Directory::splitExtension(
                              Corrade::Utility::Directory::splitExtension(
                                  Corrade::Utility::Directory::filename(
                                      aoModelFileName))
                                  .first)
                              .first;

                      dsAttribs->setArticulatedObjectModelFilename(
                          key, aoModelFileName);
                    }
                  }
                  ESP_DEBUG()
                      << "Specified" << std::to_string(aoFilePaths.size())
                      << "articulated object model filenames specified "
                         "in path GLOB object :"
                      << absolutePath << ".";

                  //**//** end call to loadAllFileBasedTemplates
                  //**** end call to loadAllConfigsFromPath in AOManager

                } else {
                  ESP_WARNING()
                      << "(Articulated Object) : No Glob path result for"
                      << absolutePath;
                  continue;
                }
              }  // for every path object in list in json

              ESP_DEBUG() << "(Articulated Object) :"
                          << std::to_string(aoPathsObj.Size())
                          << "paths specified in JSON doc for articulated "
                             "object model files.";
              //** end call to buildCfgPathsFromJSONAndLoad in AOManager
            }
          }

          if (pathsWarn) {
            ESP_WARNING()
                << "\"" << tag << ".paths[" << Mn::Debug::nospace
                << pathsWarnType << Mn::Debug::nospace
                << "] cell in JSON config unable to be parsed as an array to "
                   "determine search paths for json configs so skipping.";
          }
        }  // if paths cell is an object
      }    // if has paths cell
    }
  }  // if has articulated_objects tag

  //// End temporary articulated object path loading

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
        ESP_WARNING() << jsonTag << "Value :" << loc
                      << "not found on disk as absolute path or relative to"
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
      ESP_WARNING()
          << "\"" << tag
          << "\" cell in JSON config not appropriately configured. Skipping.";

    } else {
      const auto& jCell = jsonConfig[tag];
      // process JSON jCell here - this cell potentially holds :
      // 1. "default_attributes" : a single attributes default of the
      // specified type.
      if (jCell.HasMember("default_attributes")) {
        if (!jCell["default_attributes"].IsObject()) {
          ESP_WARNING()
              << "\"" << Mn::Debug::nospace << tag << Mn::Debug::nospace
              << ".default_attributes\" cell in JSON config unable to "
                 "be parsed to set default attributes so skipping.";
        } else {
          // load attributes as default from file, do not register
          auto attr = attrMgr->buildObjectFromJSONDoc(
              "default_attributes", jCell["default_attributes"]);
          if (nullptr == attr) {
            ESP_WARNING()
                << "\"" << tag
                << ".default_attributes\" cell failed to successfully "
                   "create an attributes, so skipping.";
          } else {
            // set attributes as defaultObject_ in attrMgr.
            attrMgr->setDefaultObject(attr);
            ESP_WARNING()
                << "\"" << Mn::Debug::nospace << tag << Mn::Debug::nospace
                << ".default_attributes\" set in Attributes Manager from JSON.";
          }
        }  // if is an object
      }    // if has default_attributes cell

      // 2. "paths" an array of paths to search for appropriately typed config
      // files.
      if (jCell.HasMember("paths")) {
        if (!jCell["paths"].IsObject()) {
          ESP_WARNING()
              << "\"" << Mn::Debug::nospace << tag << Mn::Debug::nospace
              << ".paths\" cell in JSON config unable to be parsed as "
                 "a JSON object to determine search paths so skipping.";
        } else {
          const auto& pathsObj = jCell["paths"];
          bool pathsWarn = false;
          std::string pathsWarnType = "";
          if (pathsObj.HasMember(".json")) {
            if (!pathsObj[".json"].IsArray()) {
              pathsWarn = true;
              pathsWarnType = ".json";
            } else {
              const auto& paths = pathsObj[".json"];
              attrMgr->buildJSONCfgPathsFromJSONAndLoad(dsDir, paths);
            }
          }
          if (pathsObj.HasMember(".glb")) {
            if (!pathsObj[".glb"].IsArray()) {
              pathsWarn = true;
              pathsWarnType = ".glb";
            } else {
              const auto& paths = pathsObj[".glb"];
              attrMgr->buildAttrSrcPathsFromJSONAndLoad(dsDir, ".glb", paths);
            }
          }
          // TODO support other extension tags
          if (pathsWarn) {
            ESP_WARNING()
                << "\"" << tag << ".paths\"[" << Mn::Debug::nospace
                << pathsWarnType << Mn::Debug::nospace
                << "] cell in JSON config unable to be parsed as an array to "
                   "determine search paths for json configs so skipping.";
          }
        }  // if paths cell is an object
      }    // if has paths cell
      // 3. "configs" : an array of json cells defining customizations to
      // existing attributes.
      if (jCell.HasMember("configs")) {
        if (!jCell["configs"].IsArray()) {
          ESP_WARNING() << "\"" << tag
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
    ESP_WARNING()
        << "\"" << tag
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
      ESP_WARNING()
          << "\"" << tag
          << ".configs\" cell element in JSON config specified source file :"
          << originalFile << "which cannot be found, so skipping.";
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
    ESP_WARNING()
        << "\"" << tag
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

  // create attributes using original file name if specified, otherwise,
  // create from default and set new template handle upon registration.
  if (origFileNameSpecified) {
    // get copy of object if exists, else create object.  By here
    // origObjHandle is known to be legitimate file
    auto attr = attrMgr->getObjectCopyByHandle(origObjHandle);
    if (nullptr == attr) {
      ESP_WARNING() << attrMgr->getObjectType()
                    << ": Attempting to make a copy of" << origObjHandle
                    << "failing so creating and registering a new object.";
      attr = attrMgr->createObject(origObjHandle, true);
      if (nullptr == attr) {
        ESP_WARNING()
            << "\"" << tag << ".configs\" cell element's original file ("
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
      ESP_WARNING()
          << "\"" << tag
          << ".configs\" cell element failed to successfully create an "
             "attributes, so skipping.";
      return;
    }
    // set attributes' setFileDirectory :  use dataset directory, since all
    // assets will be named relative to this.
    attr->setFileDirectory(dsDir);

    // default object is available now. Modify it using json tag data
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
  // template referenced by SceneDatasetAttributesHandle, or the next
  // available ID if not found.
  int datasetTemplateID = this->addObjectToLibrary(
      SceneDatasetAttributes, SceneDatasetAttributesHandle);
  return datasetTemplateID;
}  // SceneDatasetAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
