// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneDatasetAttributesManager.h"

#include <utility>

#include "esp/io/Json.h"

namespace esp {
using core::managedContainers::ManagedObjectAccess;
namespace metadata {

using attributes::SceneDatasetAttributes;
namespace managers {

SceneDatasetAttributesManager::SceneDatasetAttributesManager(
    PhysicsAttributesManager::ptr physicsAttributesMgr,
    PbrShaderAttributesManager::ptr pbrShaderAttributesMgr)
    : AttributesManager<SceneDatasetAttributes, ManagedObjectAccess::Share>::
          AttributesManager("Dataset", "scene_dataset_config.json"),
      physicsAttributesManager_(std::move(physicsAttributesMgr)),
      pbrShaderAttributesManager_(std::move(pbrShaderAttributesMgr)) {
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
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << msg << " dataset attributes created"
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
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  // set the handle of the physics manager and default Pbr/Ibl shader config
  // that is used for this newly-made dataset
  newAttributes->setPhysicsManagerHandle(physicsManagerAttributesHandle_);
  newAttributes->setDefaultPbrShaderAttrHandle(
      defaultPbrShaderAttributesHandle_);
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
  readDatasetJSONCell(dsDir, "articulated_objects", jsonConfig,
                      dsAttribs->getAOAttributesManager());

  // process light setups - implement handling light setups
  readDatasetJSONCell(dsDir, "light_setups", jsonConfig,
                      dsAttribs->getLightLayoutAttributesManager());

  // process PBR/IBL Shader configurations. Only will be relevant for
  // PBR-rendered assets, will be available to all datasets that are loaded.
  readDatasetJSONCell(dsDir, "pbr_ibl_configs", jsonConfig,
                      pbrShaderAttributesManager_);

  // process scene instances - implement handling scene instances
  readDatasetJSONCell(dsDir, "scene_instances", jsonConfig,
                      dsAttribs->getSceneInstanceAttributesManager());

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
    if (!Cr::Utility::Path::exists(loc)) {
      std::string newLoc = Cr::Utility::Path::join(dsDir, loc);
      if (!Cr::Utility::Path::exists(newLoc)) {
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "`" << jsonTag << "` Value : `" << loc
            << "` not found on disk as absolute path or relative to `" << dsDir
            << "`";
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
  io::JsonGenericValue::ConstMemberIterator jsonIter =
      jsonConfig.FindMember(tag);
  if (jsonIter != jsonConfig.MemberEnd()) {
    if (!jsonIter->value.IsObject()) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "`" << tag
          << "` cell in JSON config not appropriately configured. Skipping.";

    } else {
      const auto& jCell = jsonIter->value;
      // process JSON jCell here - this cell potentially holds :
      // 1. "default_attributes" : a single attributes default of the
      // specified type.
      io::JsonGenericValue::ConstMemberIterator jsonDfltObjIter =
          jCell.FindMember("default_attributes");

      if (jsonDfltObjIter != jCell.MemberEnd()) {
        if (!jsonDfltObjIter->value.IsObject()) {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "`" << tag
              << ".default_attributes` cell in JSON config unable to "
                 "be parsed to set default attributes so skipping.";
        } else {
          // load attributes as default from file, do not register
          auto attr = attrMgr->buildObjectFromJSONDoc("default_attributes",
                                                      jsonDfltObjIter->value);
          if (nullptr == attr) {
            ESP_WARNING(Mn::Debug::Flag::NoSpace)
                << "`" << tag
                << ".default_attributes` cell failed to successfully "
                   "create an attributes, so skipping.";
          } else {
            // set attributes as defaultObject_ in attrMgr.
            attrMgr->setDefaultObject(attr);
            ESP_DEBUG(Mn::Debug::Flag::NoSpace)
                << "`" << tag
                << ".default_attributes` set in Attributes Manager from JSON.";
          }
        }  // if is an object
      }    // if has default_attributes cell

      // 2. "paths" an array of paths to search for appropriately typed config
      // files.
      io::JsonGenericValue::ConstMemberIterator jsonPathsIter =
          jCell.FindMember("paths");
      if (jsonPathsIter != jCell.MemberEnd()) {
        if (!jsonPathsIter->value.IsObject()) {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "`" << tag
              << "`.paths` cell in JSON config unable to be parsed as "
                 "a JSON object to determine search paths so skipping.";
        } else {
          const auto& pathsObj = jsonPathsIter->value;
          // iterate through all provided extensions
          for (rapidjson::Value::ConstMemberIterator it =
                   pathsObj.MemberBegin();
               it != pathsObj.MemberEnd(); ++it) {
            // for each key, assume it is an extension and attempt to parse
            const std::string ext{it->name.GetString()};
            io::JsonGenericValue::ConstMemberIterator pathsObjIter =
                pathsObj.FindMember(ext.c_str());
            if (!pathsObjIter->value.IsArray()) {
              ESP_WARNING(Mn::Debug::Flag::NoSpace)
                  << "`" << tag << ".paths`[" << ext
                  << "] cell in JSON config unable to be parsed as an array to "
                     "determine search paths for json configs so skipping.";
              continue;
            } else {
              const auto& paths = pathsObjIter->value;
              if (ext.find(".json") != std::string::npos) {
                attrMgr->buildJSONCfgPathsFromJSONAndLoad(dsDir, paths);
              } else {
                attrMgr->buildAttrSrcPathsFromJSONAndLoad(dsDir, ext, paths);
              }
            }
          }
        }  // if paths cell is an object
      }    // if has paths cell

      // 3. "configs" : an array of json cells defining customizations to
      // existing attributes.
      io::JsonGenericValue::ConstMemberIterator jsonCfgsIter =
          jCell.FindMember("configs");
      if (jsonCfgsIter != jCell.MemberEnd()) {
        if (!jsonCfgsIter->value.IsArray()) {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "`" << tag
              << ".configs` cell in JSON config unable to be parsed "
                 "as an array to determine search paths so skipping.";
        } else {
          const auto& configsAra = jsonCfgsIter->value;
          for (rapidjson::SizeType i = 0; i < configsAra.Size(); ++i) {
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
  io::JsonGenericValue::ConstMemberIterator jsonAttrIter =
      jCell.FindMember("attributes");
  if ((jsonAttrIter == jCell.MemberEnd()) ||
      (!jsonAttrIter->value.IsObject())) {
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "`" << tag
        << ".configs` cell element in JSON config lacks required data to "
           "construct configuration override (an attributes tag and data "
           "describing the overrides was not found), so skipping.";
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
    if (handles.empty()) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "`" << tag
          << ".configs` cell element in JSON config specified source file : `"
          << originalFile << "` which cannot be found, so skipping.";
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
    ESP_WARNING(Mn::Debug::Flag::NoSpace)
        << "`" << tag
        << ".configs` cell element in JSON config lacks required data to "
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
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << attrMgr->getObjectType() << ": Attempt to make a copy of `"
          << origObjHandle
          << "` failed so creating and registering a new object.";
      attr = attrMgr->createObject(origObjHandle, true);
      if (nullptr == attr) {
        ESP_WARNING(Mn::Debug::Flag::NoSpace)
            << "`" << tag << ".configs` cell element's original file `"
            << originalFile
            << "` failed to successfully create a base attributes to modify, "
               "so skipping.";
        return;
      }
    }  // create copy/new object using old object file name
    // set copied object's handle based on registration handle
    attr->setHandle(regHandle);
    // object is available now. Modify it using json tag data
    attrMgr->setValsFromJSONDoc(attr, jsonAttrIter->value);
    // register object
    attrMgr->registerObject(std::move(attr), regHandle);
  } else {  // orig file name not specified, create a new object
    // create a default object
    auto attr = attrMgr->createDefaultObject(newTemplateHandle, false);
    // if null then failed for some reason to create a new default object.
    if (nullptr == attr) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "`" << tag
          << ".configs` cell element failed to successfully create an "
             "attributes, so skipping.";
      return;
    }
    // set attributes' setFileDirectory :  use dataset directory, since all
    // assets will be named relative to this.
    attr->setFileDirectory(dsDir);

    // default object is available now. Modify it using json tag data
    attrMgr->setValsFromJSONDoc(attr, jsonAttrIter->value);
    // register object
    attrMgr->registerObject(std::move(attr), regHandle);
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
      std::move(SceneDatasetAttributes), SceneDatasetAttributesHandle);
  return datasetTemplateID;
}  // SceneDatasetAttributesManager::registerObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
