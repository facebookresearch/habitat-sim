// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AOAttributesManager.h"
#include "AttributesManagerBase.h"
#include "esp/metadata/MetadataUtils.h"

namespace Cr = Corrade;
namespace esp {

namespace metadata {

using attributes::ArticulatedObjectAttributes;
namespace managers {

ArticulatedObjectAttributes::ptr AOAttributesManager::createObject(
    const std::string& aoFilename,
    bool registerTemplate) {
  std::string msg;
  ArticulatedObjectAttributes::ptr attrs =
      this->createFromJsonOrDefaultInternal(aoFilename, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << msg << " articulated object attributes created"
        << (registerTemplate ? " and registered." : ".");
  }
  return attrs;
}  // AOAttributesManager::createObject

void AOAttributesManager::setValsFromJSONDoc(
    ArticulatedObjectAttributes::ptr aoAttr,
    const io::JsonGenericValue& jsonConfig) {
  std::string urdf_filepath = "";
  // load the urdf filepath, prefixing with file path if not found
  if (io::readMember<std::string>(jsonConfig, "urdf_filepath", urdf_filepath)) {
    // If specified urdf_filepath is not found directly, prefix it with file
    // directory where the configuration file was found.
    if (!Corrade::Utility::Path::exists(urdf_filepath)) {
      urdf_filepath =
          Cr::Utility::Path::join(aoAttr->getFileDirectory(), urdf_filepath);
    }
    aoAttr->setURDFPath(urdf_filepath);
  }

  std::string render_asset = "";
  // load the render asset handle, prefixing with file path if not found
  if (io::readMember<std::string>(jsonConfig, "render_asset", render_asset)) {
    // If specified render_asset is not found directly, prefix it with file
    // directory where the configuration file was found.
    if (!Corrade::Utility::Path::exists(render_asset)) {
      render_asset =
          Cr::Utility::Path::join(aoAttr->getFileDirectory(), render_asset);
    }
    aoAttr->setRenderAssetHandle(render_asset);
  }

  // load the semantic id
  io::jsonIntoSetter<int>(jsonConfig, "semantic_id", [aoAttr](int semantic_id) {
    aoAttr->setSemanticId(semantic_id);
  });

  // set attributes shader type to use.  This may be overridden by a scene
  // instance specification.
  const std::string shaderTypeVal = getShaderTypeFromJsonDoc(jsonConfig);
  // if a known shader type val is specified in json, set that value for the
  // attributes, overriding constructor defaults.  Do not overwrite anything for
  // unknown
  if (shaderTypeVal !=
      getShaderTypeName(attributes::ObjectInstanceShaderType::Unspecified)) {
    aoAttr->setShaderType(shaderTypeVal);
  }

  // bool fixedBase,
  // float globalScale,
  // float massScale,
  // bool forceReload,
  // bool maintainLinkOrder,
  // bool intertiaFromURDF,

  // render mode
  std::string tmpRndrModeVal = "";
  if (io::readMember<std::string>(jsonConfig, "render_mode", tmpRndrModeVal)) {
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpRndrModeVal);
    auto found = attributes::AORenderModesMap.find(strToLookFor);
    if (found != attributes::AORenderModesMap.end()) {
      // only override JSON default value if new value is valid
      aoAttr->setRenderMode(strToLookFor);
    } else {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "'render_mode' Value in JSON : `" << tmpRndrModeVal
          << "` does not map to a valid "
             "attributes::AORenderModesMap value, so not setting/overriding "
             "Render mode value.";
    }
  }

  // check for user defined attributes
  this->parseUserDefinedJsonVals(aoAttr, jsonConfig);

}  // AOAttributesManager::setValsFromJSONDoc

attributes::ArticulatedObjectAttributes::ptr
AOAttributesManager::initNewObjectInternal(const std::string& attributesHandle,
                                           bool builtFromConfig) {
  // first build new attributes as a copy of dataset-specified default if
  // exists
  attributes::ArticulatedObjectAttributes::ptr newAttributes =
      this->constructFromDefault(attributesHandle);

  bool createNewAttributes = (nullptr == newAttributes);
  // if not then create new empty attributes
  if (createNewAttributes) {
    newAttributes =
        attributes::ArticulatedObjectAttributes::create(attributesHandle);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  if (!createNewAttributes) {
    // default exists and was used to create this attributes - investigate any
    // filename fields that may have %%USE_FILENAME%% directive specified in
    // the default attributes, and replace with appropriate derived value.

    // URDF source file handle
    setHandleFromDefaultTag(newAttributes, newAttributes->getURDFPath(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setURDFPath(newHandle);
                            });
    // Render asset handle
    setHandleFromDefaultTag(newAttributes,
                            newAttributes->getRenderAssetHandle(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setRenderAssetHandle(newHandle);
                            });
  }

  // set default URDF filename - only set handle defaults if attributesHandle
  // set default URDF filename - only set filename defaults if attributesHandle
  // is not a config file (which would never be a valid URDF filename).
  // Otherise, expect handles to be set when built from a config file.
  if (!builtFromConfig) {
    // If not built from json config but instead directly from URDF file, this
    // function was called from :
    //  - ManagedContainer::createDefaultObject
    //  -
    if (newAttributes->getURDFPath().empty()) {
      newAttributes->setURDFPath(attributesHandle);
    }
  }

  return newAttributes;
}  // AOAttributesManager::initNewObjectInternal

int AOAttributesManager::registerObjectFinalize(
    attributes::ArticulatedObjectAttributes::ptr AOAttributesTemplate,
    const std::string& AOAttributesHandle,
    bool) {
  // adds template to library, and returns either the ID of the existing
  // template referenced by AOAttributesHandle, or the next available ID
  // if not found.
  int AOTemplateID = this->addObjectToLibrary(std::move(AOAttributesTemplate),
                                              AOAttributesHandle);
  return AOTemplateID;
}  // AOAttributesManager::registerObjectFinalize

std::map<std::string, std::string>
AOAttributesManager::getArticulatedObjectModelFilenames() const {
  std::map<std::string, std::string> articulatedObjPaths;
  for (const auto& val : this->objectLibrary_) {
    auto attr = this->getObjectByHandle(val.first);
    auto key = attr->getSimplifiedHandle();
    auto urdf = attr->getURDFPath();
    articulatedObjPaths[key] = urdf;
  }
  return articulatedObjPaths;
}  // AOAttributesManager::getArticulatedObjectModelFilenames

}  // namespace managers
}  // namespace metadata
}  // namespace esp
