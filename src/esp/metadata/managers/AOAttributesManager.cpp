// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AOAttributesManager.h"
#include "AttributesManagerBase.h"

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
  // load the urdf filepath
  std::string urdfFName = "";

  if (io::readMember<std::string>(jsonConfig, "urdf_filepath", urdfFName)) {
    // If specified urdfFName is not found directly, prefix it with file
    // directory
    if (!Corrade::Utility::Path::exists(urdfFName)) {
      urdfFName =
          Cr::Utility::Path::join(aoAttr->getFileDirectory(), urdfFName);
    }
    aoAttr->setURDFPath(urdfFName);
  }

  // load the render asset handle
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "render_asset", [aoAttr](const std::string& render_asset) {
        aoAttr->setRenderAssetHandle(render_asset);
      });

  // load the semantic id
  io::jsonIntoSetter<int>(jsonConfig, "semantic_id", [aoAttr](int semantic_id) {
    aoAttr->setSemanticId(semantic_id);
  });

  // load whether to render using the AO primitives, regardless of whether a
  // render asset is present or not.
  io::jsonIntoSetter<bool>(
      jsonConfig, "debug_render_primitives",
      [aoAttr](bool debug_render_primitives) {
        aoAttr->setDebugRenderPrimitives(debug_render_primitives);
      });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(aoAttr, jsonConfig);

}  // AOAttributesManager::createFileBasedAttributesTemplate

attributes::ArticulatedObjectAttributes::ptr
AOAttributesManager::initNewObjectInternal(const std::string& attributesHandle,
                                           bool builtFromConfig) {
  // first build new attributes as a copy of dataset-specified default if exists
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
    // filename fields that may have %%USE_FILENAME%% directive specified in the
    // default attributes, and replace with appropriate derived value.
    // Render asset handle
    setHandleFromDefaultTag(newAttributes,
                            newAttributes->getRenderAssetHandle(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setRenderAssetHandle(newHandle);
                            });
  }

  // set default URDF filename - only set handle defaults if attributesHandle is
  // not a config file (which would never be a valid URDF filename).  Otherise,
  // expect handles to be set when config is read.
  if (!builtFromConfig) {
    // If not built from json config, this function was called from :
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