// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AOAttributesManager.h"
#include "AbstractAttributesManager.h"

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

void AOAttributesManager::setValsFromJSONDocInternal(
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
    if ((!Corrade::Utility::Path::exists(render_asset)) &&
        !render_asset.empty()) {
      render_asset =
          Cr::Utility::Path::join(aoAttr->getFileDirectory(), render_asset);
    }
    aoAttr->setRenderAssetHandle(render_asset);
  }

  // load the semantic id
  io::jsonIntoSetter<int>(jsonConfig, "semantic_id", [aoAttr](int semantic_id) {
    aoAttr->setSemanticId(semantic_id);
  });

  // load the uniform scaling
  io::jsonIntoSetter<double>(
      jsonConfig, "uniform_scale",
      [aoAttr](double scale) { aoAttr->setUniformScale(scale); });

  // load the mass scaling
  io::jsonIntoSetter<double>(jsonConfig, "mass_scale", [aoAttr](double scale) {
    aoAttr->setMassScale(scale);
  });

  // shader type
  this->setEnumStringFromJsonDoc(
      jsonConfig, "shader_type", "ShaderTypeNamesMap", false,
      attributes::ShaderTypeNamesMap,
      [aoAttr](const std::string& val) { aoAttr->setShaderType(val); });

  // render mode
  this->setEnumStringFromJsonDoc(
      jsonConfig, "render_mode", "AORenderModesMap", false,
      attributes::AORenderModesMap,
      [aoAttr](const std::string& val) { aoAttr->setRenderMode(val); });

  // base type
  this->setEnumStringFromJsonDoc(
      jsonConfig, "base_type", "AOBaseTypeMap", false,
      attributes::AOBaseTypeMap,
      [aoAttr](const std::string& val) { aoAttr->setBaseType(val); });

  // inertia source
  this->setEnumStringFromJsonDoc(
      jsonConfig, "inertia_source", "AOInertiaSourceMap", false,
      attributes::AOInertiaSourceMap,
      [aoAttr](const std::string& val) { aoAttr->setInertiaSource(val); });

  // link order
  this->setEnumStringFromJsonDoc(
      jsonConfig, "link_order", "AOLinkOrderMap", false,
      attributes::AOLinkOrderMap,
      [aoAttr](const std::string& val) { aoAttr->setLinkOrder(val); });

  // check for the existing of markersets
  this->parseMarkerSets(aoAttr, jsonConfig);
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
    setFilenameFromDefaultTag(newAttributes, newAttributes->getURDFPath(),
                              [newAttributes](const std::string& newHandle) {
                                newAttributes->setURDFPath(newHandle);
                              });
    // Render asset handle
    setFilenameFromDefaultTag(newAttributes,
                              newAttributes->getRenderAssetHandle(),
                              [newAttributes](const std::string& newHandle) {
                                newAttributes->setRenderAssetHandle(newHandle);
                              });
  } else {
    // need to set base/default urdf_filepath to a valid potential
    // filepath if a new attributes is being created here, to cover for older AO
    // configs that may not reference their owning URDF files. This will only
    // work for configs that reside in the same directory as their URDF
    // counterparts.

    const auto urdfFilePath = newAttributes->getURDFPath();
    // If .urdf extension not found then replace with string with extension.
    if (urdfFilePath.find(".urdf", urdfFilePath.length() - 5) ==
        std::string::npos) {
      newAttributes->setURDFPath(
          Cr::Utility::Path::splitExtension(
              Cr::Utility::Path::splitExtension(urdfFilePath).first())
              .first() +
          ".urdf");
    }
  }

  // set default URDF filename - only set filename defaults if
  // attributesHandle is not a config file (which would never be a valid URDF
  // filename). Otherise, expect handles to be set when built from a config
  // file.
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

core::managedContainers::ManagedObjectPreregistration
AOAttributesManager::preRegisterObjectFinalize(
    attributes::ArticulatedObjectAttributes::ptr AOAttributesTemplate,
    const std::string& AOAttributesHandle,
    bool) {
  // ArticulatedObjectsAttributes must have a valid URDF_file it relates to or
  // it should not be registered.
  const std::string urdfFilePath = AOAttributesTemplate->getURDFPath();
  const std::string urdfFullFilePath = AOAttributesTemplate->getURDFFullPath();
  if (urdfFilePath.empty()) {
    // URDF Filepath empty is bad
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "ArticulatedObjectAttributes template named `" << AOAttributesHandle
        << "` does not specify a valid URDF Filepath, so registration is "
           "aborted.";
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  } else if (!Cr::Utility::Path::exists(urdfFilePath) &&
             !Cr::Utility::Path::exists(urdfFullFilePath)) {
    // URDF File not found is bad
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "ArticulatedObjectAttributes template named `" << AOAttributesHandle
        << "` specifies the URDF Filepath `" << urdfFilePath << "` full path `"
        << urdfFullFilePath
        << "`, but this file cannot be found, so registration is aborted.";
    return core::managedContainers::ManagedObjectPreregistration::Failed;
  }
  const std::string renderAssetHandle =
      AOAttributesTemplate->getRenderAssetHandle();
  // Furthermore, if 'skin' is specified as render_mode and no skin is
  // specified or the specified skin cannot be found, the registration should
  // also fail and the template should not be registered.
  bool useSkinRenderMode =
      (AOAttributesTemplate->getRenderMode() ==
       attributes::ArticulatedObjectRenderMode::Skin) ||
      ((AOAttributesTemplate->getRenderMode() ==
        attributes::ArticulatedObjectRenderMode::Default) &&
       (!renderAssetHandle.empty()));
  if (useSkinRenderMode) {
    // if 'skin' render mode is specified as render mode or default and a skin
    // is specified

    const std::string renderAssetFullPath =
        AOAttributesTemplate->getRenderAssetFullPath();

    const std::string urdfSimpleName =
        Corrade::Utility::Path::splitExtension(
            Corrade::Utility::Path::splitExtension(
                Corrade::Utility::Path::split(urdfFilePath).second())
                .first())
            .first();
    if (renderAssetHandle.empty() && renderAssetFullPath.empty()) {
      // Empty is bad when 'skin' render mode is specified
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "ArticulatedObjectAttributes template named `"
          << AOAttributesHandle
          << "` specifies a render mode of `skin` for the AO created by `"
          << urdfSimpleName
          << "`, but no render asset was specifed in the configuration, so "
             "registration is aborted.";
      return core::managedContainers::ManagedObjectPreregistration::Failed;
    } else if (!Cr::Utility::Path::exists(renderAssetHandle) &&
               !Cr::Utility::Path::exists(renderAssetFullPath)) {
      // Skin render asset specified not found is bad when 'skin' render mode
      // is specified
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "ArticulatedObjectAttributes template named `"
          << AOAttributesHandle
          << "` specifies a render mode of `skin` for the AO created by `"
          << urdfSimpleName << "`, but the render asset specified, `"
          << renderAssetHandle << "` full path `" << renderAssetFullPath
          << "` cannot be found, so registration is aborted.";
      return core::managedContainers::ManagedObjectPreregistration::Failed;
    }
  }
  // filter all paths properly so that the handles don't have filepaths and the
  // accessors are hidden fields
  this->finalizeAttrPathsBeforeRegister(AOAttributesTemplate);
  // Clear dirty flag from when asset handles are changed
  AOAttributesTemplate->setFilePathsAreClean();

  return core::managedContainers::ManagedObjectPreregistration::Success;
}  // AOAttributesManager::preRegisterObjectFinalize

void AOAttributesManager::finalizeAttrPathsBeforeRegister(
    const attributes::ArticulatedObjectAttributes::ptr& AOAttributesTemplate)
    const {
  // URDF file this articulated object attributes references
  this->filterAttribsFilenames(
      AOAttributesTemplate, AOAttributesTemplate->getURDFPath(),
      AOAttributesTemplate->getURDFFullPath(),
      [AOAttributesTemplate](const std::string& urdfAsset) {
        AOAttributesTemplate->setURDFPath(urdfAsset);
      },
      [AOAttributesTemplate](const std::string& urdfAsset) {
        AOAttributesTemplate->setURDFFullPath(urdfAsset);
      });

  // Render asset filename filter out path and set internal reference to
  // full filepath
  this->filterAttribsFilenames(
      AOAttributesTemplate, AOAttributesTemplate->getRenderAssetHandle(),
      AOAttributesTemplate->getRenderAssetFullPath(),
      [AOAttributesTemplate](const std::string& renderAsset) {
        AOAttributesTemplate->setRenderAssetHandle(renderAsset);
      },
      [AOAttributesTemplate](const std::string& renderAsset) {
        AOAttributesTemplate->setRenderAssetFullPath(renderAsset);
      });

}  // AOAttributesManager::finalizeAttrPathsBeforeRegister

std::map<std::string, std::string>
AOAttributesManager::getArticulatedObjectModelFilenames() const {
  std::map<std::string, std::string> articulatedObjPaths;

  auto objIterPair = this->getObjectLibIterator();
  for (auto& objIter = objIterPair.first; objIter != objIterPair.second;
       ++objIter) {
    auto attr = this->getObjectByHandle(objIter->first);
    auto key = attr->getSimplifiedHandle();
    auto urdf = attr->getURDFFullPath();
    articulatedObjPaths[key] = urdf;
  }
  return articulatedObjPaths;
}  // AOAttributesManager::getArticulatedObjectModelFilenames

}  // namespace managers
}  // namespace metadata
}  // namespace esp
