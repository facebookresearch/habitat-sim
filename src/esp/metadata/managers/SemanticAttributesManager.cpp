// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticAttributesManager.h"
#include "AbstractAttributesManager.h"

#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace esp {

namespace metadata {

using attributes::SemanticAttributes;
using attributes::SemanticVolumeAttributes;
namespace managers {

SemanticAttributes::ptr SemanticAttributesManager::createObject(
    const std::string& semanticConfigFilename,
    bool registerTemplate) {
  std::string msg;
  SemanticAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      semanticConfigFilename, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG() << msg << "Semantic Attributes created"
                << (registerTemplate ? "and registered." : ".");
  }
  return attrs;
}  // SemanticAttributesManager::createObject

SemanticAttributes::ptr SemanticAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool builtFromConfig) {
  // first try to build new attributes as a copy of dataset-specified default if
  // exists
  SemanticAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);

  bool createNewAttributes = (nullptr == newAttributes);
  // if not then create new empty attributes
  if (createNewAttributes) {
    newAttributes = SemanticAttributes::create(handleName);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  if (!createNewAttributes) {
    // default exists and was used to create this attributes - investigate any
    // filename fields that may have %%USE_FILENAME%% directive specified in
    // the default attributes, and replace with appropriate derived value.

    // Semantic Scene Descriptor text filehandle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getSemanticDescriptorFilename(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setSemanticDescriptorFilename(newHandle);
        });
    // Semantic Scene asset handle
    setFilenameFromDefaultTag(
        newAttributes, newAttributes->getSemanticAssetHandle(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setSemanticAssetHandle(newHandle);
        });
  }
  if (builtFromConfig) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Semantic Attributes :`" << handleName << "` built from a config.";
  } else {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "Semantic Attributes :`" << handleName
        << "` not built from a config/synthesized.";
  }

  return newAttributes;
}  // SemanticAttributesManager::initNewObjectInternal

void SemanticAttributesManager::setSemanticVolumeAttributesFromJson(
    const SemanticVolumeAttributes::ptr& instanceAttrs,
    const io::JsonGenericValue& jCell) {
  // unique name for region
  io::jsonIntoConstSetter<std::string>(
      jCell, "name", [instanceAttrs](const std::string& region_name) {
        instanceAttrs->setHandle(region_name);
      });

  // semantic label for region
  io::jsonIntoConstSetter<std::string>(
      jCell, "label", [instanceAttrs](const std::string& label) {
        instanceAttrs->setLabel(label);
      });

  // floor height
  io::jsonIntoSetter<double>(jCell, "floor_height",
                             [instanceAttrs](double floor_height) {
                               instanceAttrs->setFloorHeight(floor_height);
                             });

  // extrusion height above floor
  io::jsonIntoSetter<double>(
      jCell, "extrusion_height", [instanceAttrs](double extrusion_height) {
        instanceAttrs->setExtrusionHeight(extrusion_height);
      });

  // min bounds of region
  io::jsonIntoConstSetter<Mn::Vector3>(
      jCell, "min_bounds", [instanceAttrs](const Mn::Vector3& minBounds) {
        instanceAttrs->setMinBounds(minBounds);
      });

  // max bounds of region
  io::jsonIntoConstSetter<Mn::Vector3>(
      jCell, "max_bounds", [instanceAttrs](const Mn::Vector3& maxBounds) {
        instanceAttrs->setMaxBounds(maxBounds);
      });

  //////////////////////
  // Polyloop points and user defined

  // parse poly loop points, whether defined as an array or a key-value store in
  // JSON
  this->parseSubconfigJsonVals("poly_loop", instanceAttrs, jCell);

  // check for user defined attributes
  this->parseUserDefinedJsonVals(instanceAttrs, jCell);

}  // SemanticAttributesManager::setSemanticVolumeAttributesFromJson

SemanticVolumeAttributes::ptr
SemanticAttributesManager::createRegionAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SemanticVolumeAttributes::ptr regionAttrs = createEmptyRegionAttributes("");
  // populate attributes
  this->setSemanticVolumeAttributesFromJson(regionAttrs, jCell);
  return regionAttrs;
}  // SemanticAttributesManager::createRegionAttributesFromJSON

void SemanticAttributesManager::setValsFromJSONDoc(
    SemanticAttributes::ptr semanticAttribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string attribsDispName = semanticAttribs->getSimplifiedHandle();
  // ROOT LEVEL SEMANTICS TODO
  // directory location where semantic attributes files are found
  std::string semanticLocFileDir = semanticAttribs->getFileDirectory();

  // load semantic asset specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "semantic_up", [semanticAttribs](const Magnum::Vector3& up) {
        semanticAttribs->setSemanticOrientUp(up);
      });

  // load semantic asset specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "semantic_front",
      [semanticAttribs](const Magnum::Vector3& front) {
        semanticAttribs->setSemanticOrientFront(front);
      });

  // load whether the semantic asset described has semantically annotated
  // textures
  io::jsonIntoSetter<bool>(
      jsonConfig, "has_semantic_textures",
      [semanticAttribs](bool has_semantic_textures) {
        semanticAttribs->setHasSemanticTextures(has_semantic_textures);
      });

  // Set the semantic asset filename
  std::string semanticAsset = "";
  if (io::readMember<std::string>(jsonConfig, "semantic_asset",
                                  semanticAsset)) {
    // if "semantic mesh" is specified in source json to non-empty value, set
    // value (override default).
    // semantic asset filename might already be fully qualified; if
    // not, might just be file name
    if (!Corrade::Utility::Path::exists(semanticAsset)) {
      semanticAsset =
          Cr::Utility::Path::join(semanticLocFileDir, semanticAsset);
    }
    semanticAttribs->setSemanticAssetHandle(semanticAsset);
  }

  // Set the semantic descriptor filename
  std::string semanticSceneDescriptor = "";
  if (io::readMember<std::string>(jsonConfig, "semantic_descriptor_filename",
                                  semanticSceneDescriptor)) {
    // if "semantic_descriptor_filename" is specified in source json, set value
    // (override default).
    // semanticSceneDescriptor filename might already be fully qualified; if
    // not, might just be file name
    if (!Corrade::Utility::Path::exists(semanticSceneDescriptor)) {
      semanticSceneDescriptor =
          Cr::Utility::Path::join(semanticLocFileDir, semanticSceneDescriptor);
    }
    semanticAttribs->setSemanticDescriptorFilename(semanticSceneDescriptor);
  }
  // TODO : drive by diagnostics when implemented
  bool validateUnique = false;
  // Check for region instances existence
  io::JsonGenericValue::ConstMemberIterator regionJSONIter =
      jsonConfig.FindMember("region_annotations");
  if (regionJSONIter != jsonConfig.MemberEnd()) {
    // region_annotations tag exists
    if (regionJSONIter->value.IsArray()) {
      const auto& regionArray = regionJSONIter->value;
      for (rapidjson::SizeType i = 0; i < regionArray.Size(); ++i) {
        const auto& regionCell = regionArray[i];
        if (regionCell.IsObject()) {
          semanticAttribs->addRegionInstanceAttrs(
              createRegionAttributesFromJSON(regionCell), validateUnique);
        } else {
          ESP_WARNING(Mn::Debug::Flag::NoSpace)
              << "Region instance issue in Semantic Configuration `"
              << attribsDispName << "` at idx : " << i
              << " : JSON cell within `region_annotations` array is not a "
                 "valid JSON object, so skipping entry.";
        }
      }
    } else {
      // region_annotations tag exists but is not an array. should warn (perhaps
      // error?)
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Region instance issue in Semantic Configuration `"
          << attribsDispName
          << "`: JSON cell `region_annotations` is not a valid JSON "
             "array, so no object instances loaded.";
    }
  } else {
    // No region_annotations tag exists in scene instance. Not necessarily a bad
    // thing - not all semantic descriptions must include region descriptions.
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "No Regions were specified in Semantic Configuration `"
        << attribsDispName
        << "`: JSON cell with tag `region_annotations` does not exist in "
           "Semantic Configuration file.";
  }

  // check for user defined attributes
  this->parseUserDefinedJsonVals(semanticAttribs, jsonConfig);

}  // SemanticAttributesManager::setValsFromJSONDoc

core::managedContainers::ManagedObjectPreregistration
SemanticAttributesManager::preRegisterObjectFinalize(
    attributes::SemanticAttributes::ptr semanticAttributes,
    CORRADE_UNUSED const std::string& semanticAttrHandle,
    CORRADE_UNUSED bool forceRegistration) {
  // filter all paths properly so that the handles don't have filepaths and the
  // accessors are hidden fields
  this->finalizeAttrPathsBeforeRegister(semanticAttributes);

  return core::managedContainers::ManagedObjectPreregistration::Success;
}  // SemanticAttributesManager::preRegisterObjectFinalize

void SemanticAttributesManager::finalizeAttrPathsBeforeRegister(
    const attributes::SemanticAttributes::ptr& semanticAttributes) const {
  // filter filepaths of full path qualifiers
  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "BEFORE : Semantic Attr `" << semanticAttributes->getHandle()
      << "`| semantic asset fn `"
      << semanticAttributes->getSemanticAssetHandle()
      << "`| semantic descriptor fn `"
      << semanticAttributes->getSemanticDescriptorFilename()
      << "`| file directory `" << semanticAttributes->getFileDirectory() << "`";
  // Semantic asset filename
  this->filterAttribsFilenames(
      semanticAttributes, semanticAttributes->getSemanticAssetHandle(),
      semanticAttributes->getSemanticAssetFullPath(),
      [semanticAttributes](const std::string& semanticAsset) {
        semanticAttributes->setSemanticAssetHandle(semanticAsset);
      },
      [semanticAttributes](const std::string& semanticAsset) {
        semanticAttributes->setSemanticAssetFullPath(semanticAsset);
      });
  // Semantic descriptor filename
  this->filterAttribsFilenames(
      semanticAttributes, semanticAttributes->getSemanticDescriptorFilename(),
      semanticAttributes->getSemanticDescriptorFullPath(),
      [semanticAttributes](const std::string& semanticAsset) {
        semanticAttributes->setSemanticDescriptorFilename(semanticAsset);
      },
      [semanticAttributes](const std::string& semanticAsset) {
        semanticAttributes->setSemanticDescriptorFullPath(semanticAsset);
      });

  ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
      << "AFTER : Semantic Attr `" << semanticAttributes->getHandle()
      << "`| semantic asset fn `"
      << semanticAttributes->getSemanticAssetHandle()
      << "`| semantic descriptor fn `"
      << semanticAttributes->getSemanticDescriptorFilename()
      << "`| file directory `" << semanticAttributes->getFileDirectory() << "`";

}  // SemanticAttributesManager::finalizeAttrPathsBeforeRegister

}  // namespace managers
}  // namespace metadata
}  // namespace esp
