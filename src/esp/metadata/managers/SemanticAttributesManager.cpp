// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SemanticAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace esp {

namespace metadata {

using attributes::SemanticAttributes;
using attributes::SemanticRegionAttributes;
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
    setHandleFromDefaultTag(
        newAttributes, newAttributes->getSemanticDescriptorFilename(),
        [newAttributes](const std::string& newHandle) {
          newAttributes->setSemanticDescriptorFilename(newHandle);
        });
    // Semantic Scene asset handle
    setHandleFromDefaultTag(newAttributes,
                            newAttributes->getSemanticAssetHandle(),
                            [newAttributes](const std::string& newHandle) {
                              newAttributes->setSemanticAssetHandle(newHandle);
                            });
  }

  return newAttributes;
}  // SemanticAttributesManager::initNewObjectInternal

void SemanticAttributesManager::setSemanticRegionAttributesFromJson(
    const SemanticRegionAttributes::ptr& instanceAttrs,
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
  // Polyloop points

  io::JsonGenericValue::ConstMemberIterator polyLoopJSONIter =
      jCell.FindMember("poly_loop");
  if (polyLoopJSONIter != jCell.MemberEnd()) {
    if (polyLoopJSONIter->value.IsArray()) {
      std::vector<Mn::Vector3> polyLoop;
      // read values into vector
      io::readMember<Mn::Vector3>(jCell, "poly_loop", polyLoop);
      instanceAttrs->setPolyLoop(polyLoop);

    } else {
      ESP_WARNING() << ": Unknown format for "
                       "poly_loop specified for region instance"
                    << instanceAttrs->getHandle()
                    << "in Semantic Config File, so no values are set.";
    }
  }

  // check for user defined attributes
  this->parseUserDefinedJsonVals(instanceAttrs, jCell);

}  // SemanticAttributesManager::setSemanticRegionAttributesFromJson

SemanticRegionAttributes::ptr
SemanticAttributesManager::createRegionAttributesFromJSON(
    const io::JsonGenericValue& jCell) {
  SemanticRegionAttributes::ptr regionAttrs = createEmptyRegionAttributes("");
  // populate attributes
  this->setSemanticRegionAttributesFromJson(regionAttrs, jCell);
  return regionAttrs;
}  // SemanticAttributesManager::createRegionAttributesFromJSON

void SemanticAttributesManager::setValsFromJSONDoc(
    SemanticAttributes::ptr semanticAttribs,
    const io::JsonGenericValue& jsonConfig) {
  const std::string attribsDispName = semanticAttribs->getSimplifiedHandle();
  // ROOT LEVEL SEMANTICS TODO

  // Set the semantic descriptor filename
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "semantic_descriptor_filename",
      [semanticAttribs](const std::string& _semanticDescriptorFilename) {
        semanticAttribs->setSemanticDescriptorFilename(
            _semanticDescriptorFilename);
      });

  // Set the semantic mesh asset filename
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "semantic_asset",
      [semanticAttribs](const std::string& _semanticAsset) {
        semanticAttribs->setSemanticAssetHandle(_semanticAsset);
      });

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
              createRegionAttributesFromJSON(regionCell));
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

}  // namespace managers
}  // namespace metadata
}  // namespace esp
