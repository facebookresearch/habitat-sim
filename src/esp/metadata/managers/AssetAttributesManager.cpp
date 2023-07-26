// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StaticArray.h>
#include <Corrade/Utility/String.h>

#include <utility>

#include "AssetAttributesManager.h"
#include "AttributesManagerBase.h"

namespace esp {
namespace metadata {

using attributes::AbstractPrimitiveAttributes;
using attributes::CapsulePrimitiveAttributes;
using attributes::ConePrimitiveAttributes;
using attributes::CubePrimitiveAttributes;
using attributes::CylinderPrimitiveAttributes;
using attributes::IcospherePrimitiveAttributes;
using attributes::UVSpherePrimitiveAttributes;
using core::managedContainers::ManagedObjectAccess;
namespace managers {

const std::map<PrimObjTypes, const char*>
    AssetAttributesManager::PrimitiveNames3DMap = {
        {PrimObjTypes::CAPSULE_SOLID, "capsule3DSolid"},
        {PrimObjTypes::CAPSULE_WF, "capsule3DWireframe"},
        {PrimObjTypes::CONE_SOLID, "coneSolid"},
        {PrimObjTypes::CONE_WF, "coneWireframe"},
        {PrimObjTypes::CUBE_SOLID, "cubeSolid"},
        {PrimObjTypes::CUBE_WF, "cubeWireframe"},
        {PrimObjTypes::CYLINDER_SOLID, "cylinderSolid"},
        {PrimObjTypes::CYLINDER_WF, "cylinderWireframe"},
        {PrimObjTypes::ICOSPHERE_SOLID, "icosphereSolid"},
        {PrimObjTypes::ICOSPHERE_WF, "icosphereWireframe"},
        {PrimObjTypes::UVSPHERE_SOLID, "uvSphereSolid"},
        {PrimObjTypes::UVSPHERE_WF, "uvSphereWireframe"},
        {PrimObjTypes::END_PRIM_OBJ_TYPES, "NONE DEFINED"}};

AssetAttributesManager::AssetAttributesManager()
    : AttributesManager<
          attributes::AbstractPrimitiveAttributes,
          ManagedObjectAccess::Copy>::AttributesManager("Primitive Asset",
                                                        "prim_config.json") {
  // function pointers to asset attributes constructors
  primTypeConstructorMap_["capsule3DSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          CapsulePrimitiveAttributes, false, PrimObjTypes::CAPSULE_SOLID>;
  primTypeConstructorMap_["capsule3DWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          CapsulePrimitiveAttributes, true, PrimObjTypes::CAPSULE_WF>;
  primTypeConstructorMap_["coneSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          ConePrimitiveAttributes, false, PrimObjTypes::CONE_SOLID>;
  primTypeConstructorMap_["coneWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          ConePrimitiveAttributes, true, PrimObjTypes::CONE_WF>;
  primTypeConstructorMap_["cubeSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          CubePrimitiveAttributes, false, PrimObjTypes::CUBE_SOLID>;
  primTypeConstructorMap_["cubeWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          CubePrimitiveAttributes, true, PrimObjTypes::CUBE_WF>;
  primTypeConstructorMap_["cylinderSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          CylinderPrimitiveAttributes, false, PrimObjTypes::CYLINDER_SOLID>;
  primTypeConstructorMap_["cylinderWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          CylinderPrimitiveAttributes, true, PrimObjTypes::CYLINDER_WF>;
  primTypeConstructorMap_["icosphereSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          IcospherePrimitiveAttributes, false, PrimObjTypes::ICOSPHERE_SOLID>;
  primTypeConstructorMap_["icosphereWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          IcospherePrimitiveAttributes, true, PrimObjTypes::ICOSPHERE_WF>;
  primTypeConstructorMap_["uvSphereSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          UVSpherePrimitiveAttributes, false, PrimObjTypes::UVSPHERE_SOLID>;
  primTypeConstructorMap_["uvSphereWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          UVSpherePrimitiveAttributes, true, PrimObjTypes::UVSPHERE_WF>;

  // function pointers to asset attributes copy constructors
  this->copyConstructorMap_["CapsulePrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<CapsulePrimitiveAttributes>;
  this->copyConstructorMap_["ConePrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<ConePrimitiveAttributes>;
  this->copyConstructorMap_["CubePrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<CubePrimitiveAttributes>;
  this->copyConstructorMap_["CylinderPrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<CylinderPrimitiveAttributes>;
  this->copyConstructorMap_["IcospherePrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<IcospherePrimitiveAttributes>;
  this->copyConstructorMap_["UVSpherePrimitiveAttributes"] =
      &AssetAttributesManager::createObjectCopy<UVSpherePrimitiveAttributes>;
  // no entry added for PrimObjTypes::END_PRIM_OBJ_TYPES
  this->undeletableObjectNames_.clear();
  // build default AbstractPrimitiveAttributes objects
  for (const std::pair<const PrimObjTypes, const char*>& elem :
       PrimitiveNames3DMap) {
    if (elem.first == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      continue;
    }
    auto tmplt = AssetAttributesManager::createObject(elem.second, true);
    std::string tmpltHandle = tmplt->getHandle();
    defaultPrimAttributeHandles_[elem.second] = tmpltHandle;
    this->undeletableObjectNames_.insert(std::move(tmpltHandle));
  }

  ESP_DEBUG() << "Built default primitive asset templates :"
              << defaultPrimAttributeHandles_.size();
}  // AssetAttributesManager::ctor

AbstractPrimitiveAttributes::ptr AssetAttributesManager::createObject(
    const std::string& primClassName,
    bool registerTemplate) {
  auto primAssetAttributes = this->createDefaultObject(primClassName, false);
  if (nullptr == primAssetAttributes) {
    return primAssetAttributes;
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Asset attributes (" << primClassName << ":"
      << primAssetAttributes->getHandle() << ") created"
      << (registerTemplate ? " and registered." : ".");

  return this->postCreateRegister(std::move(primAssetAttributes),
                                  registerTemplate);
}  // AssetAttributesManager::createObject

attributes::AbstractPrimitiveAttributes::ptr
AssetAttributesManager::createTemplateFromHandle(
    const std::string& templateHandle,
    bool registerTemplate) {
  // first determine what base type the attributes is - find first underscore.
  std::size_t nameEndLoc = templateHandle.find('_');
  if (nameEndLoc == std::string::npos) {
    // handle is of incorrect format
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Given template handle : `" << templateHandle
        << "` is not the correct format for a primitive, so "
           "createTemplateFromHandle aborting.";
    return nullptr;
  }
  std::string primClassName = templateHandle.substr(0, nameEndLoc);
  if (primTypeConstructorMap_.count(primClassName) == 0) {
    // handle does not have proper primitive tyep encoded
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Requested primitive type : `" << primClassName
        << "` from given template handle : `" << templateHandle
        << "` is not a valid Magnum::Primitives class, so "
           "createTemplateFromHandle aborting.";
    return nullptr;
  }
  // create but do not register template for this prim class, since it will be
  // modified based on config string
  auto primAssetAttributes = this->createObject(primClassName, false);
  // certain prims such as cubes do not have config settings
  if (templateHandle.length() > 0) {
    bool success = primAssetAttributes->parseStringIntoConfig(templateHandle);
    if (!success) {
      ESP_WARNING(Mn::Debug::Flag::NoSpace)
          << "Prim Asset Attributes : `" << primClassName
          << "` failed parsing config string : `" << templateHandle
          << "`. Providing `" << primClassName
          << "` template configured as closely as possible with requested "
             "values, named `"
          << primAssetAttributes->getHandle() << "`.";
    }
  }
  return this->postCreateRegister(std::move(primAssetAttributes),
                                  registerTemplate);
}  // AssetAttributesManager::createTemplateFromHandle

int AssetAttributesManager::registerObjectFinalize(
    AbstractPrimitiveAttributes::ptr primAttributesTemplate,
    const std::string&,
    bool) {
  std::string primAttributesHandle = primAttributesTemplate->getHandle();
  // verify that attributes has been edited in a legal manner
  if (!primAttributesTemplate->isValidTemplate()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Primitive asset attributes template named `" << primAttributesHandle
        << "` is not configured properly for specified prmitive `"
        << primAttributesTemplate->getPrimObjClassName()
        << "`, so Primitive asset attributes NOT registered.";
    return ID_UNDEFINED;
  }

  // return either the ID of the existing template referenced by
  // primAttributesHandle, or the next available ID if not found.
  int primTemplateID = this->addObjectToLibrary(
      std::move(primAttributesTemplate), primAttributesHandle);
  return primTemplateID;
}  // AssetAttributesManager::registerObjectFinalize

AbstractPrimitiveAttributes::ptr AssetAttributesManager::buildObjectFromJSONDoc(
    const std::string& filename,
    const io::JsonGenericValue& jsonDoc) {
  // find type of attributes - file name should contain handle
  const std::string primAttrHandle =
      Cr::Utility::Path::splitExtension(
          Cr::Utility::Path::split(filename).second())
          .first();

  std::string primClassName =
      Cr::Utility::String::partition(primAttrHandle, '_')[0];
  // if not legal primitive asset attributes file name, have message and
  // return default sphere attributes.
  if (defaultPrimAttributeHandles_.count(primClassName) == 0) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Unknown primitive class type : `" << primClassName
        << "` so returning default attributes for solid uvSphere.";
    return this->getObjectCopyByHandle<attributes::UVSpherePrimitiveAttributes>(
        defaultPrimAttributeHandles_.at("uvSphereSolid"));
  }

  // create attributes for the primitive described in the JSON file
  auto primAssetAttributes = this->initNewObjectInternal(primClassName, true);
  if (nullptr == primAssetAttributes) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Unable to create default primitive asset attributes from "
           "primClassName `"
        << primClassName
        << "` so returning default attributes for solid uvSphere.";
    return this->getObjectCopyByHandle<attributes::UVSpherePrimitiveAttributes>(
        defaultPrimAttributeHandles_.at("uvSphereSolid"));
  }
  this->setValsFromJSONDoc(primAssetAttributes, jsonDoc);
  return primAssetAttributes;
}  // AssetAttributesManager::buildObjectFromJSONDoc

void AssetAttributesManager::setValsFromJSONDoc(
    AttribsPtr attribs,
    const io::JsonGenericValue& jsonConfig) {
  // TODO support loading values from JSON docs
  // check for user defined attributes
  // this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // AssetAttributesManager::setValsFromJSONDoc

}  // namespace managers
}  // namespace metadata
}  // namespace esp
