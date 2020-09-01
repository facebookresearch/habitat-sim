// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Corrade/Containers/StaticArray.h>
#include <Corrade/Utility/String.h>

#include "AssetAttributesManager.h"
#include "AttributesManagerBase.h"

namespace esp {
namespace assets {

using attributes::AbstractPrimitiveAttributes;
using attributes::CapsulePrimitiveAttributes;
using attributes::ConePrimitiveAttributes;
using attributes::CubePrimitiveAttributes;
using attributes::CylinderPrimitiveAttributes;
using attributes::IcospherePrimitiveAttributes;
using attributes::UVSpherePrimitiveAttributes;
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

void AssetAttributesManager::buildCtorFuncPtrMaps() {
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
      &AssetAttributesManager::createAttributesCopy<CapsulePrimitiveAttributes>;
  this->copyConstructorMap_["ConePrimitiveAttributes"] =
      &AssetAttributesManager::createAttributesCopy<ConePrimitiveAttributes>;
  this->copyConstructorMap_["CubePrimitiveAttributes"] =
      &AssetAttributesManager::createAttributesCopy<CubePrimitiveAttributes>;
  this->copyConstructorMap_["CylinderPrimitiveAttributes"] =
      &AssetAttributesManager::createAttributesCopy<
          CylinderPrimitiveAttributes>;
  this->copyConstructorMap_["IcospherePrimitiveAttributes"] =
      &AssetAttributesManager::createAttributesCopy<
          IcospherePrimitiveAttributes>;
  this->copyConstructorMap_["UVSpherePrimitiveAttributes"] =
      &AssetAttributesManager::createAttributesCopy<
          UVSpherePrimitiveAttributes>;
  // no entry added for PrimObjTypes::END_PRIM_OBJ_TYPES
  this->undeletableTemplateNames_.clear();
  // build default AbstractPrimitiveAttributes objects
  for (const std::pair<const PrimObjTypes, const char*>& elem :
       PrimitiveNames3DMap) {
    if (elem.first == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      continue;
    }
    auto tmplt = createAttributesTemplate(elem.second, true);
    std::string tmpltHandle = tmplt->getHandle();
    defaultPrimAttributeHandles_[elem.second] = tmpltHandle;
    this->undeletableTemplateNames_.insert(tmpltHandle);
  }

  LOG(INFO) << "AssetAttributesManager::buildCtorFuncPtrMaps : Built default "
               "primitive asset templates : "
            << std::to_string(defaultPrimAttributeHandles_.size());
}  // AssetAttributesManager::buildMapOfPrimTypeConstructors

AbstractPrimitiveAttributes::ptr
AssetAttributesManager::createAttributesTemplate(
    const std::string& primClassName,
    bool registerTemplate) {
  auto primAssetAttributes = buildPrimAttributes(primClassName);
  if (nullptr == primAssetAttributes) {
    return primAssetAttributes;
  }
  LOG(INFO) << "Asset attributes (" << primClassName << ") created"
            << (registerTemplate ? " and registered." : ".");

  return this->postCreateRegister(primAssetAttributes, registerTemplate);
}  // AssetAttributesManager::createAttributesTemplate

int AssetAttributesManager::registerAttributesTemplateFinalize(
    AbstractPrimitiveAttributes::ptr primAttributesTemplate,
    const std::string&) {
  std::string primAttributesHandle = primAttributesTemplate->getHandle();
  // verify that attributes has been edited in a legal manner
  if (!primAttributesTemplate->isValidTemplate()) {
    LOG(ERROR) << "AssetAttributesManager::registerAttributesTemplateFinalize "
                  ": Primitive asset attributes template named"
               << primAttributesHandle
               << "is not configured properly for specified prmitive"
               << primAttributesTemplate->getPrimObjClassName()
               << ". Aborting.";
    return ID_UNDEFINED;
  }

  // return either the ID of the existing template referenced by
  // primAttributesHandle, or the next available ID if not found.
  int primTemplateID =
      this->addTemplateToLibrary(primAttributesTemplate, primAttributesHandle);
  return primTemplateID;
}  // AssetAttributesManager::registerAttributesTemplateFinalize

AbstractPrimitiveAttributes::ptr
AssetAttributesManager::loadAttributesFromJSONDoc(
    const std::string& filename,
    const io::JsonDocument& jsonDoc) {
  // find type of attributes - file name should contain handle
  const std::string primAttrHandle =
      Cr::Utility::Directory::splitExtension(
          Cr::Utility::Directory::filename(filename))
          .first;

  std::string primClassName =
      Cr::Utility::String::partition(primAttrHandle, '_')[0];
  // if not legal primitive asset attributes file name, have message and return
  // default sphere attributes.
  if (defaultPrimAttributeHandles_.count(primClassName) == 0) {
    LOG(ERROR) << "AssetAttributesManager::loadAttributesFromJSONDoc :Unknown "
                  "primitive class type : "
               << primClassName
               << " so returning default attributes for solid uvSphere.";
    return this->getTemplateCopyByHandle<Attrs::UVSpherePrimitiveAttributes>(
        defaultPrimAttributeHandles_.at("uvSphereSolid"));
  }

  // create attributes for this primitive
  auto primAssetAttributes = buildPrimAttributes(primClassName);
  if (nullptr == primAssetAttributes) {
    LOG(ERROR)
        << "AssetAttributesManager::loadAttributesFromJSONDoc : unable to "
           "create default primitive asset attributes from primClassName "
        << primClassName
        << " so returning default attributes for solid uvSphere.";
    return this->getTemplateCopyByHandle<Attrs::UVSpherePrimitiveAttributes>(
        defaultPrimAttributeHandles_.at("uvSphereSolid"));
  }

  // TODO support loading values from JSON docs

  return primAssetAttributes;
}  // AssetAttributesManager::loadAttributesFromJSONDoc

}  // namespace managers
}  // namespace assets
}  // namespace esp
