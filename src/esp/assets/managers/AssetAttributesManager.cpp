// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "AssetAttributesManager.h"
#include "AttributesManagerBase.h"

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

namespace esp {
namespace assets {

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

void AssetAttributesManager::buildMapOfPrimTypeConstructors() {
  // function pointers to asset attributes constructors
  primTypeConstructorMap_["capsule3DSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CapsulePrimitiveAttributes, false,
          PrimObjTypes::CAPSULE_SOLID>;
  primTypeConstructorMap_["capsule3DWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CapsulePrimitiveAttributes, true, PrimObjTypes::CAPSULE_WF>;
  primTypeConstructorMap_["coneSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::ConePrimitiveAttributes, false, PrimObjTypes::CONE_SOLID>;
  primTypeConstructorMap_["coneWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::ConePrimitiveAttributes, true, PrimObjTypes::CONE_WF>;
  primTypeConstructorMap_["cubeSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CubePrimitiveAttributes, false, PrimObjTypes::CUBE_SOLID>;
  primTypeConstructorMap_["cubeWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CubePrimitiveAttributes, true, PrimObjTypes::CUBE_WF>;
  primTypeConstructorMap_["cylinderSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CylinderPrimitiveAttributes, false,
          PrimObjTypes::CYLINDER_SOLID>;
  primTypeConstructorMap_["cylinderWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::CylinderPrimitiveAttributes, true, PrimObjTypes::CYLINDER_WF>;
  primTypeConstructorMap_["icosphereSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::IcospherePrimitiveAttributes, false,
          PrimObjTypes::ICOSPHERE_SOLID>;
  primTypeConstructorMap_["icosphereWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::IcospherePrimitiveAttributes, true,
          PrimObjTypes::ICOSPHERE_WF>;
  primTypeConstructorMap_["uvSphereSolid"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::UVSpherePrimitiveAttributes, false,
          PrimObjTypes::UVSPHERE_SOLID>;
  primTypeConstructorMap_["uvSphereWireframe"] =
      &AssetAttributesManager::createPrimAttributes<
          assets::UVSpherePrimitiveAttributes, true, PrimObjTypes::UVSPHERE_WF>;

  // function pointers to asset attributes copy constructors
  primTypeCopyConstructorMap_["capsule3DSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CapsulePrimitiveAttributes>;
  primTypeCopyConstructorMap_["capsule3DWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CapsulePrimitiveAttributes>;
  primTypeCopyConstructorMap_["coneSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::ConePrimitiveAttributes>;
  primTypeCopyConstructorMap_["coneWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::ConePrimitiveAttributes>;
  primTypeCopyConstructorMap_["cubeSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CubePrimitiveAttributes>;
  primTypeCopyConstructorMap_["cubeWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CubePrimitiveAttributes>;
  primTypeCopyConstructorMap_["cylinderSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CylinderPrimitiveAttributes>;
  primTypeCopyConstructorMap_["cylinderWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::CylinderPrimitiveAttributes>;
  primTypeCopyConstructorMap_["icosphereSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::IcospherePrimitiveAttributes>;
  primTypeCopyConstructorMap_["icosphereWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::IcospherePrimitiveAttributes>;
  primTypeCopyConstructorMap_["uvSphereSolid"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::UVSpherePrimitiveAttributes>;
  primTypeCopyConstructorMap_["uvSphereWireframe"] =
      &AssetAttributesManager::createPrimAttributesCopy<
          assets::UVSpherePrimitiveAttributes>;
  // no entry added for PrimObjTypes::END_PRIM_OBJ_TYPES

  // build default AbstractPrimitiveAttributes objects
  for (const std::pair<PrimObjTypes, const char*>& elem : PrimitiveNames3DMap) {
    if (elem.first == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      continue;
    }
    createAttributesTemplate(elem.second, true);
  }
}  // buildMapOfPrimTypeConstructors

std::shared_ptr<AbstractPrimitiveAttributes>
AssetAttributesManager::createAttributesTemplate(
    const std::string& primClassName,
    bool registerTemplate) {
  auto primAssetAttributes = buildPrimAttributes(primClassName);

  if (registerTemplate) {
    registerAttributesTemplate(primAssetAttributes, "");
  }
  return primAssetAttributes;
}  // AssetAttributesManager::createAttributesTemplate

std::shared_ptr<AbstractPrimitiveAttributes>
AssetAttributesManager::createAttributesTemplate(PrimObjTypes primObjType,
                                                 bool registerTemplate) {
  auto primAssetAttributes = buildPrimAttributes(primObjType);

  if (registerTemplate) {
    registerAttributesTemplate(primAssetAttributes, "");
  }
  return primAssetAttributes;
}  // AssetAttributesManager::createAttributesTemplate

int AssetAttributesManager::registerAttributesTemplate(
    std::shared_ptr<AbstractPrimitiveAttributes> primAttributesTemplate,
    const std::string&) {
  std::string primAttributesHandle = primAttributesTemplate->getOriginHandle();
  // verify that attributes has been edited in a legal manner
  CORRADE_ASSERT(
      primAttributesTemplate->isValidTemplate(),
      "AssetAttributesManager::registerAttributesTemplate : Primitive "
      "asset attributes template named"
          << primAttributesHandle
          << "is not configured properly for specified prmitive"
          << primAttributesTemplate->getPrimObjClassName() << ". Aborting.",
      ID_UNDEFINED);

  // return either the ID of the existing template referenced by
  // primAttributesHandle, or the next available ID if not found.
  int primTemplateID =
      addTemplateToLibrary(primAttributesTemplate, primAttributesHandle);
  return primTemplateID;
}  // AssetAttributesManager::registerAttributesTemplate

std::shared_ptr<AbstractPrimitiveAttributes>
AssetAttributesManager::getAttributesTemplateCopy(
    const std::string& primTemplateHandle) {
  CORRADE_ASSERT((templateLibrary_.count(primTemplateHandle) > 0),
                 "AssetAttributesManager::getAttributesCopy : Unknown "
                 "template handle :"
                     << primTemplateHandle << ". Aborting",
                 nullptr);
  // need to return copy so that user mods do not modify existing
  // asset template
  auto orig = templateLibrary_.at(primTemplateHandle);
  // build a copy of existing template
  return copyPrimAttributes(orig);
}  // AssetAttributesManager::getPrimAssetAttributesCopy

}  // namespace managers
}  // namespace assets
}  // namespace esp