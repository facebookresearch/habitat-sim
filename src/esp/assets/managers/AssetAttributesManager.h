// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"
namespace esp {
namespace assets {
/**
 * @brief The kinds of primitive modelled objects supported.  Paired with
 * Magnum::Primitive namespace objects
 */
enum class PrimObjTypes : uint32_t {
  /**
   * Primitive object corresponding to Magnum::Primitives::capsule3DSolid
   */
  CAPSULE_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::capsule3DWireframe
   */
  CAPSULE_WF,
  /**
   * Primitive object corresponding to Magnum::Primitives::coneSolid
   */
  CONE_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::coneWireframe
   */
  CONE_WF,
  /**
   * Primitive object corresponding to Magnum::Primitives::cubeSolid
   */
  CUBE_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::cubeWireframe
   */
  CUBE_WF,
  /**
   * Primitive object corresponding to Magnum::Primitives::cylinderSolid
   */
  CYLINDER_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::cylinderWireframe
   */
  CYLINDER_WF,
  /**
   * Primitive object corresponding to Magnum::Primitives::icosphereSolid
   */
  ICOSPHERE_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::icosphereWireframe
   */
  ICOSPHERE_WF,
  /**
   * Primitive object corresponding to Magnum::Primitives::uvSphereSolid
   */
  UVSPHERE_SOLID,
  /**
   * Primitive object corresponding to Magnum::Primitives::uvSphereWireframe
   */
  UVSPHERE_WF,
  /**
   * marker for no more primitive objects - add any new objects above this entry
   */
  END_PRIM_OBJ_TYPES
};
namespace managers {

class AssetAttributesManager
    : public AttributesManager<AbstractPrimitiveAttributes> {
 public:
  /**
   * @brief Constant Map holding names of all Magnum 3D primitive classes
   * supported, keyed by @ref PrimObjTypes enum entry.  Note final entry is not
   * a valid primitive.
   */
  static const std::map<PrimObjTypes, const char*> PrimitiveNames3DMap;
  AssetAttributesManager()
      : AttributesManager<AbstractPrimitiveAttributes>::AttributesManager() {
    buildMapOfPrimTypeConstructors();
  }

  /**
   * @brief Creates an instance of a primtive asset attributes template
   * described by passed string.
   * For primitive assets this is the Magnum primitive class name
   *
   * @param primClassName a string descriptor of the primitive asset
   * template to be created, corresponding to the Magnum Primitive class
   * name.
   * @param registerTemplate whether to add this template to the library or
   * not. If the user is going to edit this template, this should be false.
   * @return a reference to the desired template.
   */

  std::shared_ptr<AbstractPrimitiveAttributes> createAttributesTemplate(
      const std::string& primClassName,
      bool registerTemplate = true);

  /**
   * @brief Creates an instance of a primtive asset attributes template
   * described by passed enum value. For primitive assets this mapes to the
   * Magnum primitive class name
   *
   * @param primObjType an enum value denoting the class of the primitive to
   * instantiate
   * @param registerTemplate whether to add this template to the library or
   * not. If the user is going to edit this template, this should be false.
   * @return a reference to the desired template.
   */
  std::shared_ptr<AbstractPrimitiveAttributes> createAttributesTemplate(
      PrimObjTypes primObjType,
      bool registerTemplate = true);

  /**
   * @brief Add an @ref AbstractPrimitiveAttributes object to the @ref
   * templateLibrary_.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in the
   * @ref templateLibrary_.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplate(
      std::shared_ptr<AbstractPrimitiveAttributes> attributesTemplate,
      const std::string& attributesTemplateHandle);

  /**
   * @brief Get list of primitive asset template handles used as keys in @ref
   * primitiveAssetTemplateLibrary_ related to passed primitive descriptor enum.
   *
   * @param primType Enum
   * @param contains whether to search for keys containing, or not containing,
   * @ref subStr
   * @return list containing 0 or more string keys corresponding to templates in
   * @ref primitiveAssetTemplateLibrary_ that contain the passed substring
   */
  std::vector<std::string> getTemplateHandlesByPrimType(
      PrimObjTypes primType,
      bool contains = true) const {
    CORRADE_ASSERT(primType != PrimObjTypes::END_PRIM_OBJ_TYPES,
                   "AssetAttributesManager::getTemplateHandlesByPrimType : "
                   "Illegal primtitive type "
                   "name PrimObjTypes::END_PRIM_OBJ_TYPES.  Aborting.",
                   {});
    std::string subStr = PrimitiveNames3DMap.at(primType);
    return getTemplateHandlesBySubStringPerType(templateLibKeyByID_, subStr,
                                                contains);
  }  // getTemplateHandlesByPrimType

  /**
   * @brief Return a copy of the primitive asset attributes object specified
   * by passed handle.  This is the version that should be accessed by the
   * user.
   * @param primTemplateHandle the string key of the attributes desired -
   * this key will be synthesized based on attributes values.
   * @return a copy of the desired primitive attributes, or nullptr if does
   * not exist
   */

  std::shared_ptr<AbstractPrimitiveAttributes> getAttributesTemplateCopy(
      const std::string& primTemplateHandle);

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed class name
   */
  std::shared_ptr<AbstractPrimitiveAttributes> copyPrimAttributes(
      const std::shared_ptr<AbstractPrimitiveAttributes>& origAttr) {
    std::string primTypeName = origAttr->getPrimObjClassName();
    return (*this.*primTypeCopyConstructorMap_[primTypeName])(origAttr);
  }  // buildPrimAttributes

 protected:
  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed class name
   */
  std::shared_ptr<AbstractPrimitiveAttributes> buildPrimAttributes(
      const std::string& primTypeName) {
    CORRADE_ASSERT(
        primTypeConstructorMap_.count(primTypeName) > 0,
        "AssetAttributesManager::buildPrimAttributes : No primivite of type"
            << primTypeName << "exists.  Aborting.",
        nullptr);
    return (*this.*primTypeConstructorMap_[primTypeName])();
  }  // buildPrimAttributes

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed enum value, which maps to class name via @ref
   * PrimitiveNames3DMap
   */
  std::shared_ptr<AbstractPrimitiveAttributes> buildPrimAttributes(
      PrimObjTypes primType) {
    CORRADE_ASSERT(
        primType != PrimObjTypes::END_PRIM_OBJ_TYPES,
        "AssetAttributesManager::buildPrimAttributes : Illegal primtitive type "
        "name PrimObjTypes::END_PRIM_OBJ_TYPES.  Aborting.",
        nullptr);
    return (*this.*primTypeConstructorMap_[PrimitiveNames3DMap.at(primType)])();
  }  // buildPrimAttributes

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed enum value, which maps to class name via @ref
   * PrimitiveNames3DMap
   */
  std::shared_ptr<AbstractPrimitiveAttributes> buildPrimAttributes(
      int primTypeVal) {
    CORRADE_ASSERT(
        (primTypeVal >= 0) &&
            (primTypeVal < static_cast<int>(PrimObjTypes::END_PRIM_OBJ_TYPES)),
        "AssetAttributesManager::buildPrimAttributes : Unknown PrimObjTypes "
        "value requested :"
            << primTypeVal << ". Aborting",
        nullptr);
    return (*this.*primTypeConstructorMap_[PrimitiveNames3DMap.at(
                       static_cast<PrimObjTypes>(primTypeVal))])();
  }  // buildPrimAttributes

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   */
  template <typename T, bool isWireFrame, PrimObjTypes primitiveType>
  std::shared_ptr<AbstractPrimitiveAttributes> createPrimAttributes() {
    CORRADE_ASSERT(
        (primitiveType != PrimObjTypes::END_PRIM_OBJ_TYPES),
        "AssetAttributeManager::createPrimAttributes : Cannot instantiate "
        "AbstractPrimitiveAttributes object for "
        "PrimObjTypes::END_PRIM_OBJ_TYPES. "
        "Aborting.",
        nullptr);
    int idx = static_cast<int>(primitiveType);
    return T::create(isWireFrame, idx, PrimitiveNames3DMap.at(primitiveType));
  }

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   * @param orig original object of type t being copied
   */
  template <typename T>
  std::shared_ptr<AbstractPrimitiveAttributes> createPrimAttributesCopy(
      const std::shared_ptr<AbstractPrimitiveAttributes>& orig) {
    return T::create(*(static_cast<T*>(orig.get())));
  }

 protected:
  /**
   * @brief This function will assign the appropriately configured function
   * pointers for @ref createPrimAttributes calls for each type of
   * supported primitive to the @ref primTypeConstructorMap, keyed by type of
   * primtive
   */
  void buildMapOfPrimTypeConstructors();

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createPrimAttributes() and @ref createPrimAttributesCopy keyed by
   * string names of classes being instanced, as defined in @ref
   * PrimNames3D
   */
  typedef std::map<std::string,
                   std::shared_ptr<esp::assets::AbstractPrimitiveAttributes> (
                       AssetAttributesManager::*)()>
      Map_Of_PrimTypeCtors;

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createPrimAttributes() and @ref createPrimAttributesCopy keyed by
   * string names of classes being instanced, as defined in @ref
   * PrimNames3D
   */
  typedef std::map<std::string,
                   std::shared_ptr<esp::assets::AbstractPrimitiveAttributes> (
                       AssetAttributesManager::*)(
                       const std::shared_ptr<AbstractPrimitiveAttributes>&)>
      Map_Of_PrimTypeCopyCtors;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_PrimTypeCtors primTypeConstructorMap_;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_PrimTypeCopyCtors primTypeCopyConstructorMap_;

 public:
  ESP_SMART_POINTERS(AssetAttributesManager)

};  // AssetAttributesManager
}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_