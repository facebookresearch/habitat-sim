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
    : public AttributesManager<AbstractPrimitiveAttributes::ptr> {
 public:
  /**
   * @brief Constant Map holding names of all Magnum 3D primitive classes
   * supported, keyed by @ref PrimObjTypes enum entry.  Note final entry is not
   * a valid primitive.
   */
  static const std::map<PrimObjTypes, const char*> PrimitiveNames3DMap;
  AssetAttributesManager()
      : AttributesManager<
            AbstractPrimitiveAttributes::ptr>::AttributesManager() {
    buildCtorFuncPtrMaps();
  }  // AssetAttributesManager::ctor

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

  const AbstractPrimitiveAttributes::ptr createAttributesTemplate(
      const std::string& primClassName,
      bool registerTemplate = true) {
    auto primAssetAttributes = buildPrimAttributes(primClassName);
    if (registerTemplate) {
      registerAttributesTemplate(primAssetAttributes, "");
    }
    return primAssetAttributes;
  }  // AssetAttributesManager::createAttributesTemplate

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
  const AbstractPrimitiveAttributes::ptr createAttributesTemplate(
      PrimObjTypes primObjType,
      bool registerTemplate = true) {
    auto primAssetAttributes = buildPrimAttributes(primObjType);
    if (registerTemplate) {
      registerAttributesTemplate(primAssetAttributes, "");
    }
    return primAssetAttributes;
  }  // AssetAttributesManager::createAttributesTemplate

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
      const AbstractPrimitiveAttributes::ptr attributesTemplate,
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
    if (primType == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      LOG(ERROR) << "AssetAttributesManager::getTemplateHandlesByPrimType : "
                    "Illegal primtitive type "
                    "name PrimObjTypes::END_PRIM_OBJ_TYPES.  Aborting.";
      return {};
    }
    std::string subStr = PrimitiveNames3DMap.at(primType);
    return this->getTemplateHandlesBySubStringPerType(this->templateLibKeyByID_,
                                                      subStr, contains);
  }  // AssetAttributeManager::getTemplateHandlesByPrimType

 protected:
  /**
   * @brief Whether template described by passed handle is read only, or can be
   * deleted.  Default primitive asset templates should not be removed.
   * @param templateHandle the handle to the template to verify removability.
   * Assumes template exists.
   * @return Whether the template is read-only or not
   */
  bool isTemplateReadOnly(const std::string& templateHandle) override {
    for (auto handle : defaultTemplateNames) {
      if (handle.compare(templateHandle) == 0) {
        return true;
      }
    }
    return false;
  };

  /**
   * @brief vector holding string template handles of all default primitive
   * asset templates, to make sure they are never deleted.
   */
  std::vector<std::string> defaultTemplateNames;

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed class name
   */
  const AbstractPrimitiveAttributes::ptr buildPrimAttributes(
      const std::string& primTypeName) {
    if (primTypeConstructorMap_.count(primTypeName) == 0) {
      LOG(ERROR) << "AssetAttributesManager::buildPrimAttributes : No "
                    "primitive of type"
                 << primTypeName << "exists.  Aborting.";
      return nullptr;
    }
    return (*this.*primTypeConstructorMap_[primTypeName])();
  }  // AssetAttributeManager::buildPrimAttributes

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed enum value, which maps to class name via @ref
   * PrimitiveNames3DMap
   */
  const AbstractPrimitiveAttributes::ptr buildPrimAttributes(
      PrimObjTypes primType) {
    if (primType == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      LOG(ERROR) << "AssetAttributesManager::buildPrimAttributes : Illegal "
                    "primtitive type name PrimObjTypes::END_PRIM_OBJ_TYPES.  "
                    "Aborting.";
      return nullptr;
    }
    return (*this.*primTypeConstructorMap_[PrimitiveNames3DMap.at(primType)])();
  }  // AssetAttributeManager::buildPrimAttributes

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed enum value, which maps to class name via @ref
   * PrimitiveNames3DMap
   */
  const AbstractPrimitiveAttributes::ptr buildPrimAttributes(int primTypeVal) {
    if ((primTypeVal < 0) ||
        (primTypeVal > static_cast<int>(PrimObjTypes::END_PRIM_OBJ_TYPES))) {
      LOG(ERROR) << "AssetAttributesManager::buildPrimAttributes : Unknown "
                    "PrimObjTypes value requested :"
                 << primTypeVal << ". Aborting";
      return nullptr;
    }
    return (*this.*primTypeConstructorMap_[PrimitiveNames3DMap.at(
                       static_cast<PrimObjTypes>(primTypeVal))])();
  }  // AssetAttributeManager::buildPrimAttributes

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   */
  template <typename T, bool isWireFrame, PrimObjTypes primitiveType>
  const AbstractPrimitiveAttributes::ptr createPrimAttributes() {
    if (primitiveType == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      LOG(ERROR)
          << "AssetAttributeManager::createPrimAttributes : Cannot instantiate "
             "AbstractPrimitiveAttributes object for "
             "PrimObjTypes::END_PRIM_OBJ_TYPES. Aborting.";
      return nullptr;
    }
    int idx = static_cast<int>(primitiveType);
    return T::create(isWireFrame, idx, PrimitiveNames3DMap.at(primitiveType));
  }  // AssetAttributeManager::createPrimAttributes

  /**
   * @brief Any Assset-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {
    // build default AbstractPrimitiveAttributes objects - reset does not remove
    // constructor mappings;
    for (const std::pair<PrimObjTypes, const char*>& elem :
         PrimitiveNames3DMap) {
      if (elem.first == PrimObjTypes::END_PRIM_OBJ_TYPES) {
        continue;
      }
      createAttributesTemplate(elem.second, true);
    }
  }  // AssetAttributesManager::resetFinalize()

  /**
   * @brief This function will assign the appropriately configured function
   * pointers for @ref createPrimAttributes calls for each type of
   * supported primitive to the @ref primTypeConstructorMap, keyed by type of
   * primtive
   */
  void buildCtorFuncPtrMaps() override;

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createPrimAttributes() keyed by string names of classes being instanced,
   * as defined in @ref PrimNames3D
   */
  typedef std::map<std::string,
                   const AbstractPrimitiveAttributes::ptr (
                       AssetAttributesManager::*)()>
      Map_Of_PrimTypeCtors;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_PrimTypeCtors primTypeConstructorMap_;

 public:
  ESP_SMART_POINTERS(AssetAttributesManager)

};  // AssetAttributesManager
}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_