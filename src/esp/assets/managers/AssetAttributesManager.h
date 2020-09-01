// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_

/** @file
 * @brief Class Template @ref esp::assets::AssetAttributesManager
 * This class manages attributes describing/configuring magnum mesh
 * primitives.
 */

#include "AttributesManagerBase.h"
#include "esp/assets/attributes/PrimitiveAssetAttributes.h"

namespace esp {
namespace assets {
/**
 * @brief The kinds of primitive modelled objects supported. Paired with
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
    : public AttributesManager<Attrs::AbstractPrimitiveAttributes::ptr> {
 public:
  /**
   * @brief Constant Map holding names of all Magnum 3D primitive classes
   * supported, keyed by @ref PrimObjTypes enum entry. Note final entry is not
   * a valid primitive.
   */
  static const std::map<PrimObjTypes, const char*> PrimitiveNames3DMap;

  AssetAttributesManager(assets::ResourceManager& resourceManager)
      : AttributesManager<Attrs::AbstractPrimitiveAttributes::ptr>::
            AttributesManager(resourceManager, "Primitive Asset") {
    buildCtorFuncPtrMaps();
  }  // AssetAttributesManager::ctor

  /**
   * @brief Should only be called internally. Creates an instance of a primtive
   * asset attributes template described by passed string. For primitive assets
   * this is the Magnum primitive class name
   *
   * @param primClassName A string descriptor of the primitive asset
   * template to be created, corresponding to the Magnum Primitive class
   * name.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */

  Attrs::AbstractPrimitiveAttributes::ptr createAttributesTemplate(
      const std::string& primClassName,
      bool registerTemplate = true) override;

  /**
   * @brief Creates an instance of a template holding default values. For asset
   * attributes this is the same functionality as @ref createAttributesTemplate.
   *
   * @param primClassName A string descriptor of the primitive asset
   * template to be created, corresponding to the Magnum Primitive class
   * name.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to false. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  Attrs::AbstractPrimitiveAttributes::ptr createDefaultAttributesTemplate(
      const std::string& primClassName,
      bool registerTemplate = false) override {
    return createAttributesTemplate(primClassName, registerTemplate);
  }  // Attrs::AbstractPrimitiveAttributes::ptr createDefaultAttributesTemplate

  /**
   * @brief Parse passed JSON Document specifically for @ref
   * AbstractPrimitiveAttributes object. It always returns a valid @ref
   * AbstractPrimitiveAttributes::ptr object.
   *
   * TODO : currently do not support file-based Primitive Assets, so no actual
   * JSON parsing.
   * @param filename the name of the file describing the asset attributes, used
   * to determine type of attributes template.
   * @param jsonConfig json document to parse
   * @return a reference to the desired template.
   */
  Attrs::AbstractPrimitiveAttributes::ptr loadAttributesFromJSONDoc(
      const std::string& filename,
      const io::JsonDocument& jsonConfig) override;

  /**
   * @brief Should only be called internally. Creates an instance of a
   * primtive asset attributes template described by passed enum value. For
   * primitive assets this mapes to the Magnum primitive class name
   *
   * @param primObjType an enum value denoting the class of the primitive to
   * instantiate
   * @param registerTemplate whether to add this template to the library or
   * not. If the user is going to edit this template, this should be false.
   * @return a reference to the desired template.
   */
  Attrs::AbstractPrimitiveAttributes::ptr createAttributesTemplate(
      PrimObjTypes primObjType,
      bool registerTemplate = true) {
    if (primObjType == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      LOG(ERROR)
          << "AssetAttributesManager::createAttributesTemplate : Illegal "
             "primtitive type name PrimObjTypes::END_PRIM_OBJ_TYPES. "
             "Aborting.";
      return nullptr;
    }
    return createAttributesTemplate(PrimitiveNames3DMap.at(primObjType),
                                    registerTemplate);
  }  // AssetAttributesManager::createAttributesTemplate

  /**
   * @brief Get list of primitive asset template handles used as keys in @ref
   * primitiveAssetTemplateLibrary_ related to passed primitive descriptor enum.
   *
   * @param primType Enum describing primitive type
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
                    "name PrimObjTypes::END_PRIM_OBJ_TYPES. Aborting.";
      return {};
    }
    std::string subStr = PrimitiveNames3DMap.at(primType);
    return this->getTemplateHandlesBySubStringPerType(this->templateLibKeyByID_,
                                                      subStr, contains);
  }  // AssetAttributeManager::getTemplateHandlesByPrimType

  /**
   * @brief Return the default capsule template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::CapsulePrimitiveAttributes::ptr getDefaultCapsuleTemplate(
      bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("capsule3DWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("capsule3DSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::CapsulePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultCapsuleTemplate

  /**
   * @brief Return the spedified capsule template.
   * @param templateHndle The handle of the desired capsule template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::CapsulePrimitiveAttributes::ptr getCapsuleTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "capsule")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::CapsulePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getCapsuleTemplate

  /**
   * @brief Return the default cone template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::ConePrimitiveAttributes::ptr getDefaultConeTemplate(bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("coneWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("coneSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::ConePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultConeTemplate

  /**
   * @brief Return the spedified cone template, either solid or wireframe.
   * @param templateHndle The handle of the desired cone template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::ConePrimitiveAttributes::ptr getConeTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "cone")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::ConePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getConeTemplate

  /**
   * @brief Return the default cube template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::CubePrimitiveAttributes::ptr getDefaultCubeTemplate(bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("cubeWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("cubeSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::CubePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultCubeTemplate

  /**
   * @brief Return the spedified cube template.
   * @param templateHndle The handle of the desired cube template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::CubePrimitiveAttributes::ptr getCubeTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "cube")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::CubePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getCubeTemplate

  /**
   * @brief Return the default cylinder template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::CylinderPrimitiveAttributes::ptr getDefaultCylinderTemplate(
      bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("cylinderWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("cylinderSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::CylinderPrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultCylinderTemplate

  /**
   * @brief Return the spedified cylinder template.
   * @param templateHndle The handle of the desired cylinder template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::CylinderPrimitiveAttributes::ptr getCylinderTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "cylinder")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::CylinderPrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getCylinderTemplate

  /**
   * @brief Return the default icosphere template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::IcospherePrimitiveAttributes::ptr getDefaultIcosphereTemplate(
      bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("icosphereWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("icosphereSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::IcospherePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultIcosphereTemplate

  /**
   * @brief Return the spedified icosphere template.
   * @param templateHndle The handle of the desired icosphere template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::IcospherePrimitiveAttributes::ptr getIcosphereTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "icosphere")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::IcospherePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getIcosphereTemplate

  /**
   * @brief Return the default UVSphere template, either solid or wireframe.
   * @param isWireFrame whether should be wireframe or solid template
   * @return appropriately cast template
   */
  Attrs::UVSpherePrimitiveAttributes::ptr getDefaultUVSphereTemplate(
      bool isWireFrame) {
    std::string templateHndle;
    if (isWireFrame) {
      templateHndle = defaultPrimAttributeHandles_.at("uvSphereWireframe");
    } else {
      templateHndle = defaultPrimAttributeHandles_.at("uvSphereSolid");
    }
    return this->getTemplateCopyByHandle<Attrs::UVSpherePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getDefaultUVSphereTemplate

  /**
   * @brief Return the spedified cube template.
   * @param templateHndle The handle of the desired cube template. Verifies
   * that handle is to specified template type
   * @return appropriately cast template, or nullptr if template handle
   * incorrectly specified.
   */
  Attrs::UVSpherePrimitiveAttributes::ptr getUVSphereTemplate(
      const std::string& templateHndle) {
    if (!verifyTemplateHandle(templateHndle, "uvSphere")) {
      return nullptr;
    }
    return this->getTemplateCopyByHandle<Attrs::UVSpherePrimitiveAttributes>(
        templateHndle);
  }  // AssetAttributeManager::getUVSphereTemplate

 protected:
  /**
   * @brief Not used by Attrs::AbstractPrimitiveAttributes.
   */
  void setDefaultFileNameBasedAttributes(
      CORRADE_UNUSED Attrs::AbstractPrimitiveAttributes::ptr attributes,
      CORRADE_UNUSED bool setFrame,
      CORRADE_UNUSED const std::string& meshHandle,
      CORRADE_UNUSED std::function<void(int)> meshTypeSetter) override {}

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void updateTemplateHandleLists(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Verify that passed template handle describes attributes of type
   * specified by passed primtive name (ie "cube", "capsule")
   * @param templateHandle The handle to test.
   * @param attrType Type of primitive the attributes owning templateHandle
   * describe.
   * @return If template handle describes template for objects of desired
   * primitive class name
   */
  bool verifyTemplateHandle(const std::string& templateHandle,
                            const std::string& attrType) {
    if (std::string::npos == templateHandle.find(attrType)) {
      LOG(ERROR) << "AssetAttributesManager::verifyTemplateHandle : Handle : "
                 << templateHandle << " is not of appropriate type for desired "
                 << attrType << " primitives. Aborting.";
      return false;
    }
    return true;
  }  // AttributesManager::verifyTemplateHandle

  /**
   * @brief Add an @ref Attrs::AbstractPrimitiveAttributes object to the @ref
   * templateLibrary_.
   *
   * @param attributesTemplate The attributes template.
   * @param ignored Not used for asset attributes templates - handle is derived
   * by configuration.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplateFinalize(
      Attrs::AbstractPrimitiveAttributes::ptr attributesTemplate,
      const std::string& ignored = "") override;

  /**
   * @brief Used Internally.  Configure newly-created attributes with any
   * default values, before any specific values are set.
   *
   * @param newAttributes Newly created attributes.
   */
  Attrs::AbstractPrimitiveAttributes::ptr initNewAttribsInternal(
      Attrs::AbstractPrimitiveAttributes::ptr newAttributes) override {
    return newAttributes;
  }

  /**
   * @brief Build an @ref AbstractPrimtiveAttributes object of type associated
   * with passed class name
   * @param primClassName Magnum::Primitives class name of primitive being
   * constructed
   */
  Attrs::AbstractPrimitiveAttributes::ptr buildPrimAttributes(
      const std::string& primClassName) {
    if (primTypeConstructorMap_.count(primClassName) == 0) {
      LOG(ERROR) << "AssetAttributesManager::buildPrimAttributes : No "
                    "primitive class"
                 << primClassName << "exists in Magnum::Primitives. Aborting.";
      return nullptr;
    }
    return initNewAttribsInternal(
        (*this.*primTypeConstructorMap_[primClassName])());
  }  // AssetAttributeManager::buildPrimAttributes

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   */
  template <typename T, bool isWireFrame, PrimObjTypes primitiveType>
  Attrs::AbstractPrimitiveAttributes::ptr createPrimAttributes() {
    if (primitiveType == PrimObjTypes::END_PRIM_OBJ_TYPES) {
      LOG(ERROR)
          << "AssetAttributeManager::createPrimAttributes : Cannot instantiate "
             "Attrs::AbstractPrimitiveAttributes object for "
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
    // build default Attrs::AbstractPrimitiveAttributes objects - reset does not
    // remove constructor mappings;
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

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createPrimAttributes() keyed by string names of classes being
   * instanced, as defined in @ref PrimNames3D
   */
  typedef std::map<std::string,
                   Attrs::AbstractPrimitiveAttributes::ptr (
                       AssetAttributesManager::*)()>
      Map_Of_PrimTypeCtors;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_PrimTypeCtors primTypeConstructorMap_;

  /**
   * @brief Map relating primitive class name to default attributes template
   * handle. There should always be a template for each of these handles.
   */
  std::map<std::string, std::string> defaultPrimAttributeHandles_;

 public:
  ESP_SMART_POINTERS(AssetAttributesManager)

};  // AssetAttributesManager
}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_ASSETATTRIBUTEMANAGER_H_
