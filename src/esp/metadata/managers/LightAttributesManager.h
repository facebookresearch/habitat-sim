// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_

#include "AttributesManagerBase.h"

#include "esp/metadata/attributes/LightAttributes.h"

namespace Cr = Corrade;

namespace esp {
namespace metadata {
namespace managers {
class LightAttributesManager
    : public AttributesManager<attributes::LightAttributes> {
 public:
  LightAttributesManager()
      : AttributesManager<attributes::LightAttributes>::AttributesManager(
            "Light",
            "lighting_config.json") {
    buildCtorFuncPtrMaps();
  }

  /**
   * @brief Creates one or more instances of LightAttributes based on the
   * whether @p lightConfigName is a file or a not.  If it is a file it will
   * consider the contents of that file a layout and will use the file name as
   * the layout name and load all the attributes described and assign them to
   * that layout.  File-based loads will automatically register, regardless of
   * what @p registerTemplate is.
   *
   * If a template/layout exists with this handle, this existing template/layout
   * will be overwritten with the newly created one if registerTemplate is true.
   *
   * @param lightConfigName The configuration file to parse, or the name of the
   * single light's attributs to create.
   * @param registerTemplate whether to add this template to the library.
   * Defaults to false - overridden if @p lightConfigName is a JSON file.
   * @return a reference to the created light attributes.
   */
  attributes::LightAttributes::ptr createObject(
      const std::string& lightConfigName,
      bool registerTemplate = false) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param lightAttribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::LightAttributes::ptr lightAttribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief If the passed attributes exists, add it to named layout.
   * @param attribsHandle Handle of lighting attributes to add to layout
   * @param layoutName Name of layout to add to
   * @return Whether successful or not.
   */
  bool addLightToLayout(const std::string& attribsHandle,
                        const std::string& layoutName) {
    if (!this->getObjectLibHasHandle(attribsHandle)) {
      LOG(WARNING) << "LightAttributesManager::addLightToLayout : "
                   << attribsHandle
                   << " not found in light attributes library.  Aborting.";
      return false;
    }
    // add ref to layout to this attribs' layouts
    layoutsBelongedTo_[attribsHandle].emplace(layoutName);
    // add ref to attribs to layouts' attribs
    lightingLayoutsByName_[layoutName].emplace(attribsHandle);
    return true;
  }  // LightAttributesManager::addLightToLayout

  /**
   * @brief Parse passed JSON Document for @ref
   * esp::metadata::attributes::LightAttributes.  This overrides the class
   * template implementation.
   *
   * @param layoutName The desired name for the lighting layout these
   * attributes will belong to.
   * @param jsonConfig json document to parse
   * @return a reference to the first desired template.
   */
  AttribsPtr buildObjectFromJSONDoc(
      const std::string& layoutName,
      const io::JsonGenericValue& jsonConfig) override;

 protected:
  /**
   * @brief Light Attributes has no reason to check this value
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  bool isValidPrimitiveAttributes(
      CORRADE_UNUSED const std::string& handle) override {
    return false;
  }

  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   */
  attributes::LightAttributes::ptr initNewObjectInternal(
      const std::string& handleName) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing the
   * referenced passed template handle from all lighting layouts it may be
   * referenced in.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void updateObjectHandleLists(int templateID,
                               const std::string& templateHandle) override;

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::LightAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param lightAttributesTemplate The attributes template.
   * @param physicsAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::LightAttributes::ptr lightAttributesTemplate,
      const std::string& lightAttributesHandle) override;

  /**
   * @brief Any physics-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<PhysicsSceneAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["LightAttributes"] =
        &LightAttributesManager::createObjectCopy<attributes::LightAttributes>;
  }  // LightAttributesManager::buildCtorFuncPtrMaps

  // instance vars

 protected:
  /**
   * @brief Maps layout names to a set of handles of light attributes forming
   * the layout.
   */
  std::map<std::string, std::set<std::string>> lightingLayoutsByName_;

  /**
   * @brief Maps name of attributes' handles to set of names of layouts that
   * attributes can be found in.
   */
  std::map<std::string, std::set<std::string>> layoutsBelongedTo_;

 public:
  ESP_SMART_POINTERS(LightAttributesManager)

};  // LightAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_LIGHTATTRIBUTEMANAGER_H_
