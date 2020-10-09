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
    // add ref to layout to this attribs' layouts.  Creates lightset if none
    // exists.
    layoutsBelongedTo_[attribsHandle].emplace(layoutName);
    // add ref to attribs to layouts' attribs - creates layout if none exists.
    lightingLayoutsByName_[layoutName].emplace(attribsHandle);
    return true;
  }  // LightAttributesManager::addLightToLayout

  /**
   * @brief Returns a list of all lightAttributes belonging to a certain layout
   * @param layoutName name of layout to query for.
   * @return list of light attributes belonging to @p layoutName
   */
  std::vector<std::string> getLightHandlesInLayout(
      const std::string& layoutName) {
    std::vector<std::string> res;
    if (this->lightingLayoutsByName_.count(layoutName) == 0) {
      LOG(WARNING) << "LightAttributesManager::getLightHandlesInLayout : "
                   << layoutName << " not found in layout library.";
    } else {
      const auto& tmpList = this->lightingLayoutsByName_.at(layoutName);
      std::copy(tmpList.begin(), tmpList.end(), std::back_inserter(res));
    }
    return res;
  }  // LightAttributesManager::getLightHandlesInLayout

  /**
   * @brief Returns a list of handles of all layouts containing the passed
   * lightAttributes handle.
   * @param lightHandle name of layout to query for.
   * @return list of light attributes belonging to @p layoutName
   */
  std::vector<std::string> getLayoutsWithLightAttributes(
      const std::string& lightHandle) {
    std::vector<std::string> res;
    if (this->layoutsBelongedTo_.count(lightHandle) == 0) {
      LOG(WARNING) << "LightAttributesManager::getLayoutsWithLightAttributes : "
                   << lightHandle << " not found in lightAttributes library.";
    } else {
      const auto& tmpList = this->layoutsBelongedTo_.at(lightHandle);
      std::copy(tmpList.begin(), tmpList.end(), std::back_inserter(res));
    }
    return res;
  }  // LightAttributesManager::getLightHandlesInLayout

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

  /**
   * @brief Returns the number of defined lighting layouts.
   * @return Number of entries in layout library.
   */
  int getNumLightingLayouts() { return this->lightingLayoutsByName_.size(); }

  /**
   * @brief Get a list of all lighting layout handles whose handles contain
   * subStr, ignoring subStr's case
   * @param subStr substring to search for within existing file-based object
   * templates
   * @param contains whether to search for keys containing, or not containing,
   * subStr
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getLayoutHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true) const {
    return this->getObjectHandlesBySubStringPerType(lightingLayoutsByName_,
                                                    subStr, contains);
  }

  /**
   * @brief Remove passed @p layoutName.  All lightAttributes belonging to this
   * layout will remain but will not reference it anymore.
   * @param layoutName the layout to prune.
   */
  void removeLayoutFromAllLights(const std::string& layoutName) {
    if (this->lightingLayoutsByName_.count(layoutName) == 0) {
      LOG(WARNING) << "LightAttributesManager::removeLayout : " << layoutName
                   << " not found in layout library. Nothing to remove.";
    } else {
      const auto& tmpLightSet = this->lightingLayoutsByName_.at(layoutName);
      for (auto lightHandle : tmpLightSet) {
        this->removeLayoutFromLight(layoutName, lightHandle);
      }
      // remove entire layout
      this->lightingLayoutsByName_.erase(layoutName);
    }
  }  // LightAttributesManager::removeLayoutFromAllLights

  /**
   * @brief Remove passed @p lightAttributes handle from specified @p
   * layoutName, and remove the layout from passed  @p lightAttributes
   * collection.
   * @param lightAttributes the attributes to remove from the layout
   * @param layoutName the layout to prune.
   */
  void removeLightFromPassedLayout(const std::string& lightAttributes,
                                   const std::string& layoutName) {
    this->removeLightFromLayout(lightAttributes, layoutName);
    this->removeLayoutFromLight(layoutName, lightAttributes);
  }  // LightAttributesManager::removeLayout

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes\
   * @param builtFromConfig whether this LightAttributes is being built from a
   * config file, or from some other source (i.e. handleName contains config
   * file name)
   * @return Newly created but unregistered LightAttributes pointer, with only
   * default values set.
   */
  attributes::LightAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

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
   * @brief Used internally.  Assumes @p lightAttributes exists in passed
   * layout.
   * Remove passed @p lightAttributes handle from specified @p
   * layoutName.  Does not remove layout from lightAttributes' collection.
   * @param lightAttributes the attributes to remove from the layout
   * @param layoutName the layout to prune.
   */
  void removeLightFromLayout(const std::string& lightAttributes,
                             const std::string& layoutName) {
    if (this->lightingLayoutsByName_.count(layoutName) != 0) {
      this->lightingLayoutsByName_.at(layoutName).erase(lightAttributes);
    }
  }  // LightAttributesManager::removeLayout

  /**
   * @brief Used internally.  Assumes @p layoutName exists in passed
   * lightAttributes' collection.
   *
   * Remove passed @p layoutName handle from specified @p
   * lightAttributes collection.  Does not remove lightAttributes from layout.
   * @param layoutName the layout to remove from the passed lightAttributes
   * @param lightAttributes the attributes to prune
   * collection.
   */
  void removeLayoutFromLight(const std::string& layoutName,
                             const std::string& lightAttributes) {
    if (this->layoutsBelongedTo_.count(lightAttributes) != 0) {
      this->layoutsBelongedTo_.at(lightAttributes).erase(layoutName);
    }
  }  // LightAttributesManager::removeLayout

  /**
   * @brief Add a copy of the @ref
   * esp::metadata::attributes::LightAttributes shared_ptr object to
   * the @ref objectLibrary_.
   *
   * @param lightAttributesTemplate The attributes template.
   * @param lightAttributesHandle The key for referencing the template in the
   * @ref objectLibrary_.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::LightAttributes::ptr lightAttributesTemplate,
      const std::string& lightAttributesHandle) override;

  /**
   * @brief Any lights-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {}

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as required by
   * AttributesManager<LightAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["LightAttributes"] =
        &LightAttributesManager::createObjectCopy<attributes::LightAttributes>;
  }  // LightAttributesManager::buildCtorFuncPtrMaps

  /**
   * @brief Light Attributes has no reason to check this value
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  bool isValidPrimitiveAttributes(
      CORRADE_UNUSED const std::string& handle) override {
    return false;
  }

  // instance vars

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
