// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_OBJECTATTRIBUTEMANAGER_H_
#define ESP_ASSETS_MANAGERS_OBJECTATTRIBUTEMANAGER_H_

#include <Corrade/Utility/Assert.h>

#include "AssetAttributesManager.h"
#include "AttributesManagerBase.h"

namespace esp {
namespace assets {
namespace managers {
/**
 * @brief single instance class managing templates describing physical objects
 */
class ObjectAttributesManager
    : public AttributesManager<Attrs::ObjectAttributes::ptr> {
 public:
  ObjectAttributesManager(assets::ResourceManager& resourceManager)
      : AttributesManager<Attrs::ObjectAttributes::ptr>::AttributesManager(
            resourceManager,
            "Object") {
    buildCtorFuncPtrMaps();
  }

  void setAssetAttributesManager(
      AssetAttributesManager::cptr assetAttributesMgr) {
    assetAttributesMgr_ = assetAttributesMgr;
    // Create default primitive-based object attributess
    createDefaultPrimBasedAttributesTemplates();
  }

  /**
   * @brief Creates an instance of an object template. The passed string should
   * be either a file name or a reference to a primitive asset template that
   * should be used in the construction of the object; any other strings will
   * result in a new default template being created.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired template to be
   * created, either a file name or an existing primitive asset template. If
   * this is neither a recognized file name nor the handle of an existing
   * primitive asset, a new default template will be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  Attrs::ObjectAttributes::ptr createAttributesTemplate(
      const std::string& attributesTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Creates an instance of an empty object template populated with
   * default values. Assigns the @ref templateName as the template's handle and
   * as the renderAssetHandle.
   *
   * If a template exists with this handle, the existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   * This method is specifically intended to directly construct an attributes
   * template for editing, and so defaults to false for @ref registerTemplate.
   *
   * @param templateName The desired name for this template.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to false. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template, or nullptr if fails.
   */
  Attrs::ObjectAttributes::ptr createDefaultAttributesTemplate(
      const std::string& templateName,
      bool registerTemplate = false) override;

  /**
   * @brief Creates an instance of an object template described by passed
   * string, which should be a reference to an existing primitive asset template
   * to be used in the construction of the object (as render and collision
   * mesh). It returns created instance if successful, and nullptr if fails.
   *
   * @param primAttrTemplateHandle The handle to an existing primitive asset
   * template. Fails if does not.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true.
   * @return a reference to the desired template, or nullptr if fails.
   */

  Attrs::ObjectAttributes::ptr createPrimBasedAttributesTemplate(
      const std::string& primAttrTemplateHandle,
      bool registerTemplate = true);

  /**
   * @brief Parse passed JSON Document specifically for @ref ObjectAttributes
   * object. It always returns a valid @ref ObjectAttributes::ptr object.
   *
   * @param templateName The desired name for this @ref ObjectAttributes
   * template.
   * @param jsonConfig json document to parse
   * @return a reference to the desired template.
   */
  Attrs::ObjectAttributes::ptr loadAttributesFromJSONDoc(
      const std::string& templateName,
      const io::JsonDocument& jsonConfig) override;

  /**
   * @brief Load file-based object templates for all "*.phys_properties.json"
   * files from the provided file or directory path.
   *
   * This will take the passed @ref path string and either treat it as a file
   * name or a directory, depending on what is found in the filesystem. If @ref
   * path does not end with ".phys_properties.json", it will append this and
   * check to see if such a file exists, and load it. It will also check if @ref
   * path exists as a directory, and if so will perform a shallow search to find
   * any files ending in "*.phys_properties.json" and load those that are found.
   *
   * @param path A global path to a physics property file or directory
   * containing such files.
   * @param saveAsDefaults Set the templates loaded as undeleteable default
   * templates.
   * @return A list of template indices for loaded valid object configs
   */
  std::vector<int> loadObjectConfigs(const std::string& path,
                                     bool saveAsDefaults = false);

  /**
   * @brief Load all file-based object templates given string list of object
   * template file locations.
   *
   * This will take the list of file names currently specified in
   * physicsManagerAttributes and load the referenced object templates.
   * @param tmpltFilenames list of file names of object templates
   * @param saveAsDefaults Set these templates as un-deletable from library.
   * @return vector holding IDs of templates that have been added
   */
  std::vector<int> loadAllFileBasedTemplates(
      const std::vector<std::string>& tmpltFilenames,
      bool saveAsDefaults);

  /**
   * @brief Check if currently configured primitive asset template library has
   * passed handle.
   * @param handle String name of primitive asset attributes desired
   * @return whether handle exists or not in asset attributes library
   */
  bool isValidPrimitiveAttributes(const std::string& handle) {
    return assetAttributesMgr_->getTemplateLibHasHandle(handle);
  }

  // ======== File-based and primitive-based partition functions ========

  /**
   * @brief Gets the number of file-based loaded object templates stored in the
   * @ref physicsObjTemplateLibrary_.
   *
   * @return The number of entries in @ref physicsObjTemplateLibrary_ that are
   * loaded from files.
   */
  int getNumFileTemplateObjects() const {
    return physicsFileObjTmpltLibByID_.size();
  }

  /**
   * @brief Get a random loaded attribute handle for the loaded file-based
   * object templates
   *
   * @return a randomly selected handle corresponding to a file-based object
   * attributes template, or empty string if none loaded
   */
  std::string getRandomFileTemplateHandle() const {
    return this->getRandomTemplateHandlePerType(physicsFileObjTmpltLibByID_,
                                                "file-based ");
  }

  /**
   * @brief Get a list of all file-based templates whose origin handles contain
   * @ref subStr, ignoring subStr's case
   * @param subStr substring to search for within existing file-based object
   * templates
   * @param contains whether to search for keys containing, or not containing,
   * @ref subStr
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getFileTemplateHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true) const {
    return this->getTemplateHandlesBySubStringPerType(
        physicsFileObjTmpltLibByID_, subStr, contains);
  }

  /**
   * @brief Gets the number of synthesized (primitive-based)  template objects
   * stored in the @ref physicsObjTemplateLibrary_.
   *
   * @return The number of entries in @ref physicsObjTemplateLibrary_ that
   * describe primitives.
   */
  int getNumSynthTemplateObjects() const {
    return physicsSynthObjTmpltLibByID_.size();
  }

  /**
   * @brief Get a random loaded attribute handle for the loaded synthesized
   * (primitive-based) object templates
   *
   * @return a randomly selected handle corresponding to the a primitive
   * attributes template, or empty string if none loaded
   */
  std::string getRandomSynthTemplateHandle() const {
    return this->getRandomTemplateHandlePerType(physicsSynthObjTmpltLibByID_,
                                                "synthesized ");
  }

  /**
   * @brief Get a list of all synthesized (primitive-based) object templates
   * whose origin handles contain @ref subStr, ignoring subStr's case
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains whether to search for keys containing, or not containing,
   * @ref subStr
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getSynthTemplateHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true) const {
    return this->getTemplateHandlesBySubStringPerType(
        physicsSynthObjTmpltLibByID_, subStr, contains);
  }

  // ======== End File-based and primitive-based partition functions ========

 protected:
  /**
   * @brief Create and save default primitive asset-based object templates,
   * saving their handles as non-deletable default handles.
   */
  void createDefaultPrimBasedAttributesTemplates();

  /**
   * @brief Perform file-name-based attributes initialization. This is to
   * take the place of the AssetInfo::fromPath functionality, and is only
   * intended to provide default values and other help if certain mistakes
   * are made by the user, such as specifying an asset handle in json but not
   * specifying the asset type corresponding to that handle.  These settings
   * should not restrict anything, only provide defaults.
   *
   * @param attributes The AbstractObjectAttributes object to be configured
   * @param setFrame whether the frame should be set or not (only for render
   * assets in scenes)
   * @param fileName Mesh Handle to check.
   * @param meshTypeSetter Setter for mesh type.
   */
  void setDefaultFileNameBasedAttributes(
      Attrs::ObjectAttributes::ptr attributes,
      bool setFrame,
      const std::string& meshHandle,
      std::function<void(int)> meshTypeSetter) override;

  /**
   * @brief Used Internally.  Configure newly-created attributes with any
   * default values, before any specific values are set.
   *
   * @param newAttributes Newly created attributes.
   */
  Attrs::ObjectAttributes::ptr initNewAttribsInternal(
      Attrs::ObjectAttributes::ptr newAttributes) override;

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
      int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {
    physicsFileObjTmpltLibByID_.erase(templateID);
    physicsSynthObjTmpltLibByID_.erase(templateID);
  }

  /**
   * @brief Add a copy of @ref AbstractAttributes object to the @ref
   * templateLibrary_. Verify that render and collision handles have been
   * set properly.  We are doing this since these values can be modified by the
   * user.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in the
   * @ref templateLibrary_. Will be set as origin handle for template.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplateFinalize(
      Attrs::ObjectAttributes::ptr attributesTemplate,
      const std::string& attributesTemplateHandle) override;

  /**
   * @brief Any object-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {
    physicsFileObjTmpltLibByID_.clear();
    physicsSynthObjTmpltLibByID_.clear();
  }

  /**
   * @brief This function will assign the appropriately configured function
   * pointer for the copy constructor as defined in
   * AttributesManager<ObjectAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["ObjectAttributes"] =
        &ObjectAttributesManager::createAttributesCopy<Attrs::ObjectAttributes>;
  }  // ObjectAttributesManager::buildCtorFuncPtrMaps()

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Reference to AssetAttributesManager to give access to primitive
   * attributes for object construction
   */
  AssetAttributesManager::cptr assetAttributesMgr_ = nullptr;

  /**
   * @brief Maps loaded object template IDs to the appropriate template
   * handles
   */
  std::map<int, std::string> physicsFileObjTmpltLibByID_;

  /**
   * @brief Maps synthesized, primitive-based object template IDs to the
   * appropriate template handles
   */
  std::map<int, std::string> physicsSynthObjTmpltLibByID_;

 public:
  ESP_SMART_POINTERS(ObjectAttributesManager)

};  // ObjectAttributesManager

}  // namespace managers
}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_MANAGERS_OBJECTATTRIBUTEMANAGER_H_
