// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_OBJECTATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_OBJECTATTRIBUTEMANAGER_H_

#include <Corrade/Utility/Assert.h>

#include <utility>

#include "AbstractObjectAttributesManager.h"
#include "esp/metadata/attributes/ObjectAttributes.h"

namespace esp {
namespace metadata {
namespace managers {
using core::managedContainers::ManagedFileBasedContainer;
using core::managedContainers::ManagedObjectAccess;

/**
 * @brief single instance class managing templates describing physical objects
 */
class ObjectAttributesManager
    : public AbstractObjectAttributesManager<attributes::ObjectAttributes,
                                             ManagedObjectAccess::Copy> {
 public:
  ObjectAttributesManager()
      : AbstractObjectAttributesManager<attributes::ObjectAttributes,
                                        ManagedObjectAccess::Copy>::
            AbstractObjectAttributesManager("Object", "object_config.json") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["ObjectAttributes"] =
        &ObjectAttributesManager::createObjectCopy<
            attributes::ObjectAttributes>;
  }

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
  attributes::ObjectAttributes::ptr createPrimBasedAttributesTemplate(
      const std::string& primAttrTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(attributes::ObjectAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  // ======== File-based and primitive-based partition functions ========

  /**
   * @brief Gets the number of file-based loaded object templates stored in the
   * @ref physicsFileObjTmpltLibByID_.
   *
   * @return The number of entries in @ref physicsFileObjTmpltLibByID_ that are
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
    return this->getRandomObjectHandlePerType(physicsFileObjTmpltLibByID_,
                                              "file-based ");
  }

  /**
   * @brief Get a list of all file-based templates whose origin handles contain
   * subStr, ignoring subStr's case
   * @param subStr substring to search for within existing file-based object
   * templates
   * @param contains whether to search for keys containing, or not containing,
   * subStr
   * @param sorted whether the return vector values are sorted
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getFileTemplateHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true,
      bool sorted = true) const {
    return this->getObjectHandlesBySubStringPerType(physicsFileObjTmpltLibByID_,
                                                    subStr, contains, sorted);
  }

  /**
   * @brief Gets the number of synthesized (primitive-based)  template objects
   * stored in the @ref physicsSynthObjTmpltLibByID_.
   *
   * @return The number of entries in @ref physicsSynthObjTmpltLibByID_ that
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
    return this->getRandomObjectHandlePerType(physicsSynthObjTmpltLibByID_,
                                              "synthesized ");
  }

  /**
   * @brief Get a list of all synthesized (primitive-based) object templates
   * whose origin handles contain subStr, ignoring subStr's case
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains whether to search for keys containing, or not containing,
   * subStr
   * @param sorted whether the return vector values are sorted
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getSynthTemplateHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true,
      bool sorted = true) const {
    return this->getObjectHandlesBySubStringPerType(
        physicsSynthObjTmpltLibByID_, subStr, contains, sorted);
  }

  // ======== End File-based and primitive-based partition functions ========

 protected:
  /**
   * @brief Create and save default primitive asset-based object templates,
   * saving their handles as non-deletable default handles.
   */
  void createDefaultPrimBasedAttributesTemplates() override;

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
   * @param meshHandle Mesh Handle to check.
   * @param assetTypeSetter Setter for mesh type.
   */
  void setDefaultAssetNameBasedAttributes(
      attributes::ObjectAttributes::ptr attributes,
      bool setFrame,
      const std::string& meshHandle,
      const std::function<void(int)>& assetTypeSetter) override;

  /**
   * @brief Used Internally.  Create and configure newly-created attributes
   * with any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig Whether this object attributes is being
   * constructed from a config file or from some other source.
   * @return Newly created but unregistered ObjectAttributes pointer, with
   * only default values set.
   */
  attributes::ObjectAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      bool builtFromConfig) override;

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called @ref
   * esp::core::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {
    physicsFileObjTmpltLibByID_.erase(templateID);
    physicsSynthObjTmpltLibByID_.erase(templateID);
  }

  /**
   * @brief Add a copy of @ref  esp::metadata::attributes::AbstractAttributes
   * object to the @ref objectLibrary_. Verify that render and collision
   * handles have been set properly.  We are doing this since these values can
   * be modified by the user.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in
   * the
   * @ref objectLibrary_. Will be set as origin handle for template.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The index in the @ref objectLibrary_ of object
   * template.
   */
  int registerObjectFinalize(
      attributes::ObjectAttributes::ptr attributesTemplate,
      const std::string& attributesTemplateHandle,
      bool forceRegistration) override;

  /**
   * @brief Any object-attributes-specific resetting that needs to happen on
   * reset.
   */
  void resetFinalize() override {
    physicsFileObjTmpltLibByID_.clear();
    physicsSynthObjTmpltLibByID_.clear();
  }

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Maps loaded object template IDs to the appropriate template
   * handles
   */
  std::unordered_map<int, std::string> physicsFileObjTmpltLibByID_;

  /**
   * @brief Maps synthesized, primitive-based object template IDs to the
   * appropriate template handles
   */
  std::unordered_map<int, std::string> physicsSynthObjTmpltLibByID_;

 public:
  ESP_SMART_POINTERS(ObjectAttributesManager)

};  // ObjectAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_OBJECTATTRIBUTEMANAGER_H_
