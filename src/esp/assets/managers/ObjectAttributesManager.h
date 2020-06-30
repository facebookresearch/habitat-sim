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
    : public AttributesManager<PhysicsObjectAttributes::ptr> {
 public:
  ObjectAttributesManager()
      : AttributesManager<PhysicsObjectAttributes::ptr>::AttributesManager() {
    buildCtorFuncPtrMaps();
  }

  void setAssetAttributesManager(
      AssetAttributesManager::cptr assetAttributesMgr) {
    assetAttributesMgr_ = assetAttributesMgr;
  }

  /**
   * @brief Creates an instance of an object template.  The passed string should
   * be either a file name or a reference to a primitive asset template that
   * should be used in the construction of the object; any other strings will
   * result in a new template being created.
   *
   * If a template exists with this handle, this existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired template to be
   * created, either a file name or an existing primitive asset template. If
   * this is neither a recognized file name nor the handle of an existing
   * primitive asset, a new template will be created.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration.
   * @return a reference to the desired template.
   */

  PhysicsObjectAttributes::ptr createAttributesTemplate(
      const std::string& attributesTemplateHandle,
      bool registerTemplate = true) override;

  /**
   * @brief Creates an instance of an object template described by passed
   * string, which should be a reference to an existing primitive asset template
   * to be used in the construction of the object (as render and collision
   * mesh).  It returns existing instance if there is one, and nullptr if fails.
   *
   * @param primAttrTemplateHandle The handle to an existing primitive asset
   * template.  Fails if does not.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the desired template, or nullptr if fails.
   */

  PhysicsObjectAttributes::ptr createPrimBasedAttributesTemplate(
      const std::string& primAttrTemplateHandle,
      bool registerTemplate = true);

  /**
   * @brief Creates an instance of a template from a file using passed filename.
   * It returns existing instance if there is one, and nullptr if fails
   *
   * @param filename the name of the file describing the object attributes.
   * Assumes it exists and fails if it does not.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the desired template, or nullptr if fails.
   */

  PhysicsObjectAttributes::ptr createFileBasedAttributesTemplate(
      const std::string& filename,
      bool registerTemplate = true);

  /**
   * @brief Creates an instance of an empty object template populated with
   * default values.  Assigns the templateName as the origin handle and as the
   * renderAssetHandle.
   *
   * @param templateName the name of the file describing the object attributes.
   * Registration fails if this is not a valid asset handle.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false.
   * @return a reference to the desired template, or nullptr if fails.
   */

  PhysicsObjectAttributes::ptr createEmptyAttributesTemplate(
      const std::string& templateName,
      bool registerTemplate = true);

  /**
   * @brief Load all file-based object templates given string list of object
   * template file locations.
   *
   * This will take the list of file names currently specified in
   * physicsManagerAttributes and load the referenced object templates.
   * @param tmpltFilenames list of file names of object templates
   * @return vector holding IDs of templates that have been added
   */
  std::vector<int> loadAllFileBasedTemplates(
      const std::vector<std::string>& tmpltFilenames);

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
  };
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
  };
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
   * @brief Add a copy of @ref AbstractAttributes object to the @ref
   * templateLibrary_.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in the
   * @ref templateLibrary_. Will be set as origin handle for template.
   * @return The index in the @ref templateLibrary_ of object
   * template.
   */
  int registerAttributesTemplateFinalize(
      PhysicsObjectAttributes::ptr attributesTemplate,
      const std::string& attributesTemplateHandle) override;

  /**
   * @brief Whether template described by passed handle is read only, or can be
   * deleted.  All objectAttributes templates are removable, by default
   */
  bool isTemplateReadOnly(const std::string&) override { return false; };
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
   * AttributesManager<PhysicsObjectAttributes::ptr>
   */
  void buildCtorFuncPtrMaps() override {
    this->copyConstructorMap_["PhysicsObjectAttributes"] =
        &ObjectAttributesManager::createAttributesCopy<
            assets::PhysicsObjectAttributes>;
  }  // ObjectAttributesManager::buildCtorFuncPtrMaps()
  /**
   * @brief Load and parse a physics object template config file and generate a
   * @ref PhysicsObjectAttributes object.  Will always reload from file, since
   * existing attributes may have been changed; the user would use this pathway
   * to return to original config.
   *
   * @param objPhysConfigFilename The configuration file to parse and load.
   * @return The object attributes specified by the config file.
   */
  PhysicsObjectAttributes::ptr parseAndLoadPhysObjTemplate(
      const std::string& objPhysConfigFilename);

  /**
   * @brief Instantiate a @ref PhysicsObjectAttributes for a
   * synthetic(primitive-based render) object. NOTE : Must be registered to be
   * available for use via @ref registerObjectTemplate.  This method is provided
   * so the user can modify a specified physics object template before
   * registering it.
   *
   * @param primAssetHandle The string name of the primitive asset attributes to
   * be used to synthesize a render asset and solve collisions implicitly for
   * the desired object.  Will also become the default handle of the resultant
   * @ref physicsObjectAttributes template
   * @return The @ref physicsObjectAttributes template based on the passed
   * primitive
   */
  PhysicsObjectAttributes::ptr buildPrimBasedPhysObjTemplate(
      const std::string& primAssetHandle);

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