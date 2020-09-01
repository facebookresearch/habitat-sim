// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_
#define ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_

/** @file
 * @brief Class Template @ref esp::assets::managers::AttributesManager
 */

#include <deque>
#include <functional>
#include <map>
#include <set>

#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/assets/attributes/AttributesBase.h"
#include "esp/assets/attributes/ObjectAttributes.h"

#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {

class ResourceManager;

namespace managers {

namespace Attrs = esp::assets::attributes;
/**
 * @brief Template Class defining responsibilities for managing attributes for
 * different types of objects, such as scenes, primitive assets, physical
 * objects, etc.
 * @tparam AttribsPtr the type of attributes shared pointer object this class
 * references
 */

template <class AttribsPtr>
class AttributesManager {
 public:
  AttributesManager(assets::ResourceManager& resourceManager,
                    const std::string& attrType)
      : resourceManager_(resourceManager), attrType_(attrType) {}
  virtual ~AttributesManager() = default;

  /**
   * @brief Creates an instance of a template described by passed string. For
   * physical objects, this is either a file name or a reference to a primitive
   * template used in the construction of the object. For primitive assets, this
   * is the class name used to construct the underlying primitive.
   *
   * If a template exists with this handle, the existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   *
   * @param attributesTemplateHandle the origin of the desired template to be
   * created, either a file name or an existing primitive asset template. If is
   * not an origin handle to an existing primitive, assumes is file name.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false. Defaults
   * to true. If specified as true, then this function returns a copy of the
   * registered template.
   * @return a reference to the desired template.
   */
  virtual AttribsPtr createAttributesTemplate(
      const std::string& attributesTemplateHandle,
      bool registerTemplate = true) = 0;

  /**
   * @brief Creates an instance of a template holding default values.
   *
   * If a template exists with this handle, the existing template will be
   * overwritten with the newly created one if @ref registerTemplate is true.
   * This method is specifically intended to directly construct an attributes
   * template for editing, and so defaults to false for @ref registerTemplate
   *
   * @param templateName The desired handle for this template.
   * @param registerTemplate whether to add this template to the library or not.
   * If the user is going to edit this template, this should be false. If
   * specified as true, then this function returns a copy of the registered
   * template. Defaults to false. If specified as true, then this function
   * returns a copy of the registered template.
   * @return a reference to the desired template.
   */
  virtual AttribsPtr createDefaultAttributesTemplate(
      const std::string& templateName,
      bool registerTemplate = false) = 0;

  /**
   * @brief Creates an instance of a template from a JSON file using passed
   * filename by loading and parsing the loaded JSON and generating a @ref
   * AttribsPtr object. It returns created instance if successful,
   * and nullptr if fails.
   *
   * @param filename the name of the file describing the object attributes.
   * Assumes it exists and fails if it does not.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true.
   * @return a reference to the desired template, or nullptr if fails.
   */

  AttribsPtr createFileBasedAttributesTemplate(const std::string& filename,
                                               bool registerTemplate = true) {
    // Load JSON config file
    io::JsonDocument jsonConfig;
    bool success = this->verifyLoadJson(filename, jsonConfig);
    if (!success) {
      LOG(ERROR) << attrType_
                 << "AttributesManager::createFileBasedAttributesTemplate : "
                    "Failure reading json : "
                 << filename << ". Aborting.";
      return nullptr;
    }
    AttribsPtr attr = this->loadAttributesFromJSONDoc(filename, jsonConfig);
    return this->postCreateRegister(attr, registerTemplate);
  }  // AttributesManager::createFileBasedAttributesTemplate

  /**
   * @brief Parse passed JSON Document specifically for @ref AttribsPtr object.
   * It always returns a @ref AttribsPtr object.
   * @param filename Can be the name of the file describing the @ref AttribsPtr,
   * used for attributes handle on create and, for some attributes such as @ref
   * PrimAssetAttributes, to determine type of actual instanced attributes
   * template.
   * @param jsonConfig json document to parse - assumed to be legal JSON doc.
   * @return a reference to the desired template.
   */
  virtual AttribsPtr loadAttributesFromJSONDoc(
      const std::string& filename,
      const io::JsonDocument& jsonConfig) = 0;

  /**
   * @brief Add a copy of @ref AbstractAttributes object to the @ref
   * templateLibrary_.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in
   * the
   * @ref templateLibrary_. Will be set as origin handle for template. If
   * empty string, use existing origin handle.
   * @return The unique ID of the template being registered, or ID_UNDEFINED
   * if failed
   */
  int registerAttributesTemplate(
      AttribsPtr attributesTemplate,
      const std::string& attributesTemplateHandle = "") {
    if (nullptr == attributesTemplate) {
      LOG(ERROR) << "AttributesManager::registerAttributesTemplate : Invalid "
                    "(null) template passed to registration. Aborting.";
      return ID_UNDEFINED;
    }
    if ("" != attributesTemplateHandle) {
      return registerAttributesTemplateFinalize(attributesTemplate,
                                                attributesTemplateHandle);
    }
    std::string handleToSet = attributesTemplate->getHandle();
    if ("" == handleToSet) {
      LOG(ERROR) << "AttributesManager::registerAttributesTemplate : No "
                    "valid handle specified for "
                 << attrType_ << " attributes template to register. Aborting.";
      return ID_UNDEFINED;
    }
    return registerAttributesTemplateFinalize(attributesTemplate, handleToSet);
  }  // AttributesManager::registerAttributesTemplate

  /**
   * @brief Register template and call appropriate ResourceManager method to
   * execute appropriate post-registration processes due to change in
   * attributes. Use if user wishes to update existing objects built by
   * attributes with new attributes data and such objects support this kind of
   * update. Requires the use of Attributes' assigned handle in order to
   * reference existing constructions built from the original version of this
   * attributes.
   * @param attributesTemplate The attributes template.
   * @return The unique ID of the template being registered, or ID_UNDEFINED if
   * failed
   */
  int registerAttributesTemplateAndUpdate(AttribsPtr attributesTemplate) {
    std::string originalHandle = attributesTemplate->getHandle();
    int ID = registerAttributesTemplate(attributesTemplate, originalHandle);
    // If undefined then some error occurred.
    if (ID_UNDEFINED == ID) {
      return ID_UNDEFINED;
    }
    // TODO : call Resource Manager for post-registration processing of this
    // template

    return ID;
  }  // AttributesManager::registerAttributesTemplateAndUpdate

  /**
   * @brief clears maps of handle-keyed attributes and ID-keyed handles.
   */
  void reset() {
    templateLibKeyByID_.clear();
    templateLibrary_.clear();
    availableTemplateIDs_.clear();
    resetFinalize();
  }  // AttributesManager::reset

  // ======== Utility functions ========

  /**
   * @brief Utility function to check if passed string represents an existing,
   * user-accessible file
   * @param handle the string to check
   * @return whether the file exists in the file system and whether the user has
   * access
   */
  bool isValidFileName(const std::string& handle) const {
    return (Corrade::Utility::Directory::exists(handle));
  }  // AttributesManager::isValidFileName

  /**
   * @brief Sets/Clears lock state for a template, preventing or allowing
   * deletion of template.
   * @param templateHandle handle of template to set state of
   * @param lock boolean to set or clear lock
   * @return whether successful or not.
   */
  bool setTemplateLock(const std::string& templateHandle, bool lock);

  /**
   * @brief Sets/Clears lock state for a template, preventing or allowing
   * deletion of template.
   * @param templateHandles Vector of handles of templates to set state of
   * @param lock boolean to set or clear lock for all templates
   * @return the list of handles actually set to desired lock state.
   */
  std::vector<std::string> setTemplateLockByHandles(
      const std::vector<std::string>& templateHandles,
      bool lock) {
    std::vector<std::string> res;
    for (std::string templateHandle : templateHandles) {
      if (setTemplateLock(templateHandle, lock)) {
        res.push_back(templateHandle);
      }
    }
    return res;
  }  // AttributesManager::setTemplateLockByHandles

  /**
   * @brief set lock state on all templates that contain passed substring.
   * @param lock boolean to set or clear lock on templates
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains whether to search for keys containing, or excluding,
   * @ref subStr
   * @return A vector containing the template handles of templates whose lock
   * state has been set to passed state.
   */
  std::vector<std::string> setTemplatesLockBySubstring(
      bool lock,
      const std::string& subStr = "",
      bool contains = true) {
    std::vector<std::string> handles =
        getTemplateHandlesBySubstring(subStr, contains);
    return this->setTemplateLockByHandles(handles, lock);
  }  // AttributesManager::setTemplatesLockBySubstring

  // ======== Accessor functions ========
  /**
   * @brief Gets the number of object templates stored in the @ref
   * templateLibrary_.
   *
   * @return The size of the @ref templateLibrary_.
   */
  int getNumTemplates() const { return templateLibrary_.size(); }

  /**
   * @brief Checks whether template library has passed string handle as key
   * @param handle the key to look for
   */
  bool getTemplateLibHasHandle(const std::string& handle) const {
    return templateLibrary_.count(handle) > 0;
  }  // AttributesManager::getTemplateLibHasHandle

  /**
   * @brief Get a reference to the attributes template identified by the
   * attributesTemplateID.  Should only be used internally. Users should
   * only ever access copies of templates.
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param attributesTemplateID The ID of the template. Is mapped to the key
   * referencing the asset in @ref templateLibrary_ by @ref templateLibKeyByID_.
   * @return A mutable reference to the object template, or nullptr if does not
   * exist
   */
  AttribsPtr getTemplateByID(int attributesTemplateID) const {
    std::string templateHandle = getTemplateHandleByID(attributesTemplateID);
    if (!checkExistsWithMessage(templateHandle,
                                "AttributesManager::getTemplateByID")) {
      return nullptr;
    }
    return templateLibrary_.at(templateHandle);
  }  // AttributesManager::getTemplateByID

  /**
   * @brief Get a reference to the attributes template for the asset
   * identified by the passed templateHandle.  Should only be used
   * internally. Users should only ever access copies of templates.
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param templateHandle The key referencing the asset in @ref
   * templateLibrary_.
   * @return A reference to the object template, or nullptr if does not
   * exist
   */
  AttribsPtr getTemplateByHandle(const std::string& templateHandle) const {
    if (!checkExistsWithMessage(templateHandle,
                                "AttributesManager::getTemplateByHandle")) {
      return nullptr;
    }
    return templateLibrary_.at(templateHandle);
  }  // AttributesManager::getAttributesTemplate

  /**
   * @brief Remove the template referenced by the passed string handle. Will
   * emplace template ID within deque of usable IDs and return the template
   * being removed.
   * @param templateHandle the string key of the attributes desired.
   * @return the desired attributes being deleted, or nullptr if does not exist
   */
  AttribsPtr removeTemplateByID(int attributesTemplateID) {
    std::string templateHandle = getTemplateHandleByID(attributesTemplateID);
    if (!checkExistsWithMessage(templateHandle,
                                "AttributesManager::removeTemplateByID")) {
      return nullptr;
    }
    return removeTemplateInternal(templateHandle,
                                  "AttributesManager::removeTemplateByID");
  }

  /**
   * @brief  Remove the template referenced by the passed string handle. Will
   * emplace template ID within deque of usable IDs and return the template
   * being removed.
   * @param templateHandle the string key of the attributes desired.
   * @return the desired attributes being deleted, or nullptr if does not exist
   */
  AttribsPtr removeTemplateByHandle(const std::string& templateHandle) {
    return removeTemplateInternal(templateHandle,
                                  "AttributesManager::removeTemplateByHandle");
  }

  /**
   * @brief Remove all templates that have not been marked as
   * default/non-removable, and return a vector of the templates removed.
   * @return A vector containing all the templates that have been removed from
   * the library.
   */
  std::vector<AttribsPtr> removeAllTemplates() {
    return removeTemplatesBySubstring();
  }  // removeAllTemplates

  /**
   * @brief remove templates that contain passed substring and that have not
   * been marked as default/non-removable, and return a vector of the templates
   * removed.
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains whether to search for keys containing, or excluding,
   * @ref subStr
   * @return A vector containing all the templates that have been removed from
   * the library.
   */
  std::vector<AttribsPtr> removeTemplatesBySubstring(
      const std::string& subStr = "",
      bool contains = true);

  /**
   * @brief Get the key in @ref templateLibrary_ for the object template
   * with the given unique ID.
   *
   * @param templateID The unique ID of the desired template.
   * @return The key referencing the template in @ref
   * templateLibrary_, or nullptr if does not exist.
   */
  std::string getTemplateHandleByID(const int templateID) const {
    if (templateLibKeyByID_.count(templateID) == 0) {
      LOG(ERROR) << "AttributesManager::getTemplateHandleByID : Unknown "
                 << attrType_ << " template ID:" << templateID << ". Aborting";
      return nullptr;
    }
    return templateLibKeyByID_.at(templateID);
  }  // AttributesManager::getTemplateHandleByID

  /**
   * @brief Get the ID of the template in @ref templateLibrary_ for the given
   * template Handle, if exists.
   *
   * @param templateHandle The string key referencing the template in @ref
   * templateLibrary_. Usually the origin handle.
   * @return The object ID for the template with the passed handle, or
   * ID_UNDEFINED if none exists.
   */
  int getTemplateIDByHandle(const std::string& templateHandle) {
    return getTemplateIDByHandleOrNew(templateHandle, false);
  }  // AttributesManager::getTemplateIDByHandle

  /**
   * @brief Get a list of all templates whose origin handles contain @ref
   * subStr, ignoring subStr's case
   * @param subStr substring to search for within existing templates.
   * @param contains whether to search for keys containing, or excluding,
   * passed @ref subStr
   * @return vector of 0 or more template handles containing the passed
   * substring
   */
  std::vector<std::string> getTemplateHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true) const {
    return getTemplateHandlesBySubStringPerType(templateLibKeyByID_, subStr,
                                                contains);
  }  // AttributesManager::getTemplateHandlesBySubstring

  /**
   * @brief Get the handle for a random attributes template registered to this
   * manager.
   *
   * @return a randomly selected handle corresponding to a known object
   * attributes template, or empty string if none found
   */
  std::string getRandomTemplateHandle() const {
    return getRandomTemplateHandlePerType(templateLibKeyByID_, "");
  }  // AttributesManager::getRandomTemplateHandle

  /**
   * @brief Get a copy of the attributes template identified by the
   * attributesTemplateID.
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param attributesTemplateID The ID of the template. Is mapped to the key
   * referencing the asset in @ref templateLibrary_ by @ref templateLibKeyByID_.
   * @return A mutable reference to the object template, or nullptr if does not
   * exist
   */
  AttribsPtr getTemplateCopyByID(int attributesTemplateID) {
    std::string templateHandle = getTemplateHandleByID(attributesTemplateID);
    if (!checkExistsWithMessage(templateHandle,
                                "AttributesManager::getTemplateCopyByID")) {
      return nullptr;
    }
    auto orig = this->templateLibrary_.at(templateHandle);
    return this->copyAttributes(orig);
  }  // AttributesManager::getTemplateCopyByID

  /**
   * @brief Return a reference to a copy of the object specified
   * by passed handle. This is the version that should be accessed by the
   * user.
   * @param templateHandle the string key of the attributes desired.
   * @return a copy of the desired attributes, or nullptr if does
   * not exist
   */
  AttribsPtr getTemplateCopyByHandle(const std::string& templateHandle) {
    if (!checkExistsWithMessage(templateHandle,
                                "AttributesManager::getTemplateCopyByHandle")) {
      return nullptr;
    }
    auto orig = this->templateLibrary_.at(templateHandle);
    return this->copyAttributes(orig);
  }  // AttributesManager::getTemplateCopyByHandle

  /**
   * @brief Get a copy of the attributes template identified by the
   * attributesTemplateID, casted to the appropriate derived asset attributes
   * class
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param attributesTemplateID The ID of the template. Is mapped to the key
   * referencing the asset in @ref templateLibrary_ by @ref templateLibKeyByID_.
   * @return A mutable reference to the object template, or nullptr if does not
   * exist
   */
  template <class U>
  std::shared_ptr<U> getTemplateCopyByID(int attributesTemplateID) {
    std::string templateHandle = getTemplateHandleByID(attributesTemplateID);
    auto res = getTemplateCopyByID(attributesTemplateID);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // AttributesManager::getTemplateCopyByID

  /**
   * @brief Return a reference to a copy of the object specified
   * by passed handle, casted to the appropriate derived asset attributes class
   * This is the version that should be accessed by the user
   * @param templateHandle the string key of the attributes desired.
   * @return a copy of the desired attributes, or nullptr if does
   * not exist
   */
  template <class U>
  std::shared_ptr<U> getTemplateCopyByHandle(
      const std::string& templateHandle) {
    auto res = getTemplateCopyByHandle(templateHandle);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // AttributesManager::getTemplateCopyByHandle

  /**
   * @brief returns a vector of template handles representing the
   * system-specified undeletable templates this manager manages. These
   * templates cannot be deleted, although they can be edited.
   */
  std::vector<std::string> getUndeletableTemplateHandles() const {
    std::vector<std::string> res(this->undeletableTemplateNames_.begin(),
                                 this->undeletableTemplateNames_.end());
    return res;
  }  // AttributesManager::getUndeletableTemplateHandles

  /**
   * @brief returns a vector of template handles representing templates that
   * have been locked by the user.  These templates cannot be deleted until
   * they have been unlocked, although they can be edited while locked.
   */
  std::vector<std::string> getUserLockedTemplateHandles() const {
    std::vector<std::string> res(this->userLockedTemplateNames_.begin(),
                                 this->userLockedTemplateNames_.end());
    return res;
  }  // AttributesManager::

 protected:
  //======== Common JSON import functions ========

  /**
   * @brief Verify passed @ref filename is legal json document, return loaded
   * document or nullptr if fails
   *
   * @param filename name of potential json document to load
   * @param jsonDoc a reference to the json document to be parsed
   * @return whether document has been loaded successfully or not
   */
  bool verifyLoadJson(const std::string& filename, io::JsonDocument& jsonDoc) {
    if (isValidFileName(filename)) {
      try {
        jsonDoc = io::parseJsonFile(filename);
      } catch (...) {
        LOG(ERROR) << attrType_
                   << "AttributesManager::verifyLoadJson : Failed to parse "
                   << filename << " as JSON.";
        return false;
      }
      return true;
    } else {
      // by here always fail
      LOG(ERROR) << attrType_ << "AttributesManager::verifyLoadJson : File "
                 << filename << " does not exist";
      return false;
    }
  }  // AttributesManager::verifyLoadJson

  /**
   * @brief Create either an object or a scene attributes from a json config.
   * Since both object attributes and scene attributes inherit from @ref
   * AbstractObjectAttributes, the functionality to populate these fields from
   * json can be shared.  Also will will populate render mesh and collision mesh
   * handles in object and scene attributes with value(s) specified in json.  If
   * one is blank will use other for both.
   *
   * @tparam type of attributes to create : MUST INHERIT FROM @ref
   * AbstractObjectAttributes
   * @param filename name of json descriptor file
   * @param jsonDoc json document to parse
   * @return an appropriately cast attributes pointer with base class fields
   * filled in.
   */
  template <typename U>
  AttribsPtr createObjectAttributesFromJson(const std::string& filename,
                                            const io::JsonDocument& jsonDoc);

  /**
   * @brief Only used by @ref AbstractObjectAttributes derived-attributes. Set
   * the asset type and mesh asset filename from json file. If mesh asset
   * filename has changed in json, but type has not been specified in json,
   * re-run file-path-driven configuration to get asset type and possibly
   * orientation frame, if appropriate.
   *
   * @param attributes The AbstractObjectAttributes object to be populated
   * @param jsonDoc The json document
   * @param jsonMeshTypeTag The string tag denoting the desired mesh type in the
   * json.
   * @param jsonMeshHandleTag The string for the mesh asset handle.
   * @param fileName [in/out] On entry this is old mesh file handle, on exit is
   * new mesh file handle, or empty.
   * @param meshTypeSetter Function pointer to the appropriate mesh type setter
   * in the Attributes object.
   * @return Whether the render asset name was specified in the json and should
   * be set from fileName variable.
   */
  bool setJSONAssetHandleAndType(AttribsPtr attributes,
                                 const io::JsonDocument& jsonDoc,
                                 const char* jsonMeshTypeTag,
                                 const char* jsonMeshHandleTag,
                                 std::string& fileName,
                                 std::function<void(int)> meshTypeSetter);

  //======== Internally accessed functions ========
  /**
   * @brief Perform post creation registration if specified.
   *
   * @param attributes Attributes template
   * @param registerTemplate If template should be registered
   * @return attributes template, or null ptr if registration failed.
   */
  inline AttribsPtr postCreateRegister(AttribsPtr attributes,
                                       bool registerTemplate) {
    if (!registerTemplate) {
      return attributes;
    }
    int attrID =
        registerAttributesTemplate(attributes, attributes->getHandle());
    // return nullptr if registration error occurs.
    return (attrID == ID_UNDEFINED) ? nullptr : attributes;
  }  // postCreateRegister

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
  virtual void setDefaultFileNameBasedAttributes(
      AttribsPtr attributes,
      bool setFrame,
      const std::string& fileName,
      std::function<void(int)> meshTypeSetter) = 0;

  /**
   * @brief Get directory component of attributes handle and call @ref
   * attributes->setFileDirectory legitimate directory exists in handle.
   *
   * @param attributes pointer to attributes to set
   */
  void setFileDirectoryFromHandle(AttribsPtr attributes) {
    std::string handleName = attributes->getHandle();
    auto loc = handleName.find_last_of("/");
    if (loc != std::string::npos) {
      attributes->setFileDirectory(handleName.substr(0, loc));
    }
  }  // setFileDirectoryFromHandle

  /**
   * @brief Used Internally.  Configure newly-created attributes with any
   * default values, before any specific values are set.
   *
   * @param newAttributes Newly created attributes.
   */
  virtual AttribsPtr initNewAttribsInternal(AttribsPtr newAttributes) = 0;

  /**
   * @brief Used Internally. Remove the template referenced by the passed
   * string handle. Will emplace template ID within deque of usable IDs and
   * return the template being removed.
   * @param templateHandle the string key of the attributes desired.
   * @param src String denoting the source of the remove request.
   * @return the desired attributes being deleted, or nullptr if does not
   * exist
   */
  AttribsPtr removeTemplateInternal(const std::string& templateHandle,
                                    const std::string& src);

  /**
   * @brief This method will perform any necessary updating that is
   * attributesManager-specific upon template removal, such as removing a
   * specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called internally.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes to remove.
   */
  virtual void updateTemplateHandleLists(int templateID,
                                         const std::string& templateHandle) = 0;

  /**
   * @brief Used Internally. Get the ID of the template in @ref templateLibrary_
   * for the given template Handle, if exists. If the template is not in the
   * library and getNext is true then returns next available id, otherwise
   * throws assertion and returns ID_UNDEFINED
   *
   * @param templateHandle The string key referencing the template in @ref
   * templateLibrary_. Usually the origin handle.
   * @param getNext Whether to get the next available ID if not found, or to
   * throw an assertion. Defaults to false
   * @return The template's ID if found. The next available ID if not found and
   * getNext is true. Otherwise ID_UNDEFINED.
   */
  int getTemplateIDByHandleOrNew(const std::string& templateHandle,
                                 bool getNext) {
    if (getTemplateLibHasHandle(templateHandle)) {
      return templateLibrary_.at(templateHandle)->getID();
    } else {
      if (!getNext) {
        LOG(ERROR) << "AttributesManager::getTemplateIDByHandle : No "
                   << attrType_ << " template with handle " << templateHandle
                   << "exists. Aborting";
        return ID_UNDEFINED;
      } else {
        // return next available template ID (from previously deleted template)
        // or template library size as next ID to be used
        if (availableTemplateIDs_.size() > 0) {
          int retVal = availableTemplateIDs_.front();
          availableTemplateIDs_.pop_front();
          return retVal;
        }
        return templateLibrary_.size();
      }
    }
  }  // AttributesManager::getTemplateIDByHandle

  /**
   * @brief implementation of attributes type-specific registration
   * @param attributesTemplate the attributes template to be registered
   * @param attributesTemplateHandle the name to register the template with.
   * Expected to be valid.
   * @return The unique ID of the template being registered, or ID_UNDEFINED if
   * failed
   */
  virtual int registerAttributesTemplateFinalize(
      AttribsPtr attributesTemplate,
      const std::string& attributesTemplateHandle) = 0;

  /**
   * @brief Used Internally. Checks if template handle exists in map; if not
   * prints an error message, returns false; Otherwise returns true;
   */
  bool checkExistsWithMessage(const std::string& templateHandle,
                              const std::string& src) const {
    if (!getTemplateLibHasHandle(templateHandle)) {
      LOG(ERROR) << src << " : Unknown " << attrType_
                 << " template handle :" << templateHandle << ". Aborting";
      return false;
    }
    return true;
  }  // AttributesManager::checkExistsWithMessage

  /**
   * @brief Any implementation-specific resetting that needs to happen on reset.
   */
  virtual void resetFinalize() = 0;

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type.
   * @tparam U Type of attributes being created - must be a derived class of
   * AttribsPtr
   * @param orig original object of type AttribsPtr being copied
   */
  template <typename U>
  AttribsPtr createAttributesCopy(AttribsPtr& orig) {
    // don't call init on copy - assume copy is already properly initialized.
    return U::create(*(static_cast<U*>(orig.get())));
  }  // AttributesManager::

  /**
   * @brief This function will build the appropriate function pointer map for
   * this object's attributes, keyed on its attributes' class type.
   */
  virtual void buildCtorFuncPtrMaps() = 0;

  /**
   * @brief Build an @ref Attributes object of type associated
   * with passed object.
   * @param origAttr The original attributes object to copy
   */
  AttribsPtr copyAttributes(AttribsPtr& origAttr) {
    const std::string ctorKey = origAttr->getClassKey();
    return (*this.*(this->copyConstructorMap_[ctorKey]))(origAttr);
  }  // AttributesManager::copyAttributes

  /**
   * @brief add passed template to library, setting attributesTemplateID
   * appropriately. Called internally by registerTemplate.
   *
   * @param attributesTemplate the template to add to the library
   * @param attributesHandle the origin handle/name of the template to add. The
   * origin handle of the attributes template will be set to this here, in case
   * attributes is constructed with a different handle.
   * @return the attributesTemplateID of the template
   */
  int addTemplateToLibrary(AttribsPtr attributesTemplate,
                           const std::string& attributesHandle) {
    // set handle for template - might not have been set during construction
    attributesTemplate->setHandle(attributesHandle);
    // return either the ID of the existing template referenced by
    // attributesHandle, or the next available ID if not found.
    int attributesTemplateID =
        getTemplateIDByHandleOrNew(attributesHandle, true);
    attributesTemplate->setID(attributesTemplateID);
    // make a copy of this attributes so that user can continue to edit original
    AttribsPtr attributesTemplateCopy = copyAttributes(attributesTemplate);
    // add to libraries
    templateLibrary_[attributesHandle] = attributesTemplateCopy;
    templateLibKeyByID_.emplace(attributesTemplateID, attributesHandle);
    return attributesTemplateID;
  }  // AttributesManager::addTemplateToLibrary

  /**
   * @brief Return a random handle selected from the passed map
   *
   * @param mapOfHandles map containing the desired attribute-type template
   * handles
   * @param type the type of attributes being retrieved, for debug message
   * @return a random template handle of the chosen type, or the empty
   * string if none loaded
   */
  std::string getRandomTemplateHandlePerType(
      const std::map<int, std::string>& mapOfHandles,
      const std::string& type) const;

  /**
   * @brief Get a list of all templates of passed type whose origin handles
   * contain @ref subStr, ignoring subStr's case
   * @param mapOfHandles map containing the desired object-type template
   * handles
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains Whether to search for handles containing, or not
   * containting, @ref subStr
   * @return vector of 0 or more template handles containing/not containing
   * the passed substring
   */
  std::vector<std::string> getTemplateHandlesBySubStringPerType(
      const std::map<int, std::string>& mapOfHandles,
      const std::string& subStr,
      bool contains) const;

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createAttributesCopy keyed by string names of classes being instanced,
   * (for asset attributes these are the names of of classes being instanced
   * defined in @ref AssetAttributesManager::PrimNames3D.)
   */
  typedef std::map<std::string,
                   AttribsPtr (AttributesManager<AttribsPtr>::*)(AttribsPtr&)>
      Map_Of_CopyCtors;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_CopyCtors copyConstructorMap_;

  /** @brief A reference to a @ref esp::assets::ResourceManager which holds
   * assets that can be accessed by this @ref PhysicsManager*/
  assets::ResourceManager& resourceManager_;

  /** @brief A descriptive name of the attributes being managed by this manager.
   */
  const std::string attrType_;

  /**
   * @brief Maps string keys to attributes templates
   */
  std::map<std::string, AttribsPtr> templateLibrary_;

  /**
   * @brief Maps all object attribute IDs to the appropriate handles used
   * by lib
   */
  std::map<int, std::string> templateLibKeyByID_;

  /**
   * @brief Deque holding all IDs of deleted objects. These ID's should be
   * recycled before using map-size-based IDs
   */
  std::deque<int> availableTemplateIDs_;

  /**
   * @brief set holding string template handles of all system-locked templates,
   * to make sure they are never deleted.  Should not be overridden by user.
   */
  std::set<std::string> undeletableTemplateNames_;

  /**
   * @brief set holding string template handles of all user-locked
   * templates, to make sure they are not deleted unless the user
   * unlocks them.
   */
  std::set<std::string> userLockedTemplateNames_;

 public:
  ESP_SMART_POINTERS(AttributesManager<AttribsPtr>)

};  // namespace managers

/////////////////////////////
// Class Template Method Definitions

template <class AttribsPtr>
template <class U>
AttribsPtr AttributesManager<AttribsPtr>::createObjectAttributesFromJson(
    const std::string& configFilename,
    const io::JsonDocument& jsonDoc) {
  auto attributes = initNewAttribsInternal(U::create(configFilename));

  using std::placeholders::_1;

  // scale
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "scale",
      std::bind(&Attrs::AbstractObjectAttributes::setScale, attributes, _1));

  // margin
  io::jsonIntoSetter<double>(
      jsonDoc, "margin",
      std::bind(&Attrs::AbstractObjectAttributes::setMargin, attributes, _1));

  // load the friction coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "friction coefficient",
      std::bind(&Attrs::AbstractObjectAttributes::setFrictionCoefficient,
                attributes, _1));

  // load the restitution coefficient
  io::jsonIntoSetter<double>(
      jsonDoc, "restitution coefficient",
      std::bind(&Attrs::AbstractObjectAttributes::setRestitutionCoefficient,
                attributes, _1));

  // if object will be flat or phong shaded
  io::jsonIntoSetter<bool>(
      jsonDoc, "requires lighting",
      std::bind(&Attrs::AbstractObjectAttributes::setRequiresLighting,
                attributes, _1));

  // units to meters
  io::jsonIntoSetter<double>(
      jsonDoc, "units to meters",
      std::bind(&Attrs::AbstractObjectAttributes::setUnitsToMeters, attributes,
                _1));

  // load object/scene specific up orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "up",
      std::bind(&Attrs::AbstractObjectAttributes::setOrientUp, attributes, _1));

  // load object/scene specific front orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonDoc, "front",
      std::bind(&Attrs::AbstractObjectAttributes::setOrientFront, attributes,
                _1));

  // 4. parse render and collision mesh filepaths
  std::string rndrFName = "";
  std::string rTmpFName = attributes->getRenderAssetHandle();
  if (setJSONAssetHandleAndType(
          attributes, jsonDoc, "render mesh type", "render mesh", rTmpFName,
          std::bind(&Attrs::AbstractObjectAttributes::setRenderAssetType,
                    attributes, _1))) {
    rndrFName = rTmpFName;
  }

  std::string colFName = "";
  std::string cTmpFName = attributes->getCollisionAssetHandle();
  if (setJSONAssetHandleAndType(
          attributes, jsonDoc, "collision mesh type", "collision mesh",
          cTmpFName,
          std::bind(&Attrs::AbstractObjectAttributes::setCollisionAssetType,
                    attributes, _1))) {
    colFName = cTmpFName;
    // TODO eventually remove this, but currently collision mesh must be UNKNOWN
    attributes->setCollisionAssetType(static_cast<int>(AssetType::UNKNOWN));
  }

  // use non-empty result if either result is empty
  attributes->setRenderAssetHandle(rndrFName.compare("") == 0 ? colFName
                                                              : rndrFName);
  attributes->setCollisionAssetHandle(colFName.compare("") == 0 ? rndrFName
                                                                : colFName);
  attributes->setUseMeshCollision(true);

  return attributes;
}  // AttributesManager<AttribsPtr>::createObjectAttributesFromJson

template <class T>
bool AttributesManager<T>::setJSONAssetHandleAndType(
    T attributes,
    const io::JsonDocument& jsonDoc,
    const char* jsonMeshTypeTag,
    const char* jsonMeshHandleTag,
    std::string& fileName,
    std::function<void(int)> meshTypeSetter) {
  std::string propertiesFileDirectory = attributes->getFileDirectory();
  // save current file name
  const std::string oldFName(fileName);
  // clear var to get new value
  fileName = "";
  // Map a json string value to its corresponding AssetType if found and cast to
  // int, based on @ref AbstractObjectAttributes::AssetTypeNamesMap mappings.
  // Casts an int of the @ref esp::AssetType enum value if found and understood,
  // 0 (AssetType::UNKNOWN) if found but not understood, and
  //-1 if tag is not found in json.
  int typeVal = -1;
  std::string tmpVal = "";
  if (io::jsonIntoVal<std::string>(jsonDoc, jsonMeshTypeTag, tmpVal)) {
    // tag was found, perform check
    std::string strToLookFor = Cr::Utility::String::lowercase(tmpVal);
    if (Attrs::AbstractObjectAttributes::AssetTypeNamesMap.count(tmpVal)) {
      typeVal = static_cast<int>(
          Attrs::AbstractObjectAttributes::AssetTypeNamesMap.at(tmpVal));
    } else {
      LOG(WARNING) << "AttributesManager::convertJsonStringToAssetType : "
                      "Value in json @ tag : "
                   << jsonMeshTypeTag << " : `" << tmpVal
                   << "` does not map to a valid "
                      "AbstractObjectAttributes::AssetTypeNamesMap value, so "
                      "defaulting mesh type to AssetType::UNKNOWN.";
      typeVal = static_cast<int>(AssetType::UNKNOWN);
    }
    // value found so override current value, otherwise do not.
    meshTypeSetter(typeVal);
  }  // if type is found in json.  If not typeVal is -1

  // Read json for new mesh handle
  if (io::jsonIntoVal<std::string>(jsonDoc, jsonMeshHandleTag, fileName)) {
    // value has changed
    fileName = Cr::Utility::Directory::join(propertiesFileDirectory, fileName);
    if ((typeVal == -1) && (oldFName.compare(fileName) != 0)) {
      std::string strToFind(jsonMeshTypeTag);
      // if file name is different, and type val has not been specified, perform
      // name-specific mesh type config
      // do not override orientation - should be specified in json.
      setDefaultFileNameBasedAttributes(attributes, false, fileName,
                                        meshTypeSetter);
    }
    return true;
  }
  return false;
}  // AttributesManager<AttribsPtr>::setAssetHandleAndType

template <class AttribsPtr>
bool AttributesManager<AttribsPtr>::setTemplateLock(
    const std::string& templateHandle,
    bool lock) {
  // if template does not currently exist then do not attempt to modify its
  // lock state
  if (!checkExistsWithMessage(templateHandle,
                              "AttributesManager::setTemplateLock")) {
    return false;
  }
  // if setting lock else clearing lock
  if (lock) {
    userLockedTemplateNames_.insert(templateHandle);
  } else if (userLockedTemplateNames_.count(templateHandle) > 0) {
    // if clearing, verify exists
    userLockedTemplateNames_.erase(templateHandle);
  }
  return true;
}  // AttributesManager::setTemplateLock

template <class AttribsPtr>
std::vector<AttribsPtr>
AttributesManager<AttribsPtr>::removeTemplatesBySubstring(
    const std::string& subStr,
    bool contains) {
  std::vector<AttribsPtr> res;
  // get all handles that match query elements first
  std::vector<std::string> handles =
      getTemplateHandlesBySubstring(subStr, contains);
  for (std::string templateHandle : handles) {
    AttribsPtr ptr = removeTemplateInternal(
        templateHandle, "AttributesManager::removeTemplatesBySubstring");
    if (nullptr != ptr) {
      res.push_back(ptr);
    }
  }
  return res;
}  // removeAllTemplates

template <class T>
T AttributesManager<T>::removeTemplateInternal(
    const std::string& templateHandle,
    const std::string& sourceStr) {
  if (!checkExistsWithMessage(templateHandle, sourceStr)) {
    LOG(INFO) << sourceStr << " : Unable to remove " << attrType_
              << " template " << templateHandle << " : Does not exist.";
    return nullptr;
  }

  T attribsTemplate = getTemplateCopyByHandle(templateHandle);
  std::string msg;
  if (this->undeletableTemplateNames_.count(templateHandle) > 0) {
    msg = "Required Undeletable Template";
  } else if (this->userLockedTemplateNames_.count(templateHandle) > 0) {
    msg = "User-locked Template.  To delete template, unlock it.";
  }
  if (msg.length() != 0) {
    LOG(INFO) << sourceStr << " : Unable to remove " << attrType_
              << " template " << templateHandle << " : " << msg << ".";
    return nullptr;
  }
  int templateID = attribsTemplate->getID();
  templateLibKeyByID_.erase(templateID);
  templateLibrary_.erase(templateHandle);
  availableTemplateIDs_.emplace_front(templateID);
  // call instance-specific update to remove template handle from any local
  // lists
  updateTemplateHandleLists(templateID, templateHandle);
  return attribsTemplate;
}  // AttributesManager::removeTemplateByHandle

template <class T>
std::string AttributesManager<T>::getRandomTemplateHandlePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  std::size_t numVals = mapOfHandles.size();
  if (numVals == 0) {
    LOG(ERROR) << "Attempting to get a random " << type << attrType_
               << " template handle but none are loaded; Aboring";
    return "";
  }
  int randIDX = rand() % numVals;

  std::string res;
  for (std::pair<std::map<int, std::string>::const_iterator, int> iter(
           mapOfHandles.begin(), 0);
       (iter.first != mapOfHandles.end() && iter.second <= randIDX);
       ++iter.first, ++iter.second) {
    res = iter.first->second;
  }
  return res;
}  // AttributesManager::getRandomTemplateHandlePerType

template <class T>
std::vector<std::string>
AttributesManager<T>::getTemplateHandlesBySubStringPerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& subStr,
    bool contains) const {
  std::vector<std::string> res;
  // if empty return empty vector
  if (mapOfHandles.size() == 0) {
    return res;
  }
  // if search string is empty, return all values
  if (subStr.length() == 0) {
    for (auto elem : mapOfHandles) {
      res.push_back(elem.second);
    }
    return res;
  }
  // build search criteria
  std::string strToLookFor = Cr::Utility::String::lowercase(subStr);

  std::size_t strSize = strToLookFor.length();

  for (std::map<int, std::string>::const_iterator iter = mapOfHandles.begin();
       iter != mapOfHandles.end(); ++iter) {
    std::string key = Cr::Utility::String::lowercase(iter->second);
    // be sure that key is big enough to search in (otherwise find has undefined
    // behavior)
    if (key.length() < strSize) {
      continue;
    }
    bool found = (std::string::npos != key.find(strToLookFor));
    if (found == contains) {
      // if found and searching for contains, or not found and searching for not
      // contains
      res.push_back(iter->second);
    }
  }
  return res;
}  // AttributesManager::getTemplateHandlesBySubStringPerType

}  // namespace managers
}  // namespace assets
}  // namespace esp
#endif  // ESP_ASSETS_MANAGERS_ATTRIBUTESMANAGERBASE_H_
