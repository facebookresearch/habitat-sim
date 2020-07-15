// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_
#define ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_

/** @file
 * @brief Class Template @ref esp::assets::AttributesManager
 */

#include <deque>
#include <map>

#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/assets/Attributes.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {

class ResourceManager;
namespace managers {

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
  AttributesManager(assets::ResourceManager& resourceManager)
      : resourceManager_(resourceManager) {}
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
   * @brief Add a copy of @ref AbstractAttributes object to the @ref
   * templateLibrary_.
   *
   * @param attributesTemplate The attributes template.
   * @param attributesTemplateHandle The key for referencing the template in the
   * @ref templateLibrary_. Will be set as origin handle for template. If empty
   * string, use existing origin handle.
   * @return The unique ID of the template being registered, or ID_UNDEFINED if
   * failed
   */
  int registerAttributesTemplate(
      AttribsPtr attributesTemplate,
      const std::string& attributesTemplateHandle = "") {
    if ("" != attributesTemplateHandle) {
      return registerAttributesTemplateFinalize(attributesTemplate,
                                                attributesTemplateHandle);
    }
    std::string handleToSet = attributesTemplate->getHandle();
    if ("" == handleToSet) {
      LOG(ERROR) << "AttributesManager::registerAttributesTemplate : No "
                    "valid handle specified for attributes template to "
                    "register. Aborting.";
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
  }  // isValidFileName

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
   * attributesTemplateID.
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
   * identified by the passed templateHandle.
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
    return _removeTemplateInternal(templateHandle,
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
    return _removeTemplateInternal(templateHandle,
                                   "AttributesManager::removeTemplateByHandle");
  }
  /**
   * @brief Get the key in @ref templateLibrary_ for the object template with
   * the given unique ID.
   *
   * @param templateID The unique ID of the desired template.
   * @return The key referencing the template in @ref
   * templateLibrary_, or nullptr if does not exist.
   */
  std::string getTemplateHandleByID(const int templateID) const {
    if (templateLibKeyByID_.count(templateID) == 0) {
      LOG(ERROR) << "AttributesManager::getTemplateHandleByID : Unknown "
                    "object template ID:"
                 << templateID << ". Aborting";
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
    return _getTemplateIDByHandleOrNew(templateHandle, false);
  }  // AttributesManager::getTemplateIDByHandle

  /**
   * @brief Get a list of all templates whose origin handles contain @ref
   * subStr, ignoring subStr's case
   * @param subStr substring to search for within existing primitive object
   * templates
   * @param contains whether to search for keys containing, or excluding,
   * @ref subStr
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
   * @brief return a read-only reference to the template library managed by this
   * object.
   */
  const std::map<std::string, AttribsPtr>& getTemplateLibrary() const {
    return templateLibrary_;
  }

 protected:
  /**
   * @brief Used Internally. Remove the template referenced by the passed
   * string handle. Will emplace template ID within deque of usable IDs and
   * return the template being removed.
   * @param templateHandle the string key of the attributes desired.
   * @param src String denoting the source of the remove request.
   * @return the desired attributes being deleted, or nullptr if does not exist
   */
  AttribsPtr _removeTemplateInternal(const std::string& templateHandle,
                                     const std::string& src);
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
  int _getTemplateIDByHandleOrNew(const std::string& templateHandle,
                                  bool getNext) {
    if (getTemplateLibHasHandle(templateHandle)) {
      return templateLibrary_.at(templateHandle)->getID();
    } else {
      if (!getNext) {
        LOG(ERROR) << "AttributesManager::getTemplateIDByHandle : No template "
                      " with handle "
                   << templateHandle << "exists. Aborting";
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
      LOG(ERROR) << src << " : Unknown template handle :" << templateHandle
                 << ". Aborting";
      return false;
    }
    return true;
  }  // AttributesManager::checkExistsWithMessage

  /**
   * @brief Any implementation-specific resetting that needs to happen on reset.
   */
  virtual void resetFinalize() = 0;

  /**
   * @brief Whether template described by passed handle is read only, or can be
   * deleted. Do not wish to remove certain default templates, such as
   * primitive asset templates.
   * @param templateHandle the handle to the template to verify removability.
   * Assumes template exists.
   * @return Whether the template is read-only or not
   */
  virtual bool isTemplateReadOnly(const std::string& templateHandle) = 0;

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type.
   * @tparam U Type of attributes being created - must be a derived class of
   * AttribsPtr
   * @param orig original object of type AttribsPtr being copied
   */
  template <typename U>
  AttribsPtr createAttributesCopy(AttribsPtr& orig) {
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
        _getTemplateIDByHandleOrNew(attributesHandle, true);
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

 public:
  ESP_SMART_POINTERS(AttributesManager<AttribsPtr>)

};  // class AttributesManager

/////////////////////////////
// Class Template Method Definitions

template <class T>
T AttributesManager<T>::_removeTemplateInternal(
    const std::string& templateHandle,
    const std::string& sourceStr) {
  std::string msg;
  if (!checkExistsWithMessage(templateHandle, sourceStr)) {
    msg = "Does not exist";
  } else if (isTemplateReadOnly(templateHandle)) {
    msg = "Required Default Template";
  }
  if (msg.length() != 0) {
    LOG(INFO) << sourceStr << " : Unable to remove template " << templateHandle
              << " : " << msg << ".";
    return nullptr;
  }

  T attribsTemplate = getTemplateCopyByHandle(templateHandle);
  int templateID = attribsTemplate->getID();
  templateLibKeyByID_.erase(templateID);
  templateLibrary_.erase(templateHandle);
  availableTemplateIDs_.emplace_front(templateID);
  return attribsTemplate;
}  // AttributesManager::removeTemplateByHandle

template <class T>
std::string AttributesManager<T>::getRandomTemplateHandlePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  std::size_t numVals = mapOfHandles.size();
  if (numVals == 0) {
    LOG(ERROR) << "Attempting to get a random " << type
               << "object template handle but none are loaded; Aboring";
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