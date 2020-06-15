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
  //   //======== Accessor functions ========
  /**
   * @brief clears maps of handle-keyed attributes and ID-keyed handles.
   */
  void reset() {
    templateLibKeyByID_.clear();
    templateLibrary_.clear();
    availableTemplateIDs_.clear();
    resetFinalize();
  }  // AttributesManager::reset

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
  }

  /**
   * @brief Get a reference to the attributes template for the asset
   * identified by the objectTemplateID.
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param objectTemplateID The ID of the template.  Is mapped to the key
   * referencing the asset in @ref templateLibrary_ by @ref templateLibKeyByID_.
   * @return A mutable reference to the object template, or nullptr if does not
   * exist
   */
  const AttribsPtr getTemplateByID(int objectTemplateID) const {
    std::string templateHandle = getTemplateHandleByID(objectTemplateID);
    if (!checkExistsWithMessage(templateHandle, "getTemplateByID")) {
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
   * @return A mutable reference to the object template, or nullptr if does not
   * exist
   */
  const AttribsPtr getTemplateByHandle(
      const std::string& templateHandle) const {
    if (!checkExistsWithMessage(templateHandle, "getTemplateByHandle")) {
      return nullptr;
    }
    return templateLibrary_.at(templateHandle);
  }  // AttributesManager::getAttributesTemplate

  /**
   * @brief Return a reference to a copy of the object specified
   * by passed handle.  This is the version that should be accessed by the
   * user.
   * @param templateHandle the string key of the attributes desired.
   * @return a copy of the desired attributes, or nullptr if does
   * not exist
   */
  AttribsPtr getTemplateCopyByHandle(const std::string& templateHandle) {
    if (!checkExistsWithMessage(templateHandle, "getTemplateCopyByHandle")) {
      return nullptr;
    }
    auto orig = this->templateLibrary_.at(templateHandle);
    return this->copyAttributes(orig);
  }  // AttributesManager::getTemplateCopyByHandle

  /**
   * @brief Remove the template referenced by the passed string handle.  Will
   * enmplace template ID within deque of usable IDs and return the template
   * being removed.
   * @param templateHandle the string key of the attributes desired.
   * @return the desired attributes being deleted, or nullptr if does not exist
   */
  AttribsPtr removeTemplateByHandle(const std::string& templateHandle) {
    if (!checkExistsWithMessage(templateHandle, "removeTemplateByHandle")) {
      LOG(INFO) << "AttributesManager::removeTemplateByHandle : Unable to "
                   "remove default template :"
                << templateHandle << ". Aborting";
      return nullptr;
    }
    if (isTemplateReadOnly(templateHandle)) {
    }
    AttribsPtr attribsTemplate = templateLibrary_.at(templateHandle);
    int templateID = attribsTemplate->getObjectTemplateID();
    templateLibKeyByID_.erase(templateID);
    templateLibrary_.erase(templateHandle);
    availableTemplateIDs_.emplace_front(templateID);
    return attribsTemplate;
  }  // AttributesManager::removeTemplateByHandle

  /**
   * @brief Get the key in @ref templateLibrary_ for the object
   * template index.
   *
   * @param templateID The index of the template in @ref templateLibrary_.
   * @return The key referencing the template in @ref
   * templateLibrary_, or an empty string if does not exist.
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
   * template Handle, if exists.  If getNext is true then returns size of
   * library, otherwise throws assertion and returns ID_UNDEFINED
   *
   * @param templateHandle The string key referencing the template in @ref
   * templateLibrary_.  Usually the origin handle.
   * @param getNext Whether to get the next available ID if not found, or to
   * throw an assertion.  Defaults to false
   * @return The key referencing the template in @ref
   * templateLibrary_, or an empty string if does not exist.
   */
  int getTemplateIDByHandle(const std::string& templateHandle,
                            bool getNext = false) {
    if (getTemplateLibHasHandle(templateHandle)) {
      return templateLibrary_.at(templateHandle)->getObjectTemplateID();
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
   * @brief Get a random object attribute handle (that could possibly describe
   * either file-based or a primitive) for the loaded file-based object
   * templates
   *
   * @return a randomly selected handle corresponding to a known object
   * attributes template, or empty string if none found
   */
  std::string getRandomTemplateHandle() const {
    return getRandomTemplateHandlePerType(templateLibKeyByID_, "");
  }

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
  }

  /**
   * @brief return a read-only reference to the template library managed by this
   * object.
   */
  const std::map<std::string, AttribsPtr>& getTemplateLibrary() const {
    return templateLibrary_;
  }

 protected:
  /**
   * @brief Used Internally.  Checks if template handle exists in map; if not
   * prints an error message, returns false; Otherwise returns true;
   */
  bool checkExistsWithMessage(const std::string& templateHandle,
                              const std::string& src) const {
    if (!getTemplateLibHasHandle(templateHandle)) {
      LOG(ERROR) << "AttributesManager::" << src
                 << " : Unknown template handle :" << templateHandle
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
   * deleted.  Do not wish to remove certain default templates, such as
   * primitive asset templates.
   * @param templateHandle the handle to the template to verify removability.
   * Assumes template exists.
   * @return Whether the template is read-only or not
   */
  virtual bool isTemplateReadOnly(const std::string& templateHandle) = 0;

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type.
   * @tparam T Type of attributes being created - must be a derived class of
   * AttribsPtr
   * @param orig original object of type AttribsPtr being copied
   */
  template <typename T>
  const AttribsPtr createAttributesCopy(const AttribsPtr& orig) {
    return T::create(*(static_cast<T*>(orig.get())));
  }

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
  const AttribsPtr copyAttributes(const AttribsPtr& origAttr) {
    const std::string ctorKey = origAttr->getClassKey();
    return (*this.*(this->copyConstructorMap_[ctorKey]))(origAttr);
  }  // copyAttributes

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createAttributesCopy keyed by string names of classes being instanced,
   * (for asset attributes these are the names of of classes being instanced
   * defined in @ref AssetAttributesManager::PrimNames3D.)
   */
  typedef std::map<std::string,
                   const AttribsPtr (AttributesManager<AttribsPtr>::*)(
                       const AttribsPtr&)>
      Map_Of_CopyCtors;

  /**
   * @brief Map of function pointers to instantiate a primitive attributes
   * object, keyed by the Magnum primitive class name as listed in @ref
   * PrimNames3D. A primitive attributes object is instanced by accessing
   * the approrpiate function pointer.
   */
  Map_Of_CopyCtors copyConstructorMap_;

  /**
   * @brief add passed template to library, setting objectTemplateID
   * appropriately.  Called internally by registerTemplate.
   *
   * @param attributesTemplate the template to add to the library
   * @param attributesHandle the origin handle/name of the template to add
   * @return the objectTemplateID of the template
   */

  int addTemplateToLibrary(
      const AttribsPtr attributesTemplate,
      const std::string& attributesHandle) {  // return either the ID of the
                                              // existing template referenced by
    // attributesHandle, or the next available ID if not found.
    int attributesTemplateID = getTemplateIDByHandle(attributesHandle, true);
    attributesTemplate->setObjectTemplateID(attributesTemplateID);
    templateLibrary_[attributesHandle] = attributesTemplate;
    templateLibKeyByID_.emplace(attributesTemplateID, attributesHandle);
    return attributesTemplateID;
  }

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

  //   // ======== Instance Variables ========

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
   * @brief Deque holding all IDs of deleted objects.  These ID's should be
   * recycled before using map-size-based IDs
   */
  std::deque<int> availableTemplateIDs_;

 public:
  ESP_SMART_POINTERS(AttributesManager<AttribsPtr>)

};  // class AttributesManager

/////////////////////////////
// Class Template Definitions

template <typename T>
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
}  // getRandomTemplateHandlePerType

template <typename T>
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
}  // getTemplateHandlesBySubStringPerType

}  // namespace managers
}  // namespace assets
}  // namespace esp
#endif  // ESP_ASSETS_MANAGERS_ATTRIBUTESMANAGERBASE_H_