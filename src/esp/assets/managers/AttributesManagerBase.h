// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_
#define ESP_ASSETS_MANAGERS_ATTRIBUTEMANAGERBASE_H_

/** @file
 * @brief Class Template @ref esp::assets::AttributesManager
 */

#include <map>

#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/ConfigurationGroup.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>
#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/assets/Attributes.h"

namespace esp {
namespace assets {

namespace managers {

/**
 * @brief Template Class defining responsibilities for managing attributes for
 * different types of objects, such as scenes, primitive assets, physical
 * objects, etc.
 * @tparam T the type of attributes object this class references
 */

template <class T>
class AttributesManager {
 public:
  AttributesManager() {}
  ~AttributesManager() {}

  //   //======== Accessor functions ========

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
   * referencing the asset in @ref templateLibrary_ by @ref
   templateLibKeyByID_.
   * @return A mutable reference to the object template, or nullptr if does
   not
   * exist
   */
  std::shared_ptr<T> getAttributesTemplate(const int objectTemplateID) const {
    std::string key = getTemplateHandleByID(objectTemplateID);
    CORRADE_ASSERT(getTemplateLibHasHandle(key),
                   "AttributesManager::getAttributesTemplate : Unknown "
                   "object template ID:"
                       << objectTemplateID << ". Aborting",
                   nullptr);
    return templateLibrary_.at(key);
  }  // getAttributesTemplate

  /**
   * @brief Get a reference to the attributes template for the asset
   * identified by the passed templateHandle.
   *
   * Can be used to manipulate a template before instancing new objects.
   * @param templateHandle The key referencing the asset in @ref
   * templateLibrary_.
   * @return A mutable reference to the object template, or nullptr if does
   not
   * exist
   */
  std::shared_ptr<T> getAttributesTemplate(
      const std::string& templateHandle) const {
    CORRADE_ASSERT(getTemplateLibHasHandle(templateHandle),
                   "AttributesManager::getAttributesTemplate : Unknown "
                   "object template Handle:"
                       << templateHandle << ". Aborting",
                   nullptr);
    return templateLibrary_.at(templateHandle);
  }  // getAttributesTemplate

  /**
   * @brief Get the key in @ref templateLibrary_ for the object
   * template index.
   *
   * @param templateID The index of the template in @ref templateLibrary_.
   * @return The key referencing the template in @ref
   * templateLibrary_, or an empty string if does not exist.
   */
  std::string getTemplateHandleByID(const int templateID) const {
    CORRADE_ASSERT(templateLibKeyByID_.count(templateID) > 0,
                   "AttributesManager::getTemplateHandleByID : Unknown "
                   "object template ID:"
                       << templateID << ". Aborting",
                   nullptr);
    return templateLibKeyByID_.at(templateID);
  }  // getTemplateHandleByID

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
                            bool getNext = false) const {
    if (getTemplateLibHasHandle(templateHandle)) {
      return templateLibrary_.at(templateHandle)->getObjectTemplateID();
    } else {
      if (!getNext) {
        CORRADE_ASSERT(false,
                       "AttributesManager::getTemplateIDByHandle : No template "
                       " with handle "
                           << templateHandle << "exists. Aborting",
                       ID_UNDEFINED);

      } else {
        // TODO : This needs to return an unused ID number, to support deleting
        // templates
        return templateLibrary_.size();
      }
    }
  }  // getTemplateIDByHandle

  /**
   * @brief Get a random object attribute handle (that could possibly
   describe
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
   * @param contains whether to search for keys containing, or not
   containing,
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

  // protected:
  /**
   * @brief add passed template to library, setting objectTemplateID
   * appropriately.  Called internally by registerTemplate.
   *
   * @param attributesTemplate the template to add to the library
   * @param attributesHandle the origin handle/name of the template to add
   * @return the objectTemplateID of the template
   */

  int addTemplateToLibrary(
      std::shared_ptr<T> attributesTemplate,
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
   the
   * passed substring
   */
  std::vector<std::string> getTemplateHandlesBySubStringPerType(
      const std::map<int, std::string>& mapOfHandles,
      const std::string& subStr,
      bool contains) const;

  //   // ======== Instance Variables ========

  /**
   * @brief Maps string keys to attributes templates
   */
  std::map<std::string, std::shared_ptr<T>> templateLibrary_;

  /**
   * @brief Maps all object attribute IDs to the appropriate handles used
   by
   * lib
   */
  std::map<int, std::string> templateLibKeyByID_;

 public:
  ESP_SMART_POINTERS(AttributesManager)

};  // class AttributesManager

/////////////////////////////
// Class Template Definitions
// template <typename T>
// AttributesManager<T>::AttributesManager() {}

// template <typename T>
// AttributesManager<T>::~AttributesManager() {}

template <typename T>
std::string AttributesManager<T>::getRandomTemplateHandlePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  int numVals = mapOfHandles.size();
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
  std::string strToLookFor(subStr);

  int strSize = strToLookFor.length();
  // force lowercase
  std::transform(strToLookFor.begin(), strToLookFor.end(), strToLookFor.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  for (std::map<int, std::string>::const_iterator iter = mapOfHandles.begin();
       iter != mapOfHandles.end(); ++iter) {
    std::string key(iter->second);
    // be sure that key is big enough to search in (otherwise find has undefined
    // behavior)
    if (key.length() < strSize) {
      continue;
    }
    // force lowercase - search is case insensitive
    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return std::tolower(c); });
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