// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MANAGEDCONTAINERBASE_H_
#define ESP_CORE_MANAGEDCONTAINERBASE_H_

/** @file
 * @brief Abstract Class @ref esp::core::ManagedContainerBase : type-independent
 * container functionality consumed by @ref esp::core::ManagedContainer to cut
 * down on code bload.
 */

#include <deque>
#include <functional>
#include <map>
#include <set>

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/core/AbstractManagedObject.h"

#include "esp/io/io.h"
#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace core {

/**
 * @brief Base class of Managed Container, holding template-type-independent
 * functionality
 */
class ManagedContainerBase {
 public:
  explicit ManagedContainerBase(const std::string& metadataType)
      : objectType_(metadataType) {}
  virtual ~ManagedContainerBase() = default;
  /**
   * @brief Utility function to check if passed string represents an existing,
   * user-accessible file
   * @param handle the string to check
   * @return whether the file exists in the file system and whether the user has
   * access
   */
  bool isValidFileName(const std::string& handle) const {
    return (Corrade::Utility::Directory::exists(handle));
  }  // ManagedContainerBase::isValidFileName

  /**
   * @brief Sets/Clears lock state for a managed object, preventing or allowing
   * deletion of managed object.
   * @param objectHandle handle of managed object to set state of
   * @param lock boolean to set or clear lock
   * @return whether successful or not.
   */
  bool setLock(const std::string& objectHandle, bool lock);

  /**
   * @brief Sets/Clears lock state for a managed object, preventing or allowing
   * deletion of managed object.
   * @param objectHandles Vector of handles of managed objects to set state of
   * @param lock boolean to set or clear lock for all managed objects
   * @return the list of handles actually set to desired lock state.
   */
  std::vector<std::string> setLockByHandles(
      const std::vector<std::string>& objectHandles,
      bool lock) {
    std::vector<std::string> res;
    for (const std::string& objectHandle : objectHandles) {
      if (setLock(objectHandle, lock)) {
        res.push_back(objectHandle);
      }
    }
    return res;
  }  // ManagedContainerBase::setLockByHandles

  /**
   * @brief set lock state on all managed objects that contain passed substring.
   * @param lock boolean to set or clear lock on managed objects
   * @param subStr substring to search for within existing primitive object
   * managed objects
   * @param contains whether to search for keys containing, or excluding,
   * substr
   * @return A vector containing the managed object handles of managed objects
   * whose lock state has been set to passed state.
   */
  std::vector<std::string> setLockBySubstring(bool lock,
                                              const std::string& subStr = "",
                                              bool contains = true) {
    std::vector<std::string> handles =
        getObjectHandlesBySubstring(subStr, contains);
    return this->setLockByHandles(handles, lock);
  }  // ManagedContainerBase::setLockBySubstring

  /**
   * @brief Get the handle for a random managed object registered to
   * this manager.
   *
   * @return a randomly selected handle corresponding to a known object
   * managed object, or empty string if none found
   */
  std::string getRandomObjectHandle() const {
    return getRandomObjectHandlePerType(objectLibKeyByID_, "");
  }  // ManagedContainerBase::getRandomObjectHandle

  /**
   * @brief Get a list of all managed objects whose origin handles contain
   * subStr, ignoring subStr's case
   * @param subStr substring to search for within existing managed objects.
   * @param contains whether to search for keys containing, or excluding,
   * passed subStr
   * @return vector of 0 or more managed object handles containing the passed
   * substring
   */
  std::vector<std::string> getObjectHandlesBySubstring(
      const std::string& subStr = "",
      bool contains = true) const {
    return getObjectHandlesBySubStringPerType(objectLibKeyByID_, subStr,
                                              contains);
  }  // ManagedContainerBase::getObjectHandlesBySubstring

  /**
   * @brief returns a vector of managed object handles representing the
   * system-specified undeletable managed objects this manager manages. These
   * managed objects cannot be deleted, although they can be edited.
   */
  std::vector<std::string> getUndeletableObjectHandles() const {
    std::vector<std::string> res(this->undeletableObjectNames_.begin(),
                                 this->undeletableObjectNames_.end());
    return res;
  }  // ManagedContainerBase::getUndeletableObjectHandles

  /**
   * @brief returns a vector of managed object handles representing managed
   * objects that have been locked by the user.  These managed objects cannot be
   * deleted until they have been unlocked, although they can be edited while
   * locked.
   */
  std::vector<std::string> getUserLockedObjectHandles() const {
    std::vector<std::string> res(this->userLockedObjectNames_.begin(),
                                 this->userLockedObjectNames_.end());
    return res;
  }  // ManagedContainerBase::getUserLockedObjectHandles

  /**
   * @brief clears maps of handle-keyed managed object and ID-keyed handles.
   */
  void reset() {
    objectLibKeyByID_.clear();
    objectLibrary_.clear();
    availableObjectIDs_.clear();
    resetFinalize();
  }  // ManagedContainerBase::reset

  // ======== Accessor functions ========
  /**
   * @brief Get the key in @ref objectLibrary_ for the object managed
   * object with the given unique ID.
   *
   * @param objectID The unique ID of the desired managed object.
   * @return The key referencing the managed object in @ref
   * objectLibrary_, or nullptr if does not exist.
   */
  std::string getObjectHandleByID(const int objectID) const {
    if (objectLibKeyByID_.count(objectID) == 0) {
      LOG(ERROR) << "ManagedContainerBase::getObjectHandleByID : Unknown "
                 << objectType_ << " managed object ID:" << objectID
                 << ". Aborting";
      // never will have registered object with registration handle == ""
      return "";
    }
    return objectLibKeyByID_.at(objectID);
  }  // ManagedContainer::getObjectHandleByID

  /**
   * @brief Gets the number of object managed objects stored in the @ref
   * objectLibrary_.
   *
   * @return The size of the @ref objectLibrary_.
   */
  int getNumObjects() const { return objectLibrary_.size(); }

  /**
   * @brief Checks whether managed object library has passed string handle as
   * key
   * @param handle the key to look for
   */
  bool getObjectLibHasHandle(const std::string& handle) const {
    return objectLibrary_.count(handle) > 0;
  }  // ManagedContainerBase::getObjectLibHasHandle

  /**
   * @brief Get the type of object this ManagedContainer manages.
   */
  const std::string& getObjectType() const { return objectType_; }

 protected:
  //======== Internally accessed getter/setter ================

  /**
   * @brief Retrieve shared pointer to object held in library, NOT a copy.
   * @param handle the name of the object held in the smart pointer
   */
  template <class U>
  auto getObjectInternal(const std::string& handle) const {
    return std::static_pointer_cast<U>(objectLibrary_.at(handle));
  }

  /**
   * @brief Only used from class template AddObject method.  put the passed
   * smart poitner in the library.
   * @param ptr the smart pointer to the object being managed
   * @param handle the name (key) to use for the object in the library
   */
  void setObjectInternal(const std::shared_ptr<void>& ptr,
                         const std::string& handle) {
    objectLibrary_[handle] = ptr;
  }

  //======== Common JSON import and utility functions ========

  /**
   * @brief Verify passd @p filename is legal document of type T. Returns loaded
   * document in passed argument if successful. This requires appropriate
   * specialization for each type name, so if this method is executed it means
   * no appropriate specialization exists for passed type of document.
   *
   * @tparam type of document
   * @param filename name of potentia document to load
   * @param resDoc a reference to the document to be parsed.
   * @return whether document has been loaded successfully or not
   */
  template <class U>
  bool verifyLoadDocument(const std::string& filename,
                          CORRADE_UNUSED U& resDoc) {
    // by here always fail
    LOG(ERROR) << objectType_
               << "ManagedContainerBase::verifyLoadDocument : File " << filename
               << " failed due to unknown file type.";
    return false;
  }  // ManagedContainerBase::verifyLoadDocument
  /**
   * @brief Verify passed @p filename is legal json document, return loaded
   * document or nullptr if fails
   *
   * @param filename name of potential json document to load
   * @param jsonDoc a reference to the json document to be parsed
   * @return whether document has been loaded successfully or not
   */
  bool verifyLoadDocument(const std::string& filename,
                          io::JsonDocument& jsonDoc);

  /**
   * @brief Will build a json file name for @p filename by appending/replacing
   * the extension with the passed @p jsonTypeExt, if it is missing.  NOTE :
   * this does not verify that file exists.
   * @param filename The original file name
   * @param jsonTypeExt The extension to use.
   * @return The file name changed so that it has the correct @p jsonTypeExtif
   * it was missing.
   */
  std::string convertFilenameToJSON(const std::string& filename,
                                    const std::string& jsonTypeExt);

  /**
   * @brief This method will perform any necessary updating that is
   * ManagedContainer specialization-specific upon managed object removal, such
   * as removing a specific managed object handle from the list of file-based
   * managed object handles in ObjectAttributesManager.  This should only be
   * called internally.
   *
   * @param objectID the ID of the managed object to remove
   * @param objectHandle the string key of the managed object to remove.
   */
  virtual void updateObjectHandleLists(int objectID,
                                       const std::string& objectHandle) = 0;

  /**
   * @brief Used Internally. Checks if managed object handle exists in map; if
   * not prints an error message, returns false; Otherwise returns true;
   */
  bool checkExistsWithMessage(const std::string& objectHandle,
                              const std::string& src) const {
    if (!getObjectLibHasHandle(objectHandle)) {
      LOG(ERROR) << src << " : Unknown " << objectType_
                 << " managed object handle :" << objectHandle << ". Aborting";
      return false;
    }
    return true;
  }  // ManagedContainerBase::checkExistsWithMessage

  /**
   * @brief Used Internally. Get an unused/available ID to be assigned to an
   * object as it is being added to the library.
   * @return The managed object's ID if found. The next available ID if not
   * found and getNext is true. Otherwise ID_UNDEFINED.
   */
  int getUnusedObjectID() {
    // return next available managed object ID (from previously deleted
    // managed object) or managed object library size as next ID to be used
    if (availableObjectIDs_.size() > 0) {
      int retVal = availableObjectIDs_.front();
      availableObjectIDs_.pop_front();
      return retVal;
    }
    return objectLibrary_.size();

  }  // ManagedContainerBase::getObjectIDByHandle

  /**
   * @brief Return a random handle selected from the passed map
   *
   * @param mapOfHandles map containing the desired attribute-type managed
   * object handles
   * @param type the type of managed object being retrieved, for debug message
   * @return a random managed object handle of the chosen type, or the empty
   * string if none loaded
   */
  std::string getRandomObjectHandlePerType(
      const std::map<int, std::string>& mapOfHandles,
      const std::string& type) const;

  /**
   * @brief Get a list of all managed objects of passed type whose origin
   * handles contain substr, ignoring subStr's case.
   *
   * This version works on std::map<int,std::string> maps' values.
   * @param mapOfHandles map containing the desired object-type managed object
   * handles
   * @param subStr substring to search for within existing primitive object
   * managed objects
   * @param contains Whether to search for handles containing, or not
   * containting, substr
   * @return vector of 0 or more managed object handles containing/not
   * containing the passed substring
   */
  std::vector<std::string> getObjectHandlesBySubStringPerType(
      const std::map<int, std::string>& mapOfHandles,
      const std::string& subStr,
      bool contains) const;

  /**
   * @brief Get a list of all managed objects of passed type whose origin
   * handles contain substr, ignoring subStr's case.
   *
   * This version works on std::map<std::string, std::set<std::string>> maps's
   * keys.
   * @param mapOfHandles map containing the desired keys to search.
   * @param subStr substring to search for within existing primitive object
   * managed objects
   * @param contains Whether to search for handles containing, or not
   * containting, substr
   * @return vector of 0 or more managed object handles containing/not
   * containing the passed substring
   */
  std::vector<std::string> getObjectHandlesBySubStringPerType(
      const std::map<std::string, std::set<std::string>>& mapOfHandles,
      const std::string& subStr,
      bool contains) const;

  /**
   * @brief Called internally only.  Remove all references from libraries for
   * object with passed ID and handle.
   * @param objectID the ID of the object to remove.
   * @param objectHandle the handle of the object to remove.
   */
  void deleteObjectInternal(int objectID, const std::string& objectHandle) {
    objectLibKeyByID_.erase(objectID);
    objectLibrary_.erase(objectHandle);
    availableObjectIDs_.emplace_front(objectID);
    // call instance-specific update to remove managed object handle from any
    // local lists
    updateObjectHandleLists(objectID, objectHandle);
  }  // ManagedContainerBase::deleteObjectInternal

  /**
   * @brief Any implementation-specific resetting that needs to happen on reset.
   */
  virtual void resetFinalize() = 0;

  // ========  Instance Variables ========
  /**
   * @brief Maps string keys to managed object managed objects
   */
  std::map<std::string, std::shared_ptr<void>> objectLibrary_;

  /** @brief A descriptive name of the managed object being managed by this
   * manager.
   */
  const std::string objectType_;

  /**
   * @brief Maps all object attribute IDs to the appropriate handles used
   * by lib
   */
  std::map<int, std::string> objectLibKeyByID_;

  /**
   * @brief Deque holding all IDs of deleted objects. These ID's should be
   * recycled before using map-size-based IDs
   */
  std::deque<int> availableObjectIDs_;

  /**
   * @brief set holding string managed object handles of all system-locked
   * managed objects, to make sure they are never deleted.  Should not be
   * overridden by user.
   */
  std::set<std::string> undeletableObjectNames_;

  /**
   * @brief set holding string managed object handles of all user-locked
   * managed objects, to make sure they are not deleted unless the user
   * unlocks them.
   */
  std::set<std::string> userLockedObjectNames_;

 public:
  ESP_SMART_POINTERS(ManagedContainerBase)
};  // class ManagedContainerBase

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_MANAGEDCONTAINERBASE_H_
