// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MANAGEDCONTAINERBASE_H_
#define ESP_CORE_MANAGEDCONTAINERBASE_H_

/** @file
 * @brief Class Template @ref esp::core::ManagedContainer : container
 * functionality to manage @ref esp::core::AbstractManagedObject objects
 */

#include <deque>
#include <functional>
#include <map>
#include <set>

#include <Corrade/Utility/Directory.h>
#include <Corrade/Utility/String.h>

#include "esp/core/AbstractManagedObject.h"

#include "esp/io/json.h"

namespace Cr = Corrade;

namespace esp {
namespace assets {
class ResourceManager;
}

namespace core {

/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::core::AbstractManagedObject constructs.
 * @tparam ManagedPtr the type of managed object a particular specialization of
 * this class works with.  Must inherit from @ref
 * esp::core::AbstractManagedObject.
 */
template <class T>
class ManagedContainer {
 public:
  static_assert(std::is_base_of<AbstractManagedObject, T>::value,
                "ManagedContainer :: Managed object type must be derived from "
                "AbstractManagedObject");

  typedef std::shared_ptr<T> ManagedPtr;

  ManagedContainer(esp::assets::ResourceManager& resourceManager,
                   const std::string& metadataType)
      : resourceManager_(resourceManager), objectType_(metadataType) {}
  virtual ~ManagedContainer() = default;

  /**
   * @brief Creates an instance of a managed object described by passed string.
   *
   * If a managed object exists with this handle, the existing managed object
   * will be overwritten with the newly created one if @ref
   * registerObject is true.
   *
   * @param objectHandle the origin of the desired managed object to be
   * created.
   * @param registerObject whether to add this managed object to the
   * library or not. If the user is going to edit this managed object, this
   * should be false. Defaults to true. If specified as true, then this function
   * returns a copy of the registered managed object.
   * @return a reference to the desired managed object.
   */
  virtual ManagedPtr createObject(const std::string& objectHandle,
                                  bool registerObject = true) = 0;

  /**
   * @brief Creates an instance of a managed object holding default values.
   *
   * If a managed object exists with this handle, the existing managed object
   * will be overwritten with the newly created one if @ref
   * registerObject is true. This method is specifically intended to
   * directly construct an managed object for editing, and so
   * defaults to false for @ref registerObject
   *
   * @param objectName The desired handle for this managed object.
   * @param registerObject whether to add this managed object to the
   * library or not. If the user is going to edit this managed object, this
   * should be false. If specified as true, then this function returns a copy of
   * the registered managed object. Defaults to false. If specified as true,
   * then this function returns a copy of the registered managed object.
   * @return a reference to the desired managed object.
   */
  ManagedPtr createDefaultObject(const std::string& objectName,
                                 bool registerObject = false) {
    // create default managed object
    ManagedPtr object = this->initNewObjectInternal(objectName);
    if (nullptr == object) {
      return object;
    }
    return this->postCreateRegister(object, registerObject);
  }  // ManagedContainer::createDefault

  /**
   * @brief Creates an instance of a managed object from a JSON file using
   * passed filename by loading and parsing the loaded JSON and generating a
   * @ref ManagedPtr object. It returns created instance if successful, and
   * nullptr if fails.
   *
   * @param filename the name of the file describing the object managed object.
   * Assumes it exists and fails if it does not.
   * @param registerObject whether to add this managed object to the
   * library. If the user is going to edit this managed object, this should be
   * false - any subsequent editing will require re-registration. Defaults to
   * true.
   * @return a reference to the desired managed object, or nullptr if fails.
   */

  ManagedPtr createObjectFromFile(const std::string& filename,
                                  bool registerObject = true) {
    // Load JSON config file
    io::JsonDocument jsonConfig;
    bool success = this->verifyLoadJson(filename, jsonConfig);
    if (!success) {
      LOG(ERROR) << objectType_
                 << "ManagedContainer::createObjectFromFile : "
                    "Failure reading json : "
                 << filename << ". Aborting.";
      return nullptr;
    }
    ManagedPtr attr = this->loadFromJSONDoc(filename, jsonConfig);
    return this->postCreateRegister(attr, registerObject);
  }  // ManagedContainer::createObjectFromFile

  /**
   * @brief Parse passed JSON Document specifically for @ref ManagedPtr object.
   * It always returns a @ref ManagedPtr object.
   * @param filename The name of the file describing the @ref ManagedPtr,
   * used as managed object handle/name on create.
   * @param jsonConfig json document to parse - assumed to be legal JSON doc.
   * @return a reference to the desired managed object.
   */
  virtual ManagedPtr loadFromJSONDoc(const std::string& filename,
                                     const io::JsonDocument& jsonConfig) = 0;

  /**
   * @brief Add a copy of @ref esp::core::AbstractManagedObject to the @ref
   * objectLibrary_.
   *
   * @param managedObject The managed object.
   * @param objectHandle The key for referencing the managed object in
   * the
   * @ref objectLibrary_. Will be set as origin handle for managed
   * object. If empty string, use existing origin handle.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  int registerObject(ManagedPtr managedObject,
                     const std::string& objectHandle = "") {
    if (nullptr == managedObject) {
      LOG(ERROR) << "ManagedContainer::registerObject : Invalid "
                    "(null) managed object passed to registration. Aborting.";
      return ID_UNDEFINED;
    }
    if ("" != objectHandle) {
      return registerObjectFinalize(managedObject, objectHandle);
    }
    std::string handleToSet = managedObject->getHandle();
    if ("" == handleToSet) {
      LOG(ERROR) << "ManagedContainer::registerObject : No "
                    "valid handle specified for "
                 << objectType_ << " managed object to register. Aborting.";
      return ID_UNDEFINED;
    }
    return registerObjectFinalize(managedObject, handleToSet);
  }  // ManagedContainer::registerObject

  /**
   * @brief Register managed object and call appropriate ResourceManager method
   * to execute appropriate post-registration processes due to changes in the
   * managed object. Use if user wishes to update existing objects built by
   * managed object with new managed object data and such objects support this
   * kind of update. Requires the use of managed object's assigned handle in
   * order to reference existing constructions built from the original version
   * of this managed object.
   * @param managedObject The managed object.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  int registerObjectAndUpdate(ManagedPtr managedObject) {
    std::string originalHandle = managedObject->getHandle();
    int ID = this->registerObject(managedObject, originalHandle);
    // If undefined then some error occurred.
    if (ID_UNDEFINED == ID) {
      return ID_UNDEFINED;
    }
    // TODO : call Resource Manager for post-registration processing of this
    // managed object

    return ID;
  }  // ManagedContainer::registerObjectAndUpdate

  /**
   * @brief clears maps of handle-keyed managed object and ID-keyed handles.
   */
  void reset() {
    objectLibKeyByID_.clear();
    objectLibrary_.clear();
    availableObjectIDs_.clear();
    resetFinalize();
  }  // ManagedContainer::reset

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
  }  // ManagedContainer::isValidFileName

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
    for (std::string objectHandle : objectHandles) {
      if (setLock(objectHandle, lock)) {
        res.push_back(objectHandle);
      }
    }
    return res;
  }  // ManagedContainer::setLockByHandles

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
  }  // ManagedContainer::setLockBySubstring

  // ======== Accessor functions ========
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
  }  // ManagedContainer::getObjectLibHasHandle

  /**
   * @brief Get a reference to the managed object identified by the
   * managedObjectID.  Should only be used internally. Users should
   * only ever access copies of managed objects.
   *
   * Can be used to manipulate a managed object before instancing new objects.
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref objectLibrary_ by @ref
   * objectLibKeyByID_.
   * @return A mutable reference to the object managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectByID(int managedObjectID) const {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    if (!checkExistsWithMessage(objectHandle,
                                "ManagedContainer::getObjectByID")) {
      return nullptr;
    }
    return objectLibrary_.at(objectHandle);
  }  // ManagedContainer::getObjectByID

  /**
   * @brief Get a reference to the managed object for the asset
   * identified by the passed objectHandle.  Should only be used
   * internally. Users should only ever access copies of managed objects.
   *
   * @param objectHandle The key referencing the asset in @ref
   * objectLibrary_.
   * @return A reference to the managed object, or nullptr if does not
   * exist
   */
  ManagedPtr getObjectByHandle(const std::string& objectHandle) const {
    if (!checkExistsWithMessage(objectHandle,
                                "ManagedContainer::getObjectByHandle")) {
      return nullptr;
    }
    return objectLibrary_.at(objectHandle);
  }  // ManagedContainer::getObject

  /**
   * @brief Remove the managed object referenced by the passed string handle.
   * Will emplace managed object ID within deque of usable IDs and return the
   * managed object being removed.
   * @param objectID The ID of the managed object desired.
   * @return the desired managed object being deleted, or nullptr if does not
   * exist
   */
  ManagedPtr removeObjectByID(int objectID) {
    std::string objectHandle = getObjectHandleByID(objectID);
    if (!checkExistsWithMessage(objectHandle,
                                "ManagedContainer::removeObjectByID")) {
      return nullptr;
    }
    return removeObjectInternal(objectHandle,
                                "ManagedContainer::removeObjectByID");
  }

  /**
   * @brief  Remove the managed object referenced by the passed string handle.
   * Will emplace managed object ID within deque of usable IDs and return the
   * managed object being removed.
   * @param objectHandle the string key of the managed object desired.
   * @return the desired managed object being deleted, or nullptr if does not
   * exist
   */
  ManagedPtr removeObjectByHandle(const std::string& objectHandle) {
    return removeObjectInternal(objectHandle,
                                "ManagedContainer::removeObjectByHandle");
  }

  /**
   * @brief Remove all managed objects that have not been marked as
   * default/non-removable, and return a vector of the managed objects removed.
   * @return A vector containing all the managed objects that have been removed
   * from the library.
   */
  std::vector<ManagedPtr> removeAllObjects() {
    return removeObjectsBySubstring();
  }  // removeAllObjects

  /**
   * @brief remove managed objects that contain passed substring and that have
   * not been marked as default/non-removable, and return a vector of the
   * managed objects removed.
   * @param subStr substring to search for within existing primitive object
   * managed objects
   * @param contains whether to search for keys containing, or excluding,
   * substr
   * @return A vector containing all the managed objects that have been removed
   * from the library.
   */
  std::vector<ManagedPtr> removeObjectsBySubstring(
      const std::string& subStr = "",
      bool contains = true);

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
      LOG(ERROR) << "ManagedContainer::getObjectHandleByID : Unknown "
                 << objectType_ << " managed object ID:" << objectID
                 << ". Aborting";
      return nullptr;
    }
    return objectLibKeyByID_.at(objectID);
  }  // ManagedContainer::getObjectHandleByID

  /**
   * @brief Get the ID of the managed object in @ref objectLibrary_ for
   * the given managed object Handle, if exists.
   *
   * @param objectHandle The string key referencing the managed object in
   * @ref objectLibrary_. Usually the origin handle.
   * @return The object ID for the managed object with the passed handle, or
   * ID_UNDEFINED if none exists.
   */
  int getObjectIDByHandle(const std::string& objectHandle) {
    return getObjectIDByHandleOrNew(objectHandle, false);
  }  // ManagedContainer::getObjectIDByHandle

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
  }  // ManagedContainer::getObjectHandlesBySubstring

  /**
   * @brief Get the handle for a random managed object registered to
   * this manager.
   *
   * @return a randomly selected handle corresponding to a known object
   * managed object, or empty string if none found
   */
  std::string getRandomObjectHandle() const {
    return getRandomObjectHandlePerType(objectLibKeyByID_, "");
  }  // ManagedContainer::getRandomObjectHandle

  /**
   * @brief Get a copy of the managed object identified by the
   * managedObjectID.
   *
   * Can be used to manipulate a managed object before instancing new objects.
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref objectLibrary_ by @ref
   * objectLibKeyByID_.
   * @return A mutable reference to the object managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectCopyByID(int managedObjectID) {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    if (!checkExistsWithMessage(objectHandle,
                                "ManagedContainer::getObjectCopyByID")) {
      return nullptr;
    }
    auto orig = this->objectLibrary_.at(objectHandle);
    return this->copyObject(orig);
  }  // ManagedContainer::getObjectCopyByID

  /**
   * @brief Return a reference to a copy of the object specified
   * by passed handle. This is the version that should be accessed by the
   * user.
   * @param objectHandle the string key of the managed object desired.
   * @return a copy of the desired managed object, or nullptr if does
   * not exist
   */
  ManagedPtr getObjectCopyByHandle(const std::string& objectHandle) {
    if (!checkExistsWithMessage(objectHandle,
                                "ManagedContainer::getObjectCopyByHandle")) {
      return nullptr;
    }
    auto orig = this->objectLibrary_.at(objectHandle);
    return this->copyObject(orig);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief Get a copy of the managed object identified by the
   * managedObjectID, casted to the appropriate derived managed object class.
   *
   * Can be used to manipulate a managed object before instancing new objects.
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref objectLibrary_ by @ref
   * objectLibKeyByID_.
   * @return A mutable reference to the object managed object, or nullptr if
   * does not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectCopyByID(int managedObjectID) {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    auto res = getObjectCopyByID(managedObjectID);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectCopyByID

  /**
   * @brief Return a reference to a copy of the object specified
   * by passed handle, casted to the appropriate derived managed object class.
   * This is the version that should be accessed by the user
   * @param objectHandle the string key of the managed object desired.
   * @return a copy of the desired managed object, or nullptr if does
   * not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectCopyByHandle(const std::string& objectHandle) {
    auto res = getObjectCopyByHandle(objectHandle);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief returns a vector of managed object handles representing the
   * system-specified undeletable managed objects this manager manages. These
   * managed objects cannot be deleted, although they can be edited.
   */
  std::vector<std::string> getUndeletableObjectHandles() const {
    std::vector<std::string> res(this->undeletableObjectNames_.begin(),
                                 this->undeletableObjectNames_.end());
    return res;
  }  // ManagedContainer::getUndeletableObjectHandles

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
  }  // ManagedContainer::getUserLockedObjectHandles

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
        LOG(ERROR) << objectType_
                   << "ManagedContainer::verifyLoadJson : Failed to parse "
                   << filename << " as JSON.";
        return false;
      }
      return true;
    } else {
      // by here always fail
      LOG(ERROR) << objectType_ << "ManagedContainer::verifyLoadJson : File "
                 << filename << " does not exist";
      return false;
    }
  }  // ManagedContainer::verifyLoadJson

  //======== Internally accessed functions ========
  /**
   * @brief Perform post creation registration if specified.
   *
   * @param object The managed object
   * @param doRegistration If managed object should be registered
   * @return managed object, or null ptr if registration failed.
   */
  inline ManagedPtr postCreateRegister(ManagedPtr object, bool doRegistration) {
    if (!doRegistration) {
      return object;
    }
    int objID = this->registerObject(object, object->getHandle());
    // return nullptr if registration error occurs.
    return (objID == ID_UNDEFINED) ? nullptr : object;
  }  // postCreateRegister

  /**
   * @brief Get directory component of managed object handle and call @ref
   * esp::core::AbstractManagedObject::setFileDirectory if a legitimate
   * directory exists in handle.
   *
   * @param object pointer to managed object to set
   */
  void setFileDirectoryFromHandle(ManagedPtr object) {
    std::string handleName = object->getHandle();
    auto loc = handleName.find_last_of("/");
    if (loc != std::string::npos) {
      object->setFileDirectory(handleName.substr(0, loc));
    }
  }  // setFileDirectoryFromHandle

  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle handle name to be assigned to the managed object.
   */
  virtual ManagedPtr initNewObjectInternal(const std::string& objectHandle) = 0;

  /**
   * @brief Used Internally. Remove the managed object referenced by the passed
   * string handle. Will emplace managed object ID within deque of usable IDs
   * and return the managed object being removed.
   * @param objectHandle the string key of the managed object desired.
   * @param src String denoting the source of the remove request.
   * @return the desired managed object being deleted, or nullptr if does not
   * exist
   */
  ManagedPtr removeObjectInternal(const std::string& objectHandle,
                                  const std::string& src);

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
   * @brief Used Internally. Get the ID of the managed object in @ref
   * objectLibrary_ for the given managed object Handle, if exists. If
   * the managed object is not in the library and getNext is true then returns
   * next available id, otherwise throws assertion and returns ID_UNDEFINED
   *
   * @param objectHandle The string key referencing the managed object in
   * @ref objectLibrary_. Usually the origin handle.
   * @param getNext Whether to get the next available ID if not found, or to
   * throw an assertion. Defaults to false
   * @return The managed object's ID if found. The next available ID if not
   * found and getNext is true. Otherwise ID_UNDEFINED.
   */
  int getObjectIDByHandleOrNew(const std::string& objectHandle, bool getNext) {
    if (getObjectLibHasHandle(objectHandle)) {
      return objectLibrary_.at(objectHandle)->getID();
    } else {
      if (!getNext) {
        LOG(ERROR) << "ManagedContainer::getObjectIDByHandle : No "
                   << objectType_ << " managed object with handle "
                   << objectHandle << "exists. Aborting";
        return ID_UNDEFINED;
      } else {
        // return next available managed object ID (from previously deleted
        // managed object) or managed object library size as next ID to be used
        if (availableObjectIDs_.size() > 0) {
          int retVal = availableObjectIDs_.front();
          availableObjectIDs_.pop_front();
          return retVal;
        }
        return objectLibrary_.size();
      }
    }
  }  // ManagedContainer::getObjectIDByHandle

  /**
   * @brief implementation of managed object type-specific registration
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  virtual int registerObjectFinalize(ManagedPtr object,
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
  }  // ManagedContainer::checkExistsWithMessage

  /**
   * @brief Any implementation-specific resetting that needs to happen on reset.
   */
  virtual void resetFinalize() = 0;

  /**
   * @brief Build a shared pointer to a copy of a the passed managed object, of
   * appropriate managed object type for passed object type.
   * @tparam U Type of managed object being created - must be a derived class of
   * ManagedPtr
   * @param orig original object of type ManagedPtr being copied
   */
  template <typename U>
  ManagedPtr createObjectCopy(ManagedPtr& orig) {
    // don't call init on copy - assume copy is already properly initialized.
    return U::create(*(static_cast<U*>(orig.get())));
  }  // ManagedContainer::

  /**
   * @brief This function will build the appropriate function pointer map for
   * this container's managed object, keyed on the managed object's class type.
   */
  virtual void buildCtorFuncPtrMaps() = 0;

  /**
   * @brief Build an @ref esp::core::AbstractManagedObject object of type
   * associated with passed object.
   * @param origAttr The ptr to the original AbstractManagedObject object to
   * copy
   */
  ManagedPtr copyObject(ManagedPtr& origAttr) {
    const std::string ctorKey = origAttr->getClassKey();
    return (*this.*(this->copyConstructorMap_[ctorKey]))(origAttr);
  }  // ManagedContainer::copyObject

  /**
   * @brief add passed managed object to library, setting managedObjectID
   * appropriately. Called internally by registerObject.
   *
   * @param object the managed object to add to the library
   * @param objectHandle the origin handle/name of the managed object to
   * add. The origin handle of the managed object will be set to this
   * here, in case this managed object is constructed with a different handle.
   * @return the managedObjectID of the managed object
   */
  int addObjectToLibrary(ManagedPtr object, const std::string& objectHandle) {
    // set handle for managed object - might not have been set during
    // construction
    object->setHandle(objectHandle);
    // return either the ID of the existing managed object referenced by
    // objectHandle, or the next available ID if not found.
    int objectID = getObjectIDByHandleOrNew(objectHandle, true);
    object->setID(objectID);
    // make a copy of this managed object so that user can continue to edit
    // original
    ManagedPtr managedObjectCopy = copyObject(object);
    // add to libraries
    objectLibrary_[objectHandle] = managedObjectCopy;
    objectLibKeyByID_.emplace(objectID, objectHandle);
    return objectID;
  }  // ManagedContainer::addObjectToLibrary

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
   * handles contain substr, ignoring subStr's case
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

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createObjectCopy keyed by string names of classes being instanced,
   */
  typedef std::map<std::string,
                   ManagedPtr (ManagedContainer<T>::*)(ManagedPtr&)>
      Map_Of_CopyCtors;

  /**
   * @brief Map of function pointers to instantiate a copy of a managed object.
   * A managed object is instanced by accessing the approrpiate function
   * pointer.
   */
  Map_Of_CopyCtors copyConstructorMap_;

  /** @brief A reference to a @ref esp::assets::ResourceManager
   */
  assets::ResourceManager& resourceManager_;

  /** @brief A descriptive name of the managed object being managed by this
   * manager.
   */
  const std::string objectType_;

  /**
   * @brief Maps string keys to managed object managed objects
   */
  std::map<std::string, ManagedPtr> objectLibrary_;

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
  ESP_SMART_POINTERS(ManagedContainer<ManagedPtr>)

};  // namespace managers

/////////////////////////////
// Class Template Method Definitions

template <class T>
bool ManagedContainer<T>::setLock(const std::string& objectHandle, bool lock) {
  // if managed object does not currently exist then do not attempt to modify
  // its lock state
  if (!checkExistsWithMessage(objectHandle, "ManagedContainer::setLock")) {
    return false;
  }
  // if setting lock else clearing lock
  if (lock) {
    userLockedObjectNames_.insert(objectHandle);
  } else if (userLockedObjectNames_.count(objectHandle) > 0) {
    // if clearing, verify exists
    userLockedObjectNames_.erase(objectHandle);
  }
  return true;
}  // ManagedContainer::setLock

template <class T>
auto ManagedContainer<T>::removeObjectsBySubstring(const std::string& subStr,
                                                   bool contains)
    -> std::vector<ManagedPtr> {
  std::vector<ManagedPtr> res;
  // get all handles that match query elements first
  std::vector<std::string> handles =
      getObjectHandlesBySubstring(subStr, contains);
  for (std::string objectHandle : handles) {
    ManagedPtr ptr = removeObjectInternal(
        objectHandle, "ManagedContainer::removeObjectsBySubstring");
    if (nullptr != ptr) {
      res.push_back(ptr);
    }
  }
  return res;
}  // removeAllObjects

template <class T>
auto ManagedContainer<T>::removeObjectInternal(const std::string& objectHandle,
                                               const std::string& sourceStr)
    -> ManagedPtr {
  if (!checkExistsWithMessage(objectHandle, sourceStr)) {
    LOG(INFO) << sourceStr << " : Unable to remove " << objectType_
              << " managed object " << objectHandle << " : Does not exist.";
    return nullptr;
  }

  auto attribsTemplate = getObjectCopyByHandle(objectHandle);
  std::string msg;
  if (this->undeletableObjectNames_.count(objectHandle) > 0) {
    msg = "Required Undeletable Managed Object";
  } else if (this->userLockedObjectNames_.count(objectHandle) > 0) {
    msg = "User-locked Template.  To delete managed object, unlock it.";
  }
  if (msg.length() != 0) {
    LOG(INFO) << sourceStr << " : Unable to remove " << objectType_
              << " managed object " << objectHandle << " : " << msg << ".";
    return nullptr;
  }
  int objectID = attribsTemplate->getID();
  objectLibKeyByID_.erase(objectID);
  objectLibrary_.erase(objectHandle);
  availableObjectIDs_.emplace_front(objectID);
  // call instance-specific update to remove managed object handle from any
  // local lists
  updateObjectHandleLists(objectID, objectHandle);
  return attribsTemplate;
}  // ManagedContainer::removeObjectByHandle

template <class T>
std::string ManagedContainer<T>::getRandomObjectHandlePerType(
    const std::map<int, std::string>& mapOfHandles,
    const std::string& type) const {
  std::size_t numVals = mapOfHandles.size();
  if (numVals == 0) {
    LOG(ERROR) << "Attempting to get a random " << type << objectType_
               << " managed object handle but none are loaded; Aboring";
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
}  // ManagedContainer::getRandomObjectHandlePerType

template <class T>
std::vector<std::string>
ManagedContainer<T>::getObjectHandlesBySubStringPerType(
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
}  // ManagedContainer::getObjectHandlesBySubStringPerType

}  // namespace core
}  // namespace esp
#endif  // ESP_CORE_MANAGEDCONTAINERBASE_H_
