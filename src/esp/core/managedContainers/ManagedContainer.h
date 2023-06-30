// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MANAGEDCONTAINER_H_
#define ESP_CORE_MANAGEDCONTAINER_H_

/** @file
 * @brief Class Template @ref esp::core::ManagedContainer : container
 * functionality to manage @ref esp::core::AbstractManagedObject objects
 */

#include "ManagedContainerBase.h"

namespace esp {
namespace core {
namespace managedContainers {

/**
 * @brief This enum describes how objects held in the @ref ManagedConatainer are
 * accessed.
 */
enum class ManagedObjectAccess {
  /**
   * When an object is requested to be retrieved, a copy is made and
   * returned.  Modifications to this object will only take place if the object
   * is registered.
   */
  Copy,
  /**
   * When an object is requested to be retrieved, a reference to the
   * object itself is returned, and all changes will be tracked without
   * registration.
   */
  Share
};

/**
 * @brief Class template defining responsibilities and functionality for
 * managing @ref esp::core::AbstractManagedObject constructs.
 * @tparam T the type of managed object a particular specialization of
 * this class works with.  Must inherit from @ref
 * esp::core::AbstractManagedObject.
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */
template <class T, ManagedObjectAccess Access>
class ManagedContainer : public ManagedContainerBase {
 public:
  static_assert(std::is_base_of<AbstractManagedObject, T>::value,
                "ManagedContainer :: Managed object type must be derived from "
                "AbstractManagedObject");

  /**
   * @brief Alias for shared pointer to the @ref
   * esp::core::AbstractManagedObject this container manages.
   */
  typedef std::shared_ptr<T> ManagedPtr;

  /**
   * @brief Constructor
   * @param metadataType The name of the managed object type.
   */
  explicit ManagedContainer(const std::string& metadataType)
      : ManagedContainerBase(metadataType) {}

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
    ManagedPtr object = this->initNewObjectInternal(objectName, false);
    if (nullptr == object) {
      return nullptr;
    }
    return this->postCreateRegister(std::move(object), registerObject);
  }  // ManagedContainer::createDefault

  /**
   * @brief Add a copy of @ref esp::core::AbstractManagedObject to the @ref
   * objectLibrary_.
   *
   * @param managedObject The managed object.
   * @param objectHandle The key for referencing the managed object in
   * the @ref ManagedContainerBase::objectLibrary_. Will be set as origin handle
   * for managed object. If empty string, use existing origin handle.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail in registerObjectFinalize.
   *
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  int registerObject(ManagedPtr managedObject,
                     const std::string& objectHandle = "",
                     bool forceRegistration = false) {
    if (nullptr == managedObject) {
      ESP_ERROR(Magnum::Debug::Flag::NoSpace)
          << "<" << this->objectType_
          << "> : Invalid (null) managed object passed to "
             "registration, so registration aborted.";
      return ID_UNDEFINED;
    }
    if ("" != objectHandle) {
      return registerObjectFinalize(std::move(managedObject), objectHandle,
                                    forceRegistration);
    }
    std::string handleToSet = managedObject->getHandle();
    if ("" == handleToSet) {
      ESP_ERROR(Magnum::Debug::Flag::NoSpace)
          << "<" << this->objectType_
          << "> : No valid handle specified to register this managed object, "
             "so registration aborted.";
      return ID_UNDEFINED;
    }
    return registerObjectFinalize(std::move(managedObject), handleToSet,
                                  forceRegistration);
  }  // ManagedContainer::registerObject

  /**
   * @brief Get a reference to the managed object identified by the
   * managedObjectID. Should only be used internally - Users should
   * only ever access copies of managed objects, unless this managed container's
   * @p Access policy is Share.
   *
   * Can be used to manipulate a managed object before instancing new objects.
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref ManagedContainerBase::objectLibrary_ .
   * @return A mutable reference to the object managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectByID(int managedObjectID) const {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    if (!checkExistsWithMessage(objectHandle,
                                "<" + this->objectType_ + ">::getObjectByID")) {
      return nullptr;
    }
    return getObjectInternal<T>(objectHandle);
  }  // ManagedContainer::getObjectByID

  /**
   * @brief Get a reference to the managed object identified by the passed
   * objectHandle. Should only be used internally - Users should
   * only ever access copies of managed objects, unless this managed container's
   * @p Access policy is Share.
   *
   * @param objectHandle The key referencing the managed object in @ref
   * objectLibrary_.
   * @return A reference to the managed object, or nullptr if does not
   * exist
   */
  ManagedPtr getObjectByHandle(const std::string& objectHandle) const {
    if (!checkExistsWithMessage(
            objectHandle, "<" + this->objectType_ + ">::getObjectByHandle")) {
      return nullptr;
    }
    return getObjectInternal<T>(objectHandle);
  }  // ManagedContainer::getObject

  /**
   * @brief Retrieve a map of key= std::string handle; value = copy of
   * ManagedPtr object where the handles match the passed @p .  See @ref
   * ManagedContainerBase::getObjectHandlesBySubStringPerType.
   * @param subStr substring key to search for within existing managed objects.
   * @param contains whether to search for keys containing, or excluding,
   * passed @p subStr
   * @return a map of the objects whose keys match the specified substring
   * search.
   */
  std::unordered_map<std::string, ManagedPtr> getObjectsByHandleSubstring(
      const std::string& subStr = "",
      bool contains = true) {
    std::vector<std::string> keys = this->getObjectHandlesBySubStringPerType(
        objectLibKeyByID_, subStr, contains, false);

    std::unordered_map<std::string, ManagedPtr> res;
    res.reserve(keys.size());
    if (Access == ManagedObjectAccess::Copy) {
      for (const auto& key : keys) {
        res[key] = this->getObjectCopyByHandle(key);
      }
    } else {
      for (const auto& key : keys) {
        res[key] = this->getObjectByHandle(key);
      }
    }
    return res;
  }

  /**
   * @brief Templated version. Retrieve a map of key= std::string handle; value
   * = copy of ManagedPtr object where the handles match the passed @p .  See
   * @ref ManagedContainerBase::getObjectHandlesBySubStringPerType.
   *
   * @tparam Desired downcast class that inerheits from this ManagedContainer's
   * ManagedObject type.
   * @param subStr substring key to search for within existing managed objects.
   * @param contains whether to search for keys containing, or excluding,
   * passed @p subStr
   * @return a map of the objects whose keys match the specified substring
   * search.
   */
  template <class U>
  std::unordered_map<std::string, std::shared_ptr<U>>
  getObjectsByHandleSubstring(const std::string& subStr = "",
                              bool contains = true) {
    std::vector<std::string> keys = this->getObjectHandlesBySubStringPerType(
        objectLibKeyByID_, subStr, contains, false);

    std::unordered_map<std::string, std::shared_ptr<U>> res;
    res.reserve(keys.size());
    if (Access == ManagedObjectAccess::Copy) {
      for (const auto& key : keys) {
        res[key] = this->getObjectCopyByHandle<U>(key);
      }
    } else {
      for (const auto& key : keys) {
        res[key] = std::dynamic_pointer_cast<U>(this->getObjectByHandle(key));
      }
    }
    return res;
  }

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
    if (!checkExistsWithMessage(
            objectHandle, "<" + this->objectType_ + ">::removeObjectByID")) {
      return nullptr;
    }
    return removeObjectInternal(
        objectID, objectHandle,
        "<" + this->objectType_ + ">::removeObjectByID");
  }

  /**
   * @brief Remove the managed object referenced by the passed string handle.
   * Will emplace managed object ID within deque of usable IDs and return the
   * managed object being removed.
   * @param objectHandle the string key of the managed object desired.
   * @return the desired managed object being deleted, or nullptr if does not
   * exist
   */
  ManagedPtr removeObjectByHandle(const std::string& objectHandle) {
    if (!checkExistsWithMessage(objectHandle, "<" + this->objectType_ +
                                                  ">::removeObjectByHandle")) {
      return nullptr;
    }
    int objectID = this->getObjectIDByHandle(objectHandle);
    if (objectID == ID_UNDEFINED) {
      return nullptr;
    }
    return removeObjectInternal(
        objectID, objectHandle,
        "<" + this->objectType_ + ">::removeObjectByHandle");
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
   * @brief Remove managed objects that contain passed substring and that have
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
   * @brief Get a reference to, or a copy of, the managed object identified by
   * the @p managedObjectID, depending on @p Access value.  This is the
   * function that should be be accessed by the user for general object
   * consumption by ID.
   *
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref ManagedContainerBase::objectLibrary_ .
   * @return A mutable reference to the managed object, or a copy, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectOrCopyByID(int managedObjectID) {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    return this->getObjectOrCopyByHandle(objectHandle);
  }  // ManagedContainer::getObjectOrCopyByID

  /**
   * @brief Get a reference to, or a copy of, the managed object identified by
   * the @p objectHandle, depending on @p Access value.  This is the function
   * that should be be accessed by the user for general object consumption by
   * Handle.
   *
   * @param objectHandle the string key of the managed object desired.
   * @return A mutable reference to the managed object, or a copy, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectOrCopyByHandle(const std::string& objectHandle) {
    if (Access == ManagedObjectAccess::Copy) {
      return this->getObjectCopyByHandle(objectHandle);
    } else {
      return this->getObjectByHandle(objectHandle);
    }
  }  // ManagedContainer::getObjectOrCopyByHandle

  /**
   * @brief Get a reference to, or a copy of, the managed object identified by
   * the @p managedObjectID, depending on @p Access value, and casted to the
   * appropriate derived managed object class. This is the version that should
   * be accessed by the user for type-casted object consumption by ID.
   *
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref ManagedContainerBase::objectLibrary_.
   * @return A mutable reference to the managed object, or a copy, casted to
   * requested type, or nullptr if does not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectOrCopyByID(int managedObjectID) {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    return this->getObjectOrCopyByHandle<U>(objectHandle);
  }  // ManagedContainer::getObjectOrCopyByID

  /**
   * @brief Get a reference to, or a copy of, the managed object identified by
   * the @p managedObjectID, depending on @p Access value, and casted to the
   * appropriate derived managed object class. This is the version that should
   * be accessed by the user for type-casted object consumption by
   * Handle.
   *
   * @param objectHandle the string key of the managed object desired.
   * @return A mutable reference to the managed object, or a copy, casted to
   * requested type, or nullptr if does not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectOrCopyByHandle(const std::string& objectHandle) {
    // call non-template version
    auto res = getObjectOrCopyByHandle(objectHandle);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectOrCopyByHandle

  /**
   * @brief Get a reference to a copy of the managed object identified
   * by the @p managedObjectID.
   *
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref ManagedContainerBase::objectLibrary_ .
   * @return A mutable reference to a copy of the managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectCopyByID(int managedObjectID) {
    std::string objectHandle = getObjectHandleByID(managedObjectID);
    if (!checkExistsWithMessage(
            objectHandle, "<" + this->objectType_ + ">::getObjectCopyByID")) {
      return nullptr;
    }
    auto orig = getObjectInternal<T>(objectHandle);
    return this->copyObject(orig);
  }  // ManagedContainer::getObjectCopyByID

  /**
   * @brief Get a reference to a copy of the object specified
   * by @p objectHandle
   * @param objectHandle the string key of the managed object desired.
   * @return A mutable reference to a copy of the managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getObjectCopyByHandle(const std::string& objectHandle) {
    if (!checkExistsWithMessage(objectHandle, "<" + this->objectType_ +
                                                  ">::getObjectCopyByHandle")) {
      return nullptr;
    }
    auto orig = getObjectInternal<T>(objectHandle);
    return this->copyObject(orig);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief Get a reference to a copy of the managed object identified
   * by the @p managedObjectID, casted to the appropriate derived managed object
   * class.
   *
   * @tparam Desired downcast class that inerheits from this ManagedContainer's
   * ManagedObject type.
   * @param managedObjectID The ID of the managed object. Is mapped to the key
   * referencing the asset in @ref ManagedContainerBase::objectLibrary_.
   * @return A mutable reference to a copy of the managed object casted to the
   * requested type, or nullptr if does not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectCopyByID(int managedObjectID) {
    // call non-template version
    auto res = getObjectCopyByID(managedObjectID);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectCopyByID

  /**
   * @brief Get a reference to a copy of the object specified
   * by @p objectHandle , casted to the appropriate derived managed object
   * class.
   *
   * @param objectHandle the string key of the managed object desired.
   * @return A mutable reference to a copy of the managed object casted to the
   * requested type, or nullptr if does not exist
   */
  template <class U>
  std::shared_ptr<U> getObjectCopyByHandle(const std::string& objectHandle) {
    // call non-template version
    auto res = getObjectCopyByHandle(objectHandle);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief Set the object to provide default values upon construction of @ref
   * esp::core::AbstractManagedObject.  Override if object should not have
   * defaults
   * @param _defaultObj the object to use for defaults;
   */
  virtual void setDefaultObject(ManagedPtr& _defaultObj) {
    defaultObj_ = _defaultObj;
  }

  /**
   * @brief Clear any default objects used for construction.
   */
  void clearDefaultObject() { defaultObj_ = nullptr; }

 protected:
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
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle handle name to be assigned to the managed object.
   * @param builtFromConfig Managed Object is being constructed from a config
   * file (i.e. @p objectHandle is config file filename).  If false this means
   * Manage Object is being constructed as some kind of new/default.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  virtual ManagedPtr initNewObjectInternal(const std::string& objectHandle,
                                           bool builtFromConfig) = 0;

  /**
   * @brief Used Internally. Remove the managed object referenced by the passed
   * string handle. Will emplace managed object ID within deque of usable IDs
   * and return the managed object being removed.
   *
   * @param objectID the id of the managed object desired.
   * @param objectHandle the string key of the managed object desired.
   * @param src String denoting the source of the remove request.
   * @return the desired managed object being deleted, or nullptr if does not
   * exist
   */
  ManagedPtr removeObjectInternal(int objectID,
                                  const std::string& objectHandle,
                                  const std::string& src);

  /**
   * @brief implementation of managed object type-specific registration
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  virtual int registerObjectFinalize(ManagedPtr object,
                                     const std::string& objectHandle,
                                     bool forceRegistration) = 0;

  /**
   * @brief Build a shared pointer to a copy of a the passed managed object,
   * of appropriate managed object type for passed object type.  This is the
   * function called by the copy constructor map.
   * @tparam U Type of managed object being created - must be a derived class
   * of ManagedPtr
   * @param orig original object of type ManagedPtr being copied
   */
  template <typename U>
  ManagedPtr createObjectCopy(ManagedPtr& orig) {
    // don't call init on copy - assume copy is already properly initialized.
    return U::create(*(static_cast<U*>(orig.get())));
  }  // ManagedContainer::

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
   * @brief Create a new object as a copy of @p defaultObject_  if it exists,
   * otherwise return nullptr.
   * @param newHandle the name for the copy of the default.
   * @return New object or nullptr
   */
  ManagedPtr constructFromDefault(const std::string& newHandle) {
    if (defaultObj_ == nullptr) {
      return nullptr;
    }
    ManagedPtr res = copyObject(defaultObj_);
    if (nullptr != res) {
      res->setHandle(newHandle);
    }
    return res;
  }  // ManagedContainer::constructFromDefault

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
    object->setID(getObjectIDByHandleOrNew(objectHandle, true));
    // use object's ID for ID in container - may not match ID synthesized by
    // getObjectIDByHandle, for managed objects that control their own IDs
    int objectID = object->getID();

    // make a copy of this managed object so that user can continue to edit
    // original
    ManagedPtr managedObjectCopy = copyObject(object);
    // add to libraries
    setObjectInternal(managedObjectCopy, objectHandle);
    objectLibKeyByID_.emplace(objectID, objectHandle);
    return objectID;
  }  // ManagedContainer::addObjectToLibrary

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createObjectCopy keyed by string names of classes being instanced,
   */
  typedef std::unordered_map<std::string,
                             ManagedPtr (ManagedContainer<T, Access>::*)(
                                 ManagedPtr&)>
      Map_Of_CopyCtors;

  /**
   * @brief Map of function pointers to instantiate a copy of a managed
   * object. A managed object is instanced by accessing the approrpiate
   * function pointer.  THIS MUST BE INSTANCED IN SPECIALIZATION CONSTRUCTOR.
   */
  Map_Of_CopyCtors copyConstructorMap_;

  /**
   * @brief An object to provide default values, to be used upon
   * AbstractManagedObject construction
   */
  ManagedPtr defaultObj_ = nullptr;

 public:
  ESP_SMART_POINTERS(ManagedContainer<T, Access>)

};  // class ManagedContainer

/////////////////////////////
// Class Template Method Definitions

template <class T, ManagedObjectAccess Access>
auto ManagedContainer<T, Access>::removeObjectsBySubstring(
    const std::string& subStr,
    bool contains) -> std::vector<ManagedPtr> {
  std::vector<ManagedPtr> res;
  // get all handles that match query elements first
  std::vector<std::string> handles =
      getObjectHandlesBySubstring(subStr, contains);
  for (const std::string& objectHandle : handles) {
    int objID = this->getObjectIDByHandle(objectHandle);
    ManagedPtr ptr = removeObjectInternal(objID, objectHandle,
                                          "<" + this->objectType_ + ">");
    if (nullptr != ptr) {
      res.push_back(ptr);
    }
  }
  return res;
}  // ManagedContainer<T, Access>::removeObjectsBySubstring

template <class T, ManagedObjectAccess Access>
auto ManagedContainer<T, Access>::removeObjectInternal(
    int objectID,
    const std::string& objectHandle,
    const std::string& sourceStr) -> ManagedPtr {
  if (!checkExistsWithMessage(objectHandle, sourceStr)) {
    ESP_DEBUG() << sourceStr << ": Unable to remove" << objectType_
                << "managed object" << objectHandle << ": Does not exist.";
    return nullptr;
  }
  std::string msg;
  if (this->getIsUndeletable(objectHandle)) {
    msg = "Required Undeletable Managed Object";
  } else if (this->getIsUserLocked(objectHandle)) {
    msg = "User-locked Object.  To delete managed object, unlock it";
  }
  if (msg.length() != 0) {
    ESP_DEBUG() << sourceStr << ": Unable to remove" << objectType_
                << "managed object" << objectHandle << ":" << msg << ".";
    return nullptr;
  }
  ManagedPtr managedObject = getObjectInternal<T>(objectHandle);
  // remove the object and all references to it from the various internal maps
  // holding them.

  deleteObjectInternal(objectID, objectHandle);
  return managedObject;
}  // ManagedContainer::removeObjectInternal

}  // namespace managedContainers
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_MANAGEDCONTAINER_H_
