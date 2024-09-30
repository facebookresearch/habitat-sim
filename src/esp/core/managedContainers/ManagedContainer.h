// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_MANAGEDCONTAINER_H_
#define ESP_CORE_MANAGEDCONTAINER_H_

/** @file
 * @brief Class Template @ref esp::core::managedContainers::ManagedContainer :
 * container functionality to manage @ref
 * esp::core::managedContainers::AbstractManagedObject objects
 */

#include "ManagedContainerBase.h"

namespace esp {
namespace core {
namespace managedContainers {

/**
 * @brief This enum describes the return status from preregistration
 * conditioning of attributes. Preregistration is performed by
 * @ref preRegisterObjectFinalize , which will conduct any type-specific
 * initialization and/or validation that might be required before an object
 * is registered (i.e. saved in the @ref ManagedContainer). The return status
 * of this preregistration specifies how the registration proceess should
 * proceed.
 */
enum class ManagedObjectPreregistration {
  /**
   * The preregistration processing failed for some reason, and the managed
   * object will not be registered.
   */
  Failed,
  /**
   * The preregistration succeeded, the object can be registered with the given
   * handle.
   */
  Success,
  /**
   * The preregistration succeeded, but the object has a self-derived
   * registration handle that must be used. (i.e. PrimitiveAttributes)
   */
  Success_Use_Object_Handle
};

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
 * managing @ref esp::core::managedContainers::AbstractManagedObject constructs.
 * @tparam T the type of managed object a particular specialization of
 * this class works with.  Must inherit from @ref
 * esp::core::managedContainers::AbstractManagedObject.
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
   * esp::core::managedContainers::AbstractManagedObject this container manages.
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
   * @brief Add a copy of @ref
   * esp::core::managedContainers::AbstractManagedObject to the @ref
   * objectLibrary_.
   *
   * @param managedObject The managed object.
   * @param objectHandle The key for referencing the managed object in
   * the @ref ManagedContainerBase::objectLibrary_. Will be set as origin handle
   * for managed object. If empty string, use existing origin handle.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail in registerObjectInternal.
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
    // If no handle give, query object for handle
    std::string handleToSet =
        ("" == objectHandle) ? managedObject->getHandle() : objectHandle;
    // if still no handle, fail registration
    if ("" == handleToSet) {
      ESP_ERROR(Magnum::Debug::Flag::NoSpace)
          << "<" << this->objectType_
          << "> : No valid handle specified to register this managed object, "
             "so registration aborted.";
      return ID_UNDEFINED;
    }
    // Perform actual registration
    return this->registerObjectInternal(std::move(managedObject), handleToSet,
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
    if (!checkExistsWithMessage(objectHandle, "getObjectByID")) {
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
    if (!checkExistsWithMessage(objectHandle, "getObjectByHandle")) {
      return nullptr;
    }
    return getObjectInternal<T>(objectHandle);
  }  // ManagedContainer::getObjectByHandle

  /**
   * @brief Get a reference to the first matching managed object that contains
   * the passed string as a substring for it's handle. Should only be used
   * internally - Users should only ever access copies of managed objects,
   * unless this managed container's @p Access policy is Share.
   *
   * @param objectHandle The substring of the handle referencing the managed object in @ref
   * objectLibrary_ to search for.
   * @return A reference to the first managed object, or nullptr if does not
   * exist
   */
  ManagedPtr getFirstMatchingObjectByHandle(
      const std::string& objectHandle) const {
    if (getObjectLibHasHandle(objectHandle)) {
      // Has the passed string as a handle.
      return getObjectInternal<T>(objectHandle);
    }
    // search for elements containing objectHandle
    std::vector<std::string> handles =
        getObjectHandlesBySubstring(objectHandle);
    if (handles.size() == 0) {
      // Nothing matched
      return nullptr;
    }
    // A handle matched, get the object directly (bipassing any more checks)
    return getObjectInternal<T>(handles[0]);
  }  // ManagedContainer::getFirstMatchingObjectByHandle

  /**
   * @brief Retrieve a map of key= std::string handle; value = copy of
   * ManagedPtr object where the handles match the passed @p .  See @ref
   * ManagedContainerBase::getAllObjectHandlesBySubStringPerType.
   * @param subStr substring key to search for within existing managed objects.
   * @param contains whether to search for keys containing, or excluding,
   * passed @p subStr
   * @return a map of the objects whose keys match the specified substring
   * search.
   */
  std::unordered_map<std::string, ManagedPtr> getObjectsByHandleSubstring(
      const std::string& subStr = "",
      bool contains = true) {
    std::vector<std::string> keys =
        this->getAllObjectHandlesBySubStringPerType(subStr, contains, false);

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
   * @ref ManagedContainerBase::getAllObjectHandlesBySubStringPerType.
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
    static_assert(std::is_base_of<T, U>::value,
                  "ManagedContainer :: Desired type must be derived from "
                  "Managed object type");
    std::vector<std::string> keys =
        this->getAllObjectHandlesBySubStringPerType(subStr, contains, false);

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
    if (!checkExistsWithMessage(objectHandle, "removeObjectByID")) {
      return nullptr;
    }
    return removeObjectInternal(objectID, objectHandle, "removeObjectByID");
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
    if (!checkExistsWithMessage(objectHandle, "removeObjectByHandle")) {
      return nullptr;
    }
    int objectID = this->getObjectIDByHandle(objectHandle);
    if (objectID == ID_UNDEFINED) {
      return nullptr;
    }
    return removeObjectInternal(objectID, objectHandle, "removeObjectByHandle");
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
   * @brief Get a reference to, or a copy of, the first managed object found
   * containing the passed the @p handleSubstr, as part of its handle. depending
   * on @p Access value.  This is the function that should be be accessed by the
   * user for handle substring consumption.
   *
   * @param handleSubstr the substring key to use to find the first matching
   * copy.
   * @return A mutable reference to the managed object, or a copy, or nullptr if
   * does not exist
   */
  ManagedPtr getFirstMatchingObjectOrCopyByHandle(
      const std::string& handleSubstr) {
    if (Access == ManagedObjectAccess::Copy) {
      return this->getFirstMatchingObjectCopyByHandle(handleSubstr);
    } else {
      return this->getFirstMatchingObjectByHandle(handleSubstr);
    }
  }  // ManagedContainer::getFirstMatchingObjectOrCopyByHandle

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
    static_assert(std::is_base_of<T, U>::value,
                  "ManagedContainer :: Desired type must be derived from "
                  "Managed object type");
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
    if (!checkExistsWithMessage(objectHandle, "getObjectCopyByID")) {
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
    if (!checkExistsWithMessage(objectHandle, "getObjectCopyByHandle")) {
      return nullptr;
    }
    auto orig = getObjectInternal<T>(objectHandle);
    return this->copyObject(orig);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief Get a reference to a copy of the first object found containing the
   * specified by @p handleSubstr
   * @param handleSubstr the string key of the managed object desired.
   * @return A mutable reference to a copy of the managed object, or nullptr if
   * does not exist
   */
  ManagedPtr getFirstMatchingObjectCopyByHandle(
      const std::string& handleSubstr) {
    if (getObjectLibHasHandle(handleSubstr)) {
      // Has the passed string as a handle.
      auto orig = getObjectInternal<T>(handleSubstr);
      return this->copyObject(orig);
    }
    // search for elements containing objectHandle
    std::vector<std::string> handles =
        getObjectHandlesBySubstring(handleSubstr);
    if (handles.size() == 0) {
      // Nothing matched
      return nullptr;
    }
    // A handle matched, get the object directly (bipassing any more checks)
    auto orig = getObjectInternal<T>(handles[0]);
    return this->copyObject(orig);
  }  // ManagedContainer::getFirstMatchingObjectCopyByHandle

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
    static_assert(std::is_base_of<T, U>::value,
                  "ManagedContainer :: Desired type must be derived from "
                  "Managed object type");
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
    static_assert(std::is_base_of<T, U>::value,
                  "ManagedContainer :: Desired type must be derived from "
                  "Managed object type");
    // call non-template version
    auto res = getObjectCopyByHandle(objectHandle);
    if (nullptr == res) {
      return nullptr;
    }
    return std::dynamic_pointer_cast<U>(res);
  }  // ManagedContainer::getObjectCopyByHandle

  /**
   * @brief Set the object to provide default values upon construction of @ref
   * esp::core::managedContainers::AbstractManagedObject.  Override if object
   * should not have defaults
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
   * @brief This is the function called by the copy constructor map. Build a
   * shared pointer to a copy of a the passed managed object, of appropriate
   * managed object type for passed object type.
   *
   * @tparam U Type of managed object being created - must be a derived class
   * of ManagedPtr
   * @param orig original object of type ManagedPtr being copied
   */
  template <class U>
  ManagedPtr createObjCopyCtorMapEntry(ManagedPtr& orig) {
    static_assert(std::is_base_of<T, U>::value,
                  "ManagedContainer :: Desired type must be derived from "
                  "Managed object type");
    // don't call init on copy - assume copy is already properly initialized.
    return U::create(*(static_cast<U*>(orig.get())));
  }  // ManagedContainer::createObjCopyCtorMapEntry

  /**
   * @brief Build an @ref esp::core::managedContainers::AbstractManagedObject
   * object of type associated with passed object.
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
   * @brief This method will perform any final conditioning or updated required
   * by the @ref ManagedPtr object before it is registered.
   *
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return Whether there was an error in preconditioning the object that
   * prevents successful registration.
   */
  virtual ManagedObjectPreregistration preRegisterObjectFinalize(
      ManagedPtr object,
      const std::string& objectHandle,
      bool forceRegistration) = 0;

  /**
   * @brief This method will perform any final manager-related handling after
   * successfully registering an object.
   *
   * See @ref esp::attributes::managers::ObjectAttributesManager foran example.
   *
   * @param objectID the ID of the successfully registered managed object
   * @param objectHandle The name of the managed object
   */
  virtual void postRegisterObjectHandling(int objectID,
                                          const std::string& objectHandle) = 0;

 private:
  /**
   * @brief implementation of managed object registration. Will call the
   * appropriate type-specific preregistration conditioning before registering
   * the object and post-registration handling after successful registration.
   * @ref ManagedPtr object.
   *
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return The unique ID of the managed object being registered, or
   * ID_UNDEFINED if failed
   */
  int registerObjectInternal(ManagedPtr object,
                             const std::string& objectHandle,
                             bool forceRegistration) {
    // Handle preregistration type-specific processing of managed object
    ManagedObjectPreregistration status =
        preRegisterObjectFinalize(object, objectHandle, forceRegistration);
    // Don't register if failed.
    if (status == ManagedObjectPreregistration::Failed) {
      return ID_UNDEFINED;
    }
    const std::string& handleToUse =
        (status == ManagedObjectPreregistration::Success_Use_Object_Handle
             ? object->getHandle()
             : objectHandle);
    // adds template to library, and returns the ID of the existing template
    // referenced by handleToUse
    int objectID = this->addObjectToLibrary(std::move(object), handleToUse);
    // If registration succeeded then perform post-registration handling
    if (objectID != ID_UNDEFINED) {
      // post registration manager-specific processing.
      postRegisterObjectHandling(objectID, handleToUse);
    }
    return objectID;
  }  // registerObjectInternal

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
    setObjectInternal(managedObjectCopy, objectID, objectHandle);
    return objectID;
  }  // ManagedContainer::addObjectToLibrary

  // ======== Typedefs and Instance Variables ========
 protected:
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
    ManagedPtr ptr =
        removeObjectInternal(objID, objectHandle, "removeObjectsBySubstring");
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
    ESP_DEBUG(Magnum::Debug::Flag::NoSpace)
        << "<" + this->objectType_ + ">::" << sourceStr
        << " : Unable to remove requested managed object `" << objectHandle
        << "` : Does not exist.";
    return nullptr;
  }
  std::string msg;
  if (this->getIsUndeletable(objectHandle)) {
    msg = "Required Undeletable Managed Object";
  } else if (this->getIsUserLocked(objectHandle)) {
    msg = "User-locked Object. To delete managed object, unlock it";
  }
  if (msg.length() != 0) {
    ESP_DEBUG(Magnum::Debug::Flag::NoSpace)
        << "<" + this->objectType_ + ">::" << sourceStr
        << " : Unable to remove requested managed object `" << objectHandle
        << "` : Object is a " << msg << ".";
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
