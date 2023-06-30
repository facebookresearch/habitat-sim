// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
#define ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H

/** @file
 * @brief Class Template @ref esp::physics::PhysicsObjectBaseManager
 */

#include "esp/core/managedContainers/ManagedContainer.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"

namespace esp {
namespace core {
namespace managedContainers {
enum class ManagedObjectAccess;
class ManagedContainerBase;
}  // namespace managedContainers
}  // namespace core
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality for
 * managineg object wrappers specializing @ref
 * esp::physics::AbstractManagedPhysicsObject template.
 * @tparam T the type of managed physics object wrapper a particular
 * specialization of this class works with.  Must inherit from @ref
 * esp::physics::AbstractManagedPhysicsObject
 */
template <class T>
class PhysicsObjectBaseManager
    : public esp::core::managedContainers::ManagedContainer<
          T,
          core::managedContainers::ManagedObjectAccess::Copy> {
 public:
  typedef std::shared_ptr<T> ObjWrapperPtr;
  explicit PhysicsObjectBaseManager(const std::string& objType)
      : core::managedContainers::ManagedContainer<
            T,
            core::managedContainers::ManagedObjectAccess::Copy>::
            ManagedContainer(objType) {}
  ~PhysicsObjectBaseManager() override = default;

  /**
   * @brief set the weak reference to the physics manager that owns this wrapper
   * manager
   */
  void setPhysicsManager(std::weak_ptr<physics::PhysicsManager> physMgr) {
    weakPhysManager_ = std::move(physMgr);
  }

  /**
   * @brief Creates an empty @ref esp::physics::AbstractManagedPhysicsObject of
   * the type managed by this manager.
   *
   * @param objectTypeName Class name of the wrapper to create.  This will be
   * used as a key in @p managedObjTypeConstructorMap_.
   * @param registerObject whether to add this managed object to the
   * library or not. If the user is going to edit this managed object, this
   * should be false. Defaults to true. If specified as true, then this function
   * returns a copy of the registered managed object.
   * @return a reference to the desired managed object.
   */
  ObjWrapperPtr createObject(
      const std::string& objectTypeName,
      CORRADE_UNUSED bool registerObject = false) override;

 protected:
  /**
   * @brief Any physics-object-wrapper-specific resetting that needs to happen
   * on reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Build a shared pointer to a ManagedRigidObject wrapper around an
   * appropriately cast @ref esp::physics::RigidObject, either the base
   * kinematic version, or one built to support a dynamic library.
   * @tparam The type of the underlying wrapped object to create
   */
  template <typename U>
  ObjWrapperPtr createPhysicsObjectWrapper() {
    return U::create();
  }

  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectTypeName Used to determine what kind of Rigid Object wrapper
   * to make (either kinematic or dynamic-library-specific)
   * @param builtFromConfig Unused for wrapper objects.  All wrappers are
   * constructed from scratch.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  ObjWrapperPtr initNewObjectInternal(
      const std::string& objectTypeName,
      CORRADE_UNUSED bool builtFromConfig) override {
    // construct a new wrapper based on the passed object
    auto mgdObjTypeCtorMapIter =
        managedObjTypeConstructorMap_.find(objectTypeName);
    if (mgdObjTypeCtorMapIter == managedObjTypeConstructorMap_.end()) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_ << "> Unknown constructor type "
          << objectTypeName << ", so initNewObject aborted.";
      return nullptr;
    }
    auto newWrapper = (*this.*(mgdObjTypeCtorMapIter->second))();

    return newWrapper;
  }  // RigidObjectManager::initNewObjectInternal(

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
  int registerObjectFinalize(ObjWrapperPtr object,
                             const std::string& objectHandle,
                             CORRADE_UNUSED bool forceRegistration) override {
    // Add wrapper to template library
    return this->addObjectToLibrary(std::move(object), objectHandle);
  }  // PhysicsObjectBaseManager::registerObjectFinalize

  /**
   * @brief return a reference to physicsManager_, or null ptr if it does not
   * exist anymore.  This is necessary since a reference of this manager may
   * linger in python after simulator/physicsManager get clobbered/rebuilt.
   */
  std::shared_ptr<esp::physics::PhysicsManager> getPhysicsManager() const {
    auto sp = weakPhysManager_.lock();
    if (!sp) {
      // This warning message is for python-bound refs to the manager that
      // persist after Simulator/PhysicsManager have been deleted.
      ESP_WARNING() << "This object manager is no longer valid.  Please delete "
                       "any variable references.";
    }
    return sp;
  }  // getPhysicsManager

  // ====== instance variables =====

  /** @brief Weak reference to owning physics manager.
   */
  std::weak_ptr<esp::physics::PhysicsManager> weakPhysManager_{};

  /** @brief Define a map type referencing function pointers to @ref
   * createRigidObjectWrapper() keyed by string names of classes being
   * instanced, as defined in @ref PrimitiveNames3DMap
   */
  typedef std::map<std::string,
                   ObjWrapperPtr (PhysicsObjectBaseManager<T>::*)()>
      Map_Of_ManagedObjTypeCtors;

  /** @brief Map of function pointers to instantiate a @ref
   * esp::physics::AbstractManagedPhysicsObject wrapper object, keyed by the
   * wrapper's class name. A ManagedRigidObject wrapper of the appropriate type
   * is instanced by accessing the constructor map with the appropriate
   * classname.
   */
  Map_Of_ManagedObjTypeCtors managedObjTypeConstructorMap_;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBaseManager<T>)

};  // class PhysicsObjectBaseManager

/////////////////////////////
// Class Template Method Definitions

template <class T>
auto PhysicsObjectBaseManager<T>::createObject(
    const std::string& objectTypeName,
    CORRADE_UNUSED bool registerTemplate) -> ObjWrapperPtr {
  // This creates and returns an empty object wrapper.  The shared_ptr to the
  // actual @ref esp::physics::PhysicsObjectBase needs to be passed into this
  // wrapper before it is registered, so that a name for the object wrapper will
  // be generated.  Do not register object, since wrapper object requires actual
  // object being wrapped to be set separately.
  // NO default object will exist for wrappers, since they have no independent
  // data outside of the wrapped object.
  ObjWrapperPtr objWrapper = this->initNewObjectInternal(objectTypeName, false);

  return objWrapper;
}  // PhysicsObjectBaseManager<T>::createObject

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
