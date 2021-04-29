// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
#define ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H

/** @file
 * @brief Class Template @ref esp::physics::PhysicsObjectBaseManager
 */

#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"

#include "esp/core/managedContainers/ManagedContainer.h"

namespace esp {
namespace core {
enum class ManagedObjectAccess;
class ManagedContainerBase;
}  // namespace core
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality for
 * managineg object wrappers specializing @ref
 * esp::physics::AbstractManagedPhysicsObject template.
 * @tparam T the type of managed physics object wrapper a particular
 * specialization of this class works with.  Must inherit from @ref
 * esp::physics::AbstractManagedPhysicsObject
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */
template <class T>
class PhysicsObjectBaseManager
    : public esp::core::ManagedContainer<T, core::ManagedObjectAccess::Copy> {
 public:
  typedef std::shared_ptr<T> ObjWrapperPtr;
  explicit PhysicsObjectBaseManager(const std::string& objType)
      : esp::core::ManagedContainer<T, core::ManagedObjectAccess::Copy>::
            ManagedContainer(objType) {}
  ~PhysicsObjectBaseManager() override = default;

  void setPhysicsManager(
      const std::shared_ptr<esp::physics::PhysicsManager>& physMgr) {
    weakPhysManager_ = physMgr;
  }

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
  ObjWrapperPtr createObject(
      const std::string& objectHandle,
      CORRADE_UNUSED bool registerObject = true) override;

 protected:
  /**
   * @brief Any physics-object-wrapper-specific resetting that needs to happen
   * on reset.
   */
  void resetFinalize() override {}

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
  void updateObjectHandleLists(
      CORRADE_UNUSED int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {}

  /**
   * @brief return a reference to physicsManager_, or null ptr if it does not
   * exist anymore.  This is necessary since a reference of this manager may
   * linger in python after simulator/physicsManager get clobbered/rebuilt.
   */
  std::shared_ptr<esp::physics::PhysicsManager> getPhysicsManager() const {
    std::shared_ptr<esp::physics::PhysicsManager> sp = weakPhysManager_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      LOG(WARNING) << "This object manager is no longer valid.  Please delete "
                      "any variable references.";
    }
    return sp;
  }  // getPhysicsManager

  /** @brief Weak reference to owning physics manager.
   */
  std::weak_ptr<esp::physics::PhysicsManager> weakPhysManager_{};

 public:
  ESP_SMART_POINTERS(PhysicsObjectBaseManager<T>)

};  // class PhysicsObjectBaseManager

/////////////////////////////
// Class Template Method Definitions

template <class T>
auto PhysicsObjectBaseManager<T>::createObject(
    const std::string& objectWrapperHandle,
    CORRADE_UNUSED bool registerTemplate) -> ObjWrapperPtr {
  ObjWrapperPtr objWrapper =
      this->createDefaultObject(objectWrapperHandle, false);

  return objWrapper;
}  // PhysicsObjectBaseManager<T>::createObject

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
