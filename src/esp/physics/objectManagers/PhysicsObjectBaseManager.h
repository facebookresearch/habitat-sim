// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
#define ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H

/** @file
 * @brief Class Template @ref esp::physics::PhysicsObjectBaseManager
 */

#include "esp/physics/objectWrappers/ManagedPhysicsObjectBase.h"

#include "esp/core/ManagedContainer.h"

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
  PhysicsObjectBaseManager(
      std::shared_ptr<esp::physics::PhysicsManager> physMgr,
      const std::string& objType)
      : esp::core::ManagedContainer<T, core::ManagedObjectAccess::Copy>::
            ManagedContainer(objType),
        weakPhysManager_(physMgr) {}
  ~PhysicsObjectBaseManager() override = default;

 protected:
  /**
   * @brief return a reference to physicsManager_, or null ptr if it does not
   * exist anymore.  This is necessary since a reference of this manager may
   * linger in python after simulator/physicsManager get clobbered/rebuilt.
   */
  std::shared_ptr<esp::physics::PhysicsManager> getPhysicsManager() const {
    std::shared_ptr<esp::physics::PhysicsManager> sp = weakPhysManager_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      LOG(WARNING) << "This object manager no longer exists.  Please delete "
                      "any variable references.";
    }
    return sp;
  }  // getPhysicsManager

  /** @brief Weak reference to owning physics manager.
   */
  std::weak_ptr<esp::physics::PhysicsManager> weakPhysManager_;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBaseManager<T>)

};  // class PhysicsObjectBaseManager

// /**
//  * @brief Class template specialization defining responsibilities and
//  * functionality for managing @ref
//  esp::physics::AbstractManagedPhysicsObject
//  * wrappers.
//  *
//  */

// template <template <typename> class T,
//           typename Y,
//           core::ManagedObjectAccess Access>
// class PhysicsObjectBaseManager<T<Y>, Access>
//     : public esp::core::ManagedContainer<T<Y>, Access> {
//  public:
//   typedef T<Y> objWrapper;
//   typedef std::shared_ptr<objWrapper> ObjWrapperPtr;

//   static_assert(
//       std::is_base_of<esp::physics::AbstractManagedPhysicsObject<Y>,
//                       objWrapper>::value,
//       "PhysicsObjectBaseManager :: Managed object type must be derived from
//       " "AbstractManagedPhysicsObject");

//  public:
//   ESP_SMART_POINTERS(PhysicsObjectBaseManager<objWrapper, Access>)

// };  // PhysicsObjectBaseManager
}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASEMANAGER_H
