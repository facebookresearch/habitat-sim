// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDBASEMANAGER_H
#define ESP_PHYSICS_RIGIDBASEMANAGER_H

#include "PhysicsObjectBaseManager.h"

namespace esp {
namespace physics {
/**
 * @brief Class template defining responsibilities and functionality shared for
 * managing all @ref esp::physics::ManagedRigidBase wrappers.
 * @tparam T the type of managed physics object a particular specialization
 * of this class works with.  Must inherit from @ref
 * esp::physics::ManagedRigidBase
 * @tparam Access Whether the default access (getters) for this
 * container provides copies of the objects held, or the actual objects
 * themselves.
 */

template <class T>
class RigidBaseManager : public PhysicsObjectBaseManager<T> {
 public:
  RigidBaseManager(const std::shared_ptr<esp::physics::PhysicsManager>& physMgr,
                   const std::string& objType)
      : PhysicsObjectBaseManager<T>::PhysicsObjectBaseManager(physMgr,
                                                              objType) {}

 protected:
 public:
  ESP_SMART_POINTERS(RigidBaseManager<T>)
};  // class RigidBaseManage

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDBASEMANAGER_H
