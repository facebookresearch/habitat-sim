// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
 */

template <class T>
class RigidBaseManager : public esp::physics::PhysicsObjectBaseManager<T> {
 public:
  explicit RigidBaseManager(const std::string& objType)
      : esp::physics::PhysicsObjectBaseManager<T>::PhysicsObjectBaseManager(
            objType) {}

 protected:
 public:
  ESP_SMART_POINTERS(RigidBaseManager<T>)
};  // class RigidBaseManage

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDBASEMANAGER_H
