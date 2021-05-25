// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
#define ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_

#include "ManagedPhysicsObjectBase.h"
#include "esp/physics/ArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for ArticulatedObject constructions.
 * Provides bindings for all ArticulatedObject-specific functionality.
 */
template <class T>
class AbstractManagedArticulatedObject
    : public esp::physics::AbstractManagedPhysicsObject<T> {
 public:
  static_assert(
      std::is_base_of<esp::physics::ArticulatedObject, T>::value,
      "AbstractManagedRigidBase :: Managed physics object type must be "
      "derived from esp::physics::ArticulatedObject");
  AbstractManagedArticulatedObject()
      : AbstractManagedPhysicsObject<T>("ManagedArticulatedObject") {}

 public:
  ESP_SMART_POINTERS(AbstractManagedArticulatedObject<T>)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
