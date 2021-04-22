// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDRIGIDPHYSICSOBJECT_H_
#define ESP_PHYSICS_MANAGEDRIGIDPHYSICSOBJECT_H_

#include "ManagedPhysicsObjectBase.h"
#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {
class ManagedRigidObject : public esp::physics::AbstractManagedPhysicsObject<
                               esp::physics::RigidObject> {
 public:
  ManagedRigidObject(std::shared_ptr<esp::physics::RigidObject>& objPtr)
      : AbstractManagedPhysicsObject<esp::physics::RigidObject>(objPtr,
                                                                "RigidObject") {
  }

 protected:
 public:
  ESP_SMART_POINTERS(ManagedRigidObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDRIGIDPHYSICSOBJECT_H_
