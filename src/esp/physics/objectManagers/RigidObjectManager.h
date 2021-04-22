// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDOBJECTMANAGER_H
#define ESP_PHYSICS_RIGIDOBJECTMANAGER_H

#include "RigidBaseManager.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"
namespace esp {
namespace physics {

class RigidObjectManager : public RigidBaseManager<ManagedRigidObject> {
 public:
  RigidObjectManager(std::shared_ptr<esp::physics::PhysicsManager> physMgr,
                     const std::string& objType)
      : RigidBaseManager<ManagedRigidObject>::RigidBaseManager(physMgr,
                                                               objType) {}

 protected:
 public:
  ESP_SMART_POINTERS(RigidObjectManager)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDOBJECTMANAGER_H
