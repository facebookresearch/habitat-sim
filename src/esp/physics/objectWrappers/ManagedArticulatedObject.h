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
class ManagedArticulatedObject
    : public esp::physics::AbstractManagedPhysicsObject<
          esp::physics::ArticulatedObject> {
 public:
  ManagedArticulatedObject()
      : AbstractManagedPhysicsObject<esp::physics::ArticulatedObject>(
            "ManagedArticulatedObject") {}

  int createJointMotor(const int dof, const JointMotorSettings& settings) {
    if (auto sp = getObjectReference()) {
      return sp->createJointMotor(dof, settings);
    }
    return ID_UNDEFINED;
  }

  void removeJointMotor(const int motorId) {
    if (auto sp = getObjectReference()) {
      sp->removeJointMotor(motorId);
    }
  }

 public:
  ESP_SMART_POINTERS(ManagedArticulatedObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
