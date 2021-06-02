// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDRIGIDOBJECT_H_
#define ESP_PHYSICS_MANAGEDRIGIDOBJECT_H_

#include "ManagedRigidBase.h"
#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for RigidObject constructions.
 * Provides bindings for all RigidObject-specific functionality.
 */

class ManagedRigidObject
    : public esp::physics::AbstractManagedRigidBase<esp::physics::RigidObject> {
 public:
  explicit ManagedRigidObject(
      const std::string& classKey = "ManagedRigidObject")
      : AbstractManagedRigidBase<
            esp::physics::RigidObject>::AbstractManagedRigidBase(classKey) {}

  std::shared_ptr<metadata::attributes::ObjectAttributes>
  getInitializationAttributes() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getInitializationAttributes();
    }
    return nullptr;
  }  // getInitializationAttributes()

  VelocityControl::ptr getVelocityControl() {
    if (auto sp = this->getObjectReference()) {
      return sp->getVelocityControl();
    }
    return nullptr;
  }  // getVelocityControl()

 public:
  ESP_SMART_POINTERS(ManagedRigidObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDRIGIDOBJECT_H_
