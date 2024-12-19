// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

  /**
   * @brief Get a copy of the template attributes describing the initial state
   * of this object. These attributes have the combination of date from the
   * original object attributes and specific instance attributes used to create
   * this object. Note : values will reflect both sources, and should not
   * be saved to disk as object attributes, since instance attribute
   * modifications will still occur on subsequent loads
   */
  std::shared_ptr<metadata::attributes::ObjectAttributes>
  getInitializationAttributes() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getInitializationAttributes();
    }
    return nullptr;
  }  // getInitializationAttributes()

  /**
   * @brief Return the uncorrected translation (only different if COM correction
   * has occurred)
   */
  Magnum::Vector3 getUncorrectedTranslation() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getUncorrectedTranslation();
    }
    return Magnum::Vector3{};
  }  // getTranslation
  VelocityControl::ptr getVelocityControl() {
    if (auto sp = this->getObjectReference()) {
      return sp->getVelocityControl();
    }
    return nullptr;
  }  // getVelocityControl()

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, rigid-base-specific.
   */
  std::string getRigidBaseInfoHeaderInternal() const override {
    return "Creation Attributes Name";
  }

  /**
   * @brief Specialization-specific extension of getPhysObjInfoInternal, comma
   * separated info ideal for saving to csv information about RigidBase
   * constructs.
   */
  std::string getRigidBaseInfoInternal(
      std::shared_ptr<esp::physics::RigidObject>& sp) const override {
    return sp->getInitializationAttributes()->getHandle();
  }

 public:
  ESP_SMART_POINTERS(ManagedRigidObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDRIGIDOBJECT_H_
