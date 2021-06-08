// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDBULLETARTICULATEDOBJECT_H_
#define ESP_PHYSICS_MANAGEDBULLETARTICULATEDOBJECT_H_

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletArticulatedObject.h"
#endif
#include "esp/physics/objectWrappers/ManagedArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for dynamic ArticulatedObject constructions
 * using the Bullet library. Provides bindings for all
 * ArticulatedObject-specific functionality.
 */
class ManagedBulletArticulatedObject
    : public esp::physics::ManagedArticulatedObject {
 public:
  ManagedBulletArticulatedObject()
      : ManagedArticulatedObject("ManagedBulletArticulatedObject") {}

#ifdef ESP_BUILD_WITH_BULLET
  bool contactTest() {
    if (auto sp = getBulletObjectReference()) {
      return sp->contactTest();
    }
    return false;
  }

  bool supportsJointMotor(int linkIx) {
    if (auto sp = getBulletObjectReference()) {
      return sp->supportsJointMotor(linkIx);
    }
    return false;
  }

  float getJointMotorMaxImpulse(int motorId) {
    if (auto sp = getBulletObjectReference()) {
      return sp->getJointMotorMaxImpulse(motorId);
    }
    return 0.0;
  }

 protected:
  /**
   * @brief Templated version of obj ref getter. This function accesses the
   * underlying shared pointer of this object's @p weakObjRef_ if it exists,
   * and casts it to the specified template parameter; if it the ptr does not
   * exist, it provides a message.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * dne.
   */

  std::shared_ptr<BulletArticulatedObject> getBulletObjectReference() const {
    return std::static_pointer_cast<BulletArticulatedObject>(
        this->getObjectReference());
  }
#else
  //! no bullet version
  bool contactTest() {
    LOG(WARNING) << "This functionaliy requires Habitat-Sim to be compiled "
                    "with Bullet enabled..";
    return false;
  }

  bool supportsJointMotor(CORRADE_UNUSED int linkIx) {
    LOG(WARNING) << "This functionaliy requires Habitat-Sim to be compiled "
                    "with Bullet enabled..";
    return false;
  }

  float getJointMotorMaxImpulse(CORRADE_UNUSED int motorId) {
    LOG(WARNING) << "This functionaliy requires Habitat-Sim to be compiled "
                    "with Bullet enabled..";
    return 0.0;
  }

  std::shared_ptr<ArticulatedObject> getBulletObjectReference() const {
    LOG(WARNING) << "This functionaliy requires Habitat-Sim to be compiled "
                    "with Bullet enabled..";

    return nullptr;
  }
#endif

 public:
  ESP_SMART_POINTERS(ManagedBulletArticulatedObject)
};  // namespace physics

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDBULLETARTICULATEDOBJECT_H_
