// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDBULLETRIGIDOBJECT_H_
#define ESP_PHYSICS_MANAGEDBULLETRIGIDOBJECT_H_

#ifdef ESP_BUILD_WITH_BULLET
#include "esp/physics/bullet/BulletRigidObject.h"
#endif
#include "esp/physics/objectWrappers/ManagedRigidObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for RigidObject constructions.
 * Provides bindings for all RigidObject-specific functionality.
 */
class ManagedBulletRigidObject : public esp::physics::ManagedRigidObject {
 public:
  ManagedBulletRigidObject() : ManagedRigidObject("ManagedBulletRigidObject") {}

#ifdef ESP_BUILD_WITH_BULLET
  double getMargin() const {
    if (auto sp = this->getBulletObjectReference()) {
      return sp->getMargin();
    }
    return 0.0;
  }  // getMargin

  void setMargin(const double margin) {
    if (auto sp = this->getBulletObjectReference()) {
      sp->setMargin(margin);
    }
  }  // setMass

  Magnum::Range3D getCollisionShapeAabb() {
    if (auto sp = this->getBulletObjectReference()) {
      return sp->getCollisionShapeAabb();
    }
    return {};
  }  // getCollisionShapeAabb

 protected:
  /**
   * @brief This function accesses the
   * underlying shared pointer of this object's @p weakObjRef_ if it exists,
   * and casts it to BulletRigidObject; if it the ptr does not
   * exist, it provides a message.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * dne.
   */
  std::shared_ptr<BulletRigidObject> getBulletObjectReference() const {
    return std::static_pointer_cast<BulletRigidObject>(
        this->getObjectReference());
  }

#else
  //! no bullet version
  double getMargin() const {
    ESP_WARNING() << "This functionally requires Habitat-Sim to be compiled "
                     "with Bullet enabled..";

    return 0.0;
  }  // getMargin

  void setMargin(CORRADE_UNUSED const double margin) {
    ESP_WARNING() << "This functionally requires Habitat-Sim to be compiled "
                     "with Bullet enabled..";

  }  // setMass

  Magnum::Range3D getCollisionShapeAabb() {
    ESP_WARNING() << "This functionally requires Habitat-Sim to be compiled "
                     "with Bullet enabled..";

    return {};
  }  // getCollisionShapeAabbb

#endif

 public:
  ESP_SMART_POINTERS(ManagedBulletRigidObject)
};  // namespace physics

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDBULLETRIGIDOBJECT_H_
