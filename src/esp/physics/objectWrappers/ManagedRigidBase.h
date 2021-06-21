// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDRIGIDBASE_H_
#define ESP_PHYSICS_MANAGEDRIGIDBASE_H_

#include "ManagedPhysicsObjectBase.h"
#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class template describing wrapper for RigidBase constructions.
 * Provides bindings for all RigidBase functionality.
 */

template <class T>
class AbstractManagedRigidBase
    : public esp::physics::AbstractManagedPhysicsObject<T> {
 public:
  static_assert(
      std::is_base_of<esp::physics::RigidBase, T>::value,
      "AbstractManagedRigidBase :: Managed physics object type must be "
      "derived from esp::physics::RigidBase");

  explicit AbstractManagedRigidBase(const std::string& classKey)
      : AbstractManagedPhysicsObject<T>(classKey) {}
  void applyForce(const Magnum::Vector3& force, const Magnum::Vector3& relPos) {
    if (auto sp = this->getObjectReference()) {
      sp->applyForce(force, relPos);
    }
  }  // applyForce
  void applyImpulse(const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos) {
    if (auto sp = this->getObjectReference()) {
      sp->applyImpulse(impulse, relPos);
    }
  }  // applyImpulse

  void applyTorque(const Magnum::Vector3& torque) {
    if (auto sp = this->getObjectReference()) {
      sp->applyTorque(torque);
    }
  }  // applyTorque

  void applyImpulseTorque(const Magnum::Vector3& impulse) {
    if (auto sp = this->getObjectReference()) {
      sp->applyImpulseTorque(impulse);
    }
  }  // applyImpulseTorque

  // ==== Getter/Setter functions ===

  double getAngularDamping() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getAngularDamping();
    }
    return 0.0;
  }  // getAngularDamping

  void setAngularDamping(const double angDamping) {
    if (auto sp = this->getObjectReference()) {
      sp->setAngularDamping(angDamping);
    }
  }  // setAngularDamping

  Magnum::Vector3 getAngularVelocity() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getAngularVelocity();
    }
    return Magnum::Vector3();
  }  // getAngularVelocity

  void setAngularVelocity(const Magnum::Vector3& angVel) {
    if (auto sp = this->getObjectReference()) {
      sp->setAngularVelocity(angVel);
    }
  }  // setAngularVelocity

  bool getCollidable() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getCollidable();
    }
    return false;
  }  // getCollidable()

  void setCollidable(bool collidable) {
    if (auto sp = this->getObjectReference()) {
      sp->setCollidable(collidable);
    }
  }  // setCollidable

  Magnum::Vector3 getCOM() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getCOM();
    }
    return Magnum::Vector3();
  }  // getCOM

  void setCOM(const Magnum::Vector3& COM) {
    if (auto sp = this->getObjectReference()) {
      sp->setCOM(COM);
    }
  }  // setCOM

  double getFrictionCoefficient() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getFrictionCoefficient();
    }
    return 0.0;
  }  // getFrictionCoefficient

  void setFrictionCoefficient(const double frictionCoefficient) {
    if (auto sp = this->getObjectReference()) {
      sp->setFrictionCoefficient(frictionCoefficient);
    }
  }  // setFrictionCoefficient

  Magnum::Matrix3 getInertiaMatrix() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getInertiaMatrix();
    }
    return Magnum::Matrix3();
  }  // getInertiaMatrix

  Magnum::Vector3 getInertiaVector() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getInertiaVector();
    }
    return Magnum::Vector3();
  }  // getInertiaVector

  void setInertiaVector(const Magnum::Vector3& inertia) {
    if (auto sp = this->getObjectReference()) {
      sp->setInertiaVector(inertia);
    }
  }  // setInertiaVector

  double getLinearDamping() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getLinearDamping();
    }
    return 0.0;
  }  // getLinearDamping

  void setLinearDamping(const double linDamping) {
    if (auto sp = this->getObjectReference()) {
      sp->setLinearDamping(linDamping);
    }
  }  // setLinearDamping

  Magnum::Vector3 getLinearVelocity() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getLinearVelocity();
    }
    return Magnum::Vector3();
  }  // getLinearVelocity

  void setLinearVelocity(const Magnum::Vector3& linVel) {
    if (auto sp = this->getObjectReference()) {
      sp->setLinearVelocity(linVel);
    }
  }  // setLinearVelocity

  double getMass() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getMass();
    }
    return 0.0;
  }  // getMass

  void setMass(const double mass) {
    if (auto sp = this->getObjectReference()) {
      sp->setMass(mass);
    }
  }  // setMass

  double getRestitutionCoefficient() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getRestitutionCoefficient();
    }
    return 0.0;
  }  // getRestitutionCoefficient

  void setRestitutionCoefficient(const double restitutionCoefficient) {
    if (auto sp = this->getObjectReference()) {
      sp->setRestitutionCoefficient(restitutionCoefficient);
    }
  }  // setRestitutionCoefficient

  Magnum::Vector3 getScale() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getScale();
    }
    return Magnum::Vector3();
  }  // getScale

  int getSemanticId() const {
    if (auto sp = this->getObjectReference()) {
      return sp->getSemanticId();
    }
    return 0;
  }  // getSemanticId

  void setSemanticId(uint32_t semanticId) {
    if (auto sp = this->getObjectReference()) {
      sp->setSemanticId(semanticId);
    }
  }  // setSemanticId

 public:
  ESP_SMART_POINTERS(AbstractManagedRigidBase<T>)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDRIGIDBASE_H_
