// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETRIGIDOBJECT_H_
#define ESP_PHYSICS_BULLET_BULLETRIGIDOBJECT_H_

/** @file
 * @brief Struct SimulationContactResultCallback, class @ref
 * esp::physics::BulletRigidObject
 */

#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>

#include <Magnum/BulletIntegration/MotionState.h>
#include <btBulletDynamicsCommon.h>

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "esp/core/Esp.h"

#include "esp/physics/CollisionGroupHelper.h"
#include "esp/physics/RigidObject.h"
#include "esp/physics/bullet/BulletBase.h"

namespace esp {
namespace physics {

/**
 * @brief An individual rigid object instance implementing an interface with
 * Bullet physics to enable dynamic objects. See @ref btRigidBody for @ref
 * esp::physics::RigidObjectType::OBJECT.
 *
 * Utilizes Magnum::BulletIntegration::MotionState to synchronize SceneNode
 * state with internal btRigidBody states
 */
class BulletRigidObject : public BulletBase,
                          public RigidObject,
                          public Magnum::BulletIntegration::MotionState {
 public:
  /**
   * @brief Constructor for a @ref BulletRigidObject.
   * @param rigidBodyNode The @ref scene::SceneNode this feature will be
   * attached to.
   * @param objectId The unique ID for referencing this object.
   * @param resMgr Reference to resource manager, to access relevant components
   * pertaining to the scene object
   * @param bWorld The Bullet world to which this object will belong.
   * @param collisionObjToObjIds The global map of btCollisionObjects to Habitat
   * object IDs for contact query identification.
   */
  BulletRigidObject(scene::SceneNode* rigidBodyNode,
                    int objectId,
                    const assets::ResourceManager& resMgr,
                    std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                    std::shared_ptr<std::map<const btCollisionObject*, int>>
                        collisionObjToObjIds);

  /**
   * @brief Destructor cleans up simulation structures for the object.
   */
  ~BulletRigidObject() override;

  /**
   * @brief Finalize this object with any necessary post-creation processes.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override;

  /**
   * @brief Instantiate a bullet primtive appropriate for the passed
   * AbstractPrimitiveAttributes object
   * @param primTypeVal int value corresponding to assets::PrimObjTypes enum
   * describing primitive collision shape.
   * @param halfLength half length of object, for primitives using this value
   * @return a unique pointer to the bullet primitive object
   */
  std::unique_ptr<btCollisionShape> buildPrimitiveCollisionObject(
      int primTypeVal,
      double halfLength);

  /**
   * @brief Construct the @ref bObjectShape_ for this object.
   * @return Whether or not construction was successful.
   */
  bool constructCollisionShape();

  /**
   * @brief Check whether object is being actively simulated, or sleeping.
   * See @ref btCollisionObject::isActive.
   * @return true if active, false otherwise.
   */
  bool isActive() const override { return bObjectRigidBody_->isActive(); }

  /**
   * @brief Set the object to sleep or wake.
   *
   * @param active Whether to active or sleep the object
   */
  void setActive(bool active) override {
    if (!active) {
      if (bObjectRigidBody_->isActive()) {
        bObjectRigidBody_->setActivationState(WANTS_DEACTIVATION);
      }
    } else {
      bObjectRigidBody_->activate(true);
    }
  }

  /** @brief Disable deferred updates if active and sets SceneNode states from
   * internal object physics states.
   * @param force If set, update sleeping objects as well.
   */
  void updateNodes(bool force = false) override;

  /**
   * @brief Set the @ref MotionType of the object. The object can be set to @ref
   * MotionType::STATIC, @ref MotionType::KINEMATIC or @ref MotionType::DYNAMIC.
   * See @ref btRigidBody::setCollisionFlags and @ref
   * btCollisionObject::CF_STATIC_OBJECT,CF_KINEMATIC_OBJECT.
   *
   * @param mt The desirved @ref MotionType.
   */
  void setMotionType(MotionType mt) override;

  /**
   * Set the object to be collidable or not by selectively adding or remove the
   * @ref bObjectShape_ from the @ref bRigidObject_.
   */
  void setCollidable(bool collidable) override;

  /**
   * @brief Shift the object's local origin by translating all children of this
   * @ref BulletRigidObject and all components of its @ref bObjectShape_.
   * @param shift The translation to apply.
   */
  void shiftOrigin(const Magnum::Vector3& shift) override;

  /**
   * @brief Apply a force to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Activates the object.
   * See @ref btRigidBody::applyForce.
   * @param force The desired linear force on the object in the global
   * coordinate system.
   * @param relPos The desired location of force application in the global
   * coordinate system relative to the object's center of mass.
   */
  void applyForce(const Magnum::Vector3& force,
                  const Magnum::Vector3& relPos) override {
    if (objectMotionType_ == MotionType::DYNAMIC) {
      setActive(true);
      bObjectRigidBody_->applyForce(btVector3(force), btVector3(relPos));
    }
  }

  /**
   * @brief Apply an impulse to an object.
   * Directly modifies the object's velocity without requiring
   * integration through simulation. Does nothing for @ref MotionType::STATIC
   * and @ref MotionType::KINEMATIC objects. Activates the object.
   * See @ref btRigidBody::applyImpulse.
   * @param impulse The desired impulse on the object in the global coordinate
   * system.
   * @param relPos The desired location of impulse application in the global
   * coordinate system relative to the object's center of mass.
   */
  void applyImpulse(const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos) override {
    if (objectMotionType_ == MotionType::DYNAMIC) {
      setActive(true);
      bObjectRigidBody_->applyImpulse(btVector3(impulse), btVector3(relPos));
    }
  }

  /**
   * @brief Apply an internal torque to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Activates the object.
   * See @ref btRigidBody::applyTorque.
   * @param torque The desired torque on the object in the local coordinate
   * system.
   */
  void applyTorque(const Magnum::Vector3& torque) override {
    if (objectMotionType_ == MotionType::DYNAMIC) {
      setActive(true);
      bObjectRigidBody_->applyTorque(btVector3(torque));
    }
  }

  /**
   * @brief Apply an internal impulse torque to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Activates the object.
   * See @ref btRigidBody::applyTorqueImpulse.
   * @param impulse The desired impulse torque on the object in the local
   * coordinate system. Directly modifies the object's angular velocity without
   * requiring integration through simulation.
   */
  void applyImpulseTorque(const Magnum::Vector3& impulse) override {
    if (objectMotionType_ == MotionType::DYNAMIC) {
      setActive(true);
      bObjectRigidBody_->applyTorqueImpulse(btVector3(impulse));
    }
  }

  //============ Getter/setter function =============

  /**
   * @brief Virtual linear velocity getter for an object.
   *
   * @return Linear velocity of the object.
   */
  Magnum::Vector3 getLinearVelocity() const override {
    return Magnum::Vector3{bObjectRigidBody_->getLinearVelocity()};
  }
  /**
   * @brief Angular velocity getter for an object.
   *
   * @return Angular velocity vector corresponding to world unit axis angles.
   */
  Magnum::Vector3 getAngularVelocity() const override {
    return Magnum::Vector3{bObjectRigidBody_->getAngularVelocity()};
  }

  /** @brief Get the mass of the object. See @ref btRigidBody::getInvMass.
   * @return The mass of the object.
   */
  double getMass() const override {
    return static_cast<double>(1.0f / bObjectRigidBody_->getInvMass());
  }

  /** @brief Get the center of mass (COM) of the object. For Bullet, COM is
   * always the origin of the local coordinate system. See @ref
   * btRigidBody::getCenterOfMassPosition.
   * @return Object 3D center of mass in the global coordinate system.
   */
  Magnum::Vector3 getCOM() const override;

  /** @brief Get the diagonal of the inertia matrix for an object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. This is expected for Bullet. See @ref
   * BulletRigidObject::setInertiaVector. See @ref
   * btRigidBody::getInvInertiaDiagLocal.
   * @return The diagonal of the object's inertia matrix.
   */
  Magnum::Vector3 getInertiaVector() const override {
    const Magnum::Vector3 inertia =
        1.0 / Magnum::Vector3(bObjectRigidBody_->getInvInertiaDiagLocal());
    return inertia;
  }

  /** @brief Get the 3x3 inertia matrix for an object.
   * For Bullet, this will be a diagonal matrix. See @ref getInertiaVector.
   * @return The object's 3x3 inertia matrix.
   */
  Magnum::Matrix3 getInertiaMatrix() const override {
    const Magnum::Vector3 vecInertia = getInertiaVector();
    const Magnum::Matrix3 inertia = Magnum::Matrix3::fromDiagonal(vecInertia);
    return inertia;
  }

  /** @brief Get the scalar friction coefficient of the object.
   * See @ref btCollisionObject::getFriction.
   * @return The scalar friction coefficient of the object.
   */
  double getFrictionCoefficient() const override {
    return static_cast<double>(bObjectRigidBody_->getFriction());
  }

  /** @brief Get the scalar rolling friction coefficient of the object.
   * See @ref btCollisionObject::getRollingFriction.
   * @return The scalar rolling friction coefficient of the object. Damps
   * angular velocity about axis orthogonal to the contact normal to prevent
   * rounded shapes from rolling forever.
   */
  double getRollingFrictionCoefficient() const override {
    return static_cast<double>(bObjectRigidBody_->getRollingFriction());
  }

  /** @brief Get the scalar spinning friction coefficient of the object.
   * See @ref btCollisionObject::getSpinningFriction.
   * @return The scalar spinning friction coefficient of the object. Damps
   * angular velocity about the contact normal.
   */
  double getSpinningFrictionCoefficient() const override {
    return static_cast<double>(bObjectRigidBody_->getSpinningFriction());
  }

  /** @brief Get the scalar coefficient of restitution  of the object.
   * See @ref btCollisionObject::getRestitution.
   * @return The scalar coefficient of restitution  of the object.
   */
  double getRestitutionCoefficient() const override {
    return static_cast<double>(bObjectRigidBody_->getRestitution());
  }

  /** @brief Get the scalar linear damping coefficient of the object.
   * See @ref btRigidBody::getLinearDamping.
   * @return The scalar linear damping coefficient of the object.
   */
  double getLinearDamping() const override {
    return static_cast<double>(bObjectRigidBody_->getLinearDamping());
  }

  /** @brief Get the scalar angular damping coefficient of the object.
   * See @ref btRigidBody::getAngularDamping.
   * @return The scalar angular damping coefficient of the object.
   */
  double getAngularDamping() const override {
    return static_cast<double>(bObjectRigidBody_->getAngularDamping());
  }

  /** @brief Get the scalar collision margin of an object. See @ref
   * btCompoundShape::getMargin.
   * @return The scalar collision margin of the object.
   */
  double getMargin() const override {
    return static_cast<double>(bObjectShape_->getMargin());
  }

  /**
   * @brief Linear velocity setter for an object.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects. Sets internal @ref btRigidObject state. Treated as initial
   * velocity during simulation simulation step. Activates the object.
   * @param linVel Linear velocity to set.
   */
  void setLinearVelocity(const Magnum::Vector3& linVel) override {
    if (objectMotionType_ != MotionType::STATIC) {
      setActive(true);
      bObjectRigidBody_->setLinearVelocity(btVector3(linVel));
    }
  }

  /**
   * @brief Angular velocity setter for an object.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects. Sets internal @ref btRigidObject state. Treated as initial
   * velocity during simulation simulation step. Activates the object.
   * @param angVel Angular velocity vector corresponding to world unit axis
   * angles.
   */
  void setAngularVelocity(const Magnum::Vector3& angVel) override {
    if (objectMotionType_ != MotionType::STATIC) {
      setActive(true);
      bObjectRigidBody_->setAngularVelocity(btVector3(angVel));
    }
  }

  /** @brief Set the mass of the object.
   * See @ref btRigidBody::setMassProps. Note that changing mass should affect
   * inertia, but this is not done automatically.
   * @param mass The new mass of the object.
   */
  void setMass(const double mass) override {
    bObjectRigidBody_->setMassProps(mass, btVector3(getInertiaVector()));
  }

  /** @brief Set the center of mass (COM) of the object.
   * @param COM Object 3D center of mass in the local coordinate system.
   * !!! Currently not supported !!!
   * All Bullet @ref btRigidBody objects must have a COM located at their local
   * origins.
   */
  void setCOM(const Magnum::Vector3& COM) override;

  /** @brief Set the diagonal of the inertia matrix for the object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. This is the requirement for Bullet
   * @ref btRigidBody objects. See @ref btRigidBody::setMassProps.
   * @param inertia The new diagonal for the object's inertia matrix.
   */
  void setInertiaVector(const Magnum::Vector3& inertia) override {
    bObjectRigidBody_->setMassProps(getMass(), btVector3(inertia));
  }

  /** @brief Set the scalar friction coefficient of the object.
   * See @ref btCollisionObject::setFriction.
   * @param frictionCoefficient The new scalar friction coefficient of the
   * object.
   */
  void setFrictionCoefficient(const double frictionCoefficient) override {
    bObjectRigidBody_->setFriction(frictionCoefficient);
  }

  /** @brief Set the scalar rolling friction coefficient of the object.
   * See @ref btCollisionObject::setRollingFriction.
   * @param rollingFrictionCoefficient The new scalar rolling friction
   * coefficient of the object. Damps angular velocity about axis orthogonal to
   * the contact normal to prevent rounded shapes from rolling forever.
   */
  void setRollingFrictionCoefficient(
      const double rollingFrictionCoefficient) override {
    bObjectRigidBody_->setRollingFriction(rollingFrictionCoefficient);
  }

  /** @brief Set the scalar spinning friction coefficient of the object.
   * See @ref btCollisionObject::setSpinningFriction.
   * @param spinningFrictionCoefficient The new scalar spinning friction
   * coefficient of the object. Damps angular velocity about the contact normal.
   */
  void setSpinningFrictionCoefficient(
      const double spinningFrictionCoefficient) override {
    bObjectRigidBody_->setSpinningFriction(spinningFrictionCoefficient);
  }

  /** @brief Set the scalar coefficient of restitution of the object.
   * See @ref btCollisionObject::setRestitution.
   * @param restitutionCoefficient The new scalar coefficient of restitution of
   * the object.
   */
  void setRestitutionCoefficient(const double restitutionCoefficient) override {
    bObjectRigidBody_->setRestitution(restitutionCoefficient);
  }

  /** @brief Set the scalar linear damping coefficient of the object.
   * See @ref btRigidBody::setDamping.
   * @param linearDamping The new scalar linear damping coefficient of the
   * object.
   */
  void setLinearDamping(const double linearDamping) override {
    bObjectRigidBody_->setDamping(linearDamping, getAngularDamping());
  }

  /** @brief Set the scalar angular damping coefficient for the object.
   * See @ref btRigidBody::setDamping.
   * @param angularDamping The new scalar angular damping coefficient for the
   * object.
   */
  void setAngularDamping(const double angularDamping) override {
    bObjectRigidBody_->setDamping(getLinearDamping(), angularDamping);
  }

  /** @brief Set the scalar collision margin of an object. See @ref
   * btCompoundShape::setMargin.
   * @param margin The new scalar collision margin of the object.
   */
  void setMargin(const double margin) override {
    for (std::size_t i = 0; i < bObjectConvexShapes_.size(); ++i) {
      bObjectConvexShapes_[i]->setMargin(margin);
    }
    bObjectShape_->setMargin(margin);
  }

  /** @brief Sets the object's collision shape to its bounding box.
   * Since the bounding hierarchy is not constructed when the object is
   * initialized, this needs to be called after loading the SceneNode.
   */
  void setCollisionFromBB();

  /** @brief Public getter for @ref usingBBCollisionShape_ set from
   * configuration.
   * @return @ref usingBBCollisionShape_ is true if "useBoundingBoxForCollision"
   * was set in object's configuration.
   */
  bool isUsingBBCollisionShape() const { return usingBBCollisionShape_; };

  /**
   * @brief Return result of a discrete contact test between the object and
   * collision world.
   *
   * See @ref SimulationContactResultCallback
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest() override;

  /**
   * @brief Manually set the collision group for an object.
   * @param group The desired CollisionGroup for the object.
   */
  void overrideCollisionGroup(CollisionGroup group) override;

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @return The Aabb.
   */
  Magnum::Range3D getCollisionShapeAabb() const override;

  /** @brief Object data: All components of a @ref RigidObjectType::OBJECT are
   * wrapped into one @ref btRigidBody.
   */
  std::unique_ptr<btRigidBody> bObjectRigidBody_;

 private:
  /**
   * @brief Finalize initialization of this @ref BulletRigidObject as a @ref
   * MotionType::DYNAMIC object. See @ref btRigidBody. This holds
   * bullet-specific functionality for objects.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override;

 protected:
  /**
   * @brief Used to synchronize Bullet's notion of the object state
   * after it was changed kinematically. Called automatically on kinematic
   * updates. See @ref btRigidBody::setWorldTransform. */
  void syncPose() override;

  /**
   * @brief construct a @ref btRigidBody for this object configured by
   * MotionType and add it to the world.
   */
  void constructAndAddRigidBody(MotionType mt);

  /**
   * @brief shift all child shapes of the @ref bObjectShape_ to modify collision
   * shape origin.
   */
  void shiftObjectCollisionShape(const Magnum::Vector3& shift);

  /**
   * @brief Iterate through all collision objects and active all objects sharing
   * a collision island tag with this object's collision shape.
   */
  void activateCollisionIsland();

 private:
  // === Physical object ===
  //! If true, the object's bounding box will be used for collision once
  //! computed
  bool usingBBCollisionShape_ = false;

  //! cache the origin shift applied to the object during construction for
  //! deffered construction of collision shape
  Mn::Vector3 originShift_;

  //! Object data: All components of the collision shape
  std::unique_ptr<btCompoundShape> bObjectShape_;

  std::unique_ptr<btCompoundShape> bEmptyShape_;

  void setWorldTransform(const btTransform& worldTrans) override;

  void getWorldTransform(btTransform& worldTrans) const override;

  std::string getCollisionDebugName();

  Corrade::Containers::Optional<btTransform> deferredUpdate_ =
      Corrade::Containers::NullOpt;

  ESP_SMART_POINTERS(BulletRigidObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETRIGIDOBJECT_H_
