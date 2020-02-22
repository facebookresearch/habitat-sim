// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Struct SimulationContactResultCallback, class @ref
 * esp::physics::BulletRigidObject
 */

#include <btBulletDynamicsCommon.h>
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/core/esp.h"

#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

/**
@brief Implements Bullet physics @ref btCollisionWorld::ContactResultCallback
interface.

Stores the results of a collision check within the world.
*/
struct SimulationContactResultCallback
    : public btCollisionWorld::ContactResultCallback {
  /**
   * @brief Set when a contact is detected.
   */
  bool bCollision;

  /**
   * @brief Constructor.
   */
  SimulationContactResultCallback() { bCollision = false; }

  /**
   * @brief Called when a contact is detected.
   *
   * Sets a collision flag on every detected collision. Can be updated to do
   * more.
   * @param cp Contains detailed information about the contact point being
   * added.
   */
  btScalar addSingleResult(btManifoldPoint& cp,
                           const btCollisionObjectWrapper* colObj0Wrap,
                           int partId0,
                           int index0,
                           const btCollisionObjectWrapper* colObj1Wrap,
                           int partId1,
                           int index1) {
    bCollision = true;
    return 0;  // not used
  }
};

/**
@brief An individual rigid object instance implementing an interface with Bullet
physics to enable @ref MotionType::DYNAMIC objects.

See @ref btCollisionObject for @ref RigidObjectType::SCENE and
@ref btRigidBody for @ref RigidObjectType::OBJECT.

Utilizes Magnum::BulletIntegration::MotionState to syncronize SceneNode state
with internal btRigidBody state.

*/
class BulletRigidObject : public RigidObject,
                          public Magnum::BulletIntegration::MotionState {
 public:
  /**
   * @brief Constructor for a @ref BulletRigidObject.
   * @param rigidBodyNode The @ref scene::SceneNode this feature will be
   * attached to.
   */
  BulletRigidObject(scene::SceneNode* rigidBodyNode);

  /**
   * @brief Destructor cleans up simulation structures for the object.
   */
  virtual ~BulletRigidObject();

  /**
   * @brief Initializes this @ref BulletRigidObject as static scene geometry.
   * See @ref PhysicsManager::staticSceneObject_. Sets @ref rigidObjectType_ to
   * @ref RigidObjectType::SCENE. See @ref btCollisionObject.
   * @param physicsSceneAttributes The template structure defining relevant
   * phyiscal parameters for the physical scene.
   * @param meshGroup The collision mesh data for the scene.
   * @param bWorld The @ref btDiscreteDynamicsWorld to which the scene should
   * belong.
   * @return true if initialized successfully, false otherwise.
   */
  bool initializeScene(
      const assets::PhysicsSceneAttributes& physicsSceneAttributes,
      const assets::MeshMetaData& metaData,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  /**
   * @brief Initializes this @ref BulletRigidObject as a @ref
   * MotionType::DYNAMIC object. Sets @ref rigidObjectType_ to @ref
   * RigidObjectType::OBJECT. See @ref btRigidBody.
   * @param physicsObjectAttributes The template structure defining relevant
   * phyiscal parameters for the object. See @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_.
   * @param bWorld The @ref btDiscreteDynamicsWorld to which the object should
   * belong.
   * @param metaData Mesh transform hierarchy information for the object.
   * @param meshGroup The collision mesh data for the object.
   * @return true if initialized successfully, false otherwise.
   */
  bool initializeObject(
      const assets::PhysicsObjectAttributes& physicsObjectAttributes,
      std::shared_ptr<btDiscreteDynamicsWorld> bWorld,
      const assets::MeshMetaData& metaData,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  /**
   * @brief Recursively construct a @ref btCompoundShape for collision from
   * loaded mesh assets. A @ref btConvexHullShape is constructed for each
   * sub-component, transformed to object-local space and added to the compound
   * in a flat manner for efficiency.
   * @param bCompound The @ref btCompoundShape being constructed.
   * @param transformFromParentToWorld The cumulative parent-to-world
   * transformation matrix constructed by composition down the @ref
   * MeshTransformNode tree to the current node.
   * @param meshGroup Access structure for collision mesh data.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param join Whether or not to join sub-meshes into a single con convex
   * shape, rather than creating individual convexes under the compound.
   */
  void constructBulletCompoundFromMeshes(
      const Magnum::Matrix4& transformFromParentToWorld,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      const assets::MeshTransformNode& node,
      bool join);

  /**
   * @brief Check whether object is being actively simulated, or sleeping.
   * See @ref btCollisionObject::isActive.
   * @return true if active, false otherwise.
   */
  bool isActive() override;

  /**
   * @brief Set an object as being actively simulated rather than sleeping.
   * See @ref btCollisionObject::activate.
   */
  void setActive() override;

  /**
   * @brief Set the @ref MotionType of the object. If the object is @ref
   * ObjectType::SCENE it can only be @ref MotionType::STATIC. If the object is
   * @ref ObjectType::OBJECT is can also be set to @ref MotionType::KINEMATIC or
   * @ref MotionType::DYNAMIC. See @ref btRigidBody::setCollisionFlags and @ref
   * btCollisionObject::CF_STATIC_OBJECT,CF_KINEMATIC_OBJECT.
   * @param mt The desirved @ref MotionType.
   * @return true if successfully set, false otherwise.
   */
  virtual bool setMotionType(MotionType mt) override;

  /**
   * @brief Shift the object's local origin by translating all children of this
   * @ref BulletRigidObject and all components of its @ref bObjectShape_.
   * @param shift The translation to apply.
   */
  void shiftOrigin(const Magnum::Vector3& shift) override;

  /**
   * @brief Apply a force to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Calls @ref setActive().
   * See @ref btRigidBody::applyForce.
   * @param force The desired linear force on the object in the global
   * coordinate system.
   * @param relPos The desired location of force application in the global
   * coordinate system relative to the object's center of mass.
   */
  void applyForce(const Magnum::Vector3& force,
                  const Magnum::Vector3& relPos) override;

  /**
   * @brief Apply an impulse to an object.
   * Directly modifies the object's velocity without requiring
   * integration through simulation. Does nothing for @ref MotionType::STATIC
   * and @ref MotionType::KINEMATIC objects. Calls @ref setActive().
   * See @ref btRigidBody::applyImpulse.
   * @param impulse The desired impulse on the object in the global coordinate
   * system.
   * @param relPos The desired location of impulse application in the global
   * coordinate system relative to the object's center of mass.
   */
  void applyImpulse(const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos) override;

  /**
   * @brief Apply an internal torque to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Calls @ref setActive().
   * See @ref btRigidBody::applyTorque.
   * @param torque The desired torque on the object in the local coordinate
   * system.
   */
  void applyTorque(const Magnum::Vector3& torque) override;

  /**
   * @brief Apply an internal impulse torque to an object.
   * Does nothing for @ref MotionType::STATIC and @ref
   * MotionType::KINEMATIC objects. Calls @ref setActive().
   * See @ref btRigidBody::applyTorqueImpulse.
   * @param impulse The desired impulse torque on the object in the local
   * coordinate system. Directly modifies the object's angular velocity without
   * requiring integration through simulation.
   */
  void applyImpulseTorque(const Magnum::Vector3& impulse) override;

  /**
   * @brief Linear velocity setter for an object.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects. Sets internal @ref btRigidObject state. Treated as initial
   * velocity during simulation simulation step.
   * @param linVel Linear velocity to set.
   */
  void setLinearVelocity(const Magnum::Vector3& linVel) override;

  /**
   * @brief Angular velocity setter for an object.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects. Sets internal @ref btRigidObject state. Treated as initial
   * velocity during simulation simulation step.
   * @param angVel Angular velocity vector corresponding to world unit axis
   * angles.
   */
  void setAngularVelocity(const Magnum::Vector3& angVel) override;

  /**
   * @brief Virtual linear velocity getter for an object.
   *
   * @return Linear velocity of the object.
   */
  Magnum::Vector3 getLinearVelocity() const override;

  /**
   * @brief Angular velocity getter for an object.
   *
   * @return Angular velocity vector corresponding to world unit axis angles.
   */
  Magnum::Vector3 getAngularVelocity() const override;

  //============ Getter/setter function =============

  /** @brief Get the mass of the object. Returns 0.0 for @ref
   * RigidObjectType::SCENE. See @ref btRigidBody::getInvMass.
   * @return The mass of the object.
   */
  double getMass() override;

  /** @brief Get the center of mass (COM) of the object. For Bullet, COM is
   * always the origin of the local coordinate system. Return [0,0,0] for @ref
   * RigidObjectType::SCENE. See @ref btRigidBody::getCenterOfMassPosition.
   * @return Object 3D center of mass in the global coordinate system.
   */
  Magnum::Vector3 getCOM() override;

  /** @brief Get the diagonal of the inertia matrix for an object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. This is expected for Bullet. See @ref
   * BulletRigidObject::setInertiaVector. See @ref
   * btRigidBody::getInvInertiaDiagLocal.
   * @return The diagonal of the object's inertia matrix.
   */
  Magnum::Vector3 getInertiaVector() override;

  /** @brief Get the 3x3 inertia matrix for an object.
   * For Bullet, this will be a diagonal matrix. See @ref getInertiaVector.
   * @return The object's 3x3 inertia matrix.
   */
  Magnum::Matrix3 getInertiaMatrix() override;

  /** @brief Get the uniform scale of the object.
   * @return The scalar uniform scale for the object relative to its
   * initially loaded meshes.
   * @todo !!! not implemented properly!!!
   */
  double getScale() override;

  /** @brief Get the scalar friction coefficient of the object.
   * See @ref btCollisionObject::getFriction.
   * @return The scalar friction coefficient of the object.
   */
  double getFrictionCoefficient() override;

  /** @brief Get the scalar coefficient of restitution  of the object.
   * See @ref btCollisionObject::getRestitution.
   * @return The scalar coefficient of restitution  of the object.
   */
  double getRestitutionCoefficient() override;

  /** @brief Get the scalar linear damping coefficient of the object.
   * See @ref btRigidBody::getLinearDamping.
   * @return The scalar linear damping coefficient of the object. 0.0 for @ref
   * RigidObjectType::SCENE.
   */
  double getLinearDamping() override;

  /** @brief Get the scalar angular damping coefficient of the object.
   * See @ref btRigidBody::getAngularDamping.
   * @return The scalar angular damping coefficient of the object. 0.0 for @ref
   * RigidObjectType::SCENE.
   */
  double getAngularDamping() override;

  /** @brief Get the scalar collision margin of an object. Retun 0.0 for a @ref
   * RigidObjectType::SCENE. See @ref btCompoundShape::getMargin.
   * @return The scalar collision margin of the object.
   */
  double getMargin();

  /** @brief Set the mass of the object.
   * See @ref btRigidBody::setMassProps. Note that changing mass should affect
   * inertia, but this is not done automatically. Does not affect @ref
   * RigidObjectType::SCENE.
   * @param mass The new mass of the object.
   */
  void setMass(const double mass) override;

  /** @brief Set the center of mass (COM) of the object.
   * @param COM Object 3D center of mass in the local coordinate system.
   * !!! Currently not supported !!!
   * All Bullet @ref btRigidBody objects must have a COM located at thier local
   * origins.
   */
  void setCOM(const Magnum::Vector3& COM) override;

  /** @brief Set the diagonal of the inertia matrix for the object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. This is the requirement for Bullet
   * @ref btRigidBody objects. See @ref btRigidBody::setMassProps. Does not
   * affect @ref RigidObjectType::SCENE.
   * @param inertia The new diagonal for the object's inertia matrix.
   */
  void setInertiaVector(const Magnum::Vector3& inertia) override;

  /** @brief Set the uniform scale of the object.
   * @param scale The new scalar uniform scale for the object relative to its
   * initially loaded meshes.
   * @todo !!! not implemented !!!
   */
  void setScale(const double scale) override;

  /** @brief Set the scalar friction coefficient of the object.
   * See @ref btCollisionObject::setFriction.
   * @param frictionCoefficient The new scalar friction coefficient of the
   * object.
   */
  void setFrictionCoefficient(const double frictionCoefficient) override;

  /** @brief Set the scalar coefficient of restitution of the object.
   * See @ref btCollisionObject::setRestitution.
   * @param restitutionCoefficient The new scalar coefficient of restitution of
   * the object.
   */
  void setRestitutionCoefficient(const double restitutionCoefficient) override;

  /** @brief Set the scalar linear damping coefficient of the object.
   * See @ref btRigidBody::setDamping. Does not affect @ref
   * RigidObjectType::SCENE.
   * @param linearDamping The new scalar linear damping coefficient of the
   * object.
   */
  void setLinearDamping(const double linearDamping) override;

  /** @brief Set the scalar angular damping coefficient for the object.
   * See @ref btRigidBody::setDamping. Does not affect @ref
   * RigidObjectType::SCENE.
   * @param angularDamping The new scalar angular damping coefficient for the
   * object.
   */
  void setAngularDamping(const double angularDamping) override;

  /** @brief Set the scalar collision margin of an object. Does not affect @ref
   * RigidObjectType::SCENE. See @ref btCompoundShape::setMargin.
   * @param margin The new scalar collision margin of the object.
   */
  void setMargin(const double margin);

  /** @brief ets the object's collision shape to its bounding box.
   * Since the bounding hierarchy is not constructed when the object is
   * initialized, this needs to be called after loading the SceneNode.
   */
  void setCollisionFromBB();

  /** @brief Public getter for @ref usingBBCollisionShape_ set from
   * configuration.
   * @return @ref usingBBCollisionShape_ is true if "useBoundingBoxForCollision"
   * was set in object's configuration.
   */
  const bool isUsingBBCollisionShape() const { return usingBBCollisionShape_; };

  /**
   * @brief Return result of a discrete contact test between the object and
   * collision world.
   *
   * See @ref SimulationContactResultCallback
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest();

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @return The Aabb.
   */
  const Magnum::Range3D getCollisionShapeAabb() const;

 protected:
  /**
   * @brief Used to synchronize Bullet's notion of the object state
   * after it was changed kinematically. Called automatically on kinematic
   * updates. See @ref btRigidBody::setWorldTransform. */
  void syncPose() override;

  //! If true, the object's bounding box will be used for collision once
  //! computed
  bool usingBBCollisionShape_ = false;

 private:
  /** @brief A pointer to the Bullet world to which this object belongs. See
   * @ref btDiscreteDynamicsWorld.*/
  std::shared_ptr<btDiscreteDynamicsWorld> bWorld_;

  // === Physical scene ===

  //! Scene data: Bullet triangular mesh vertices
  std::unique_ptr<btTriangleIndexVertexArray> bSceneArray_;

  //! Scene data: Bullet triangular mesh shape
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneShapes_;

  /** @brief Scene data: All components of a @ref RigidObjectType::SCENE are
   * stored here. See @ref btCollisionObject.
   */
  std::vector<std::unique_ptr<btCollisionObject>> bSceneCollisionObjects_;

  // === Physical object ===

  //! Object data: Composite convex collision shape
  std::vector<std::unique_ptr<btConvexHullShape>> bObjectConvexShapes_;

  //! list of @ref btCollisionShape for storing arbitrary collision shapes
  //! referenced within the @ref bObjectShape_.
  std::vector<std::unique_ptr<btCollisionShape>> bGenericShapes_;

  //! Object data: All components of the collision shape
  std::unique_ptr<btCompoundShape> bObjectShape_;

  /** @brief Object data: All components of a @ref RigidObjectType::OBJECT are
   * wrapped into one @ref btRigidBody.
   */
  std::unique_ptr<btRigidBody> bObjectRigidBody_;

  ESP_SMART_POINTERS(BulletRigidObject)
};

}  // namespace physics
}  // namespace esp
