// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETPHYSICSMANAGER_H_
#define ESP_PHYSICS_BULLET_BULLETPHYSICSMANAGER_H_

/** @file
 * @brief Class @ref esp::physics::BulletPhysicsManager
 */

/* Bullet Physics Integration */
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <btBulletDynamicsCommon.h>

#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletDebugManager.h"
#include "BulletRigidObject.h"
#include "BulletRigidStage.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletRigidObject.h"

namespace esp {
namespace physics {

/**
@brief Dynamic stage and object manager interfacing with Bullet physics
engine: https://github.com/bulletphysics/bullet3.

See @ref btMultiBodyDynamicsWorld.

Enables @ref RigidObject simulation with @ref MotionType::DYNAMIC.

This class handles initialization and stepping of the world as well as getting
and setting global simulation parameters. The @ref BulletRigidObject class
handles most of the specific implementations for object interactions with
Bullet.
*/
class BulletPhysicsManager : public PhysicsManager {
 public:
  /**
   * @brief Construct a @ref BulletPhysicsManager with access to specific
   * resourse assets.
   *
   * @param _resourceManager The @ref esp::assets::ResourceManager which
   * tracks the assets this
   * @ref BulletPhysicsManager will have access to.
   */
  explicit BulletPhysicsManager(
      assets::ResourceManager& _resourceManager,
      const metadata::attributes::PhysicsManagerAttributes::cptr&
          _physicsManagerAttributes);

  /** @brief Destructor which destructs necessary Bullet physics structures.*/
  ~BulletPhysicsManager() override;

  //============ Simulator functions =============

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * BulletArticulatedObject in the world.
   *
   * @return A unique id for the @ref BulletArticulatedObject, allocated from
   * the same id set as rigid objects.
   */
  virtual int addArticulatedObjectFromURDF(const std::string& filepath,
                                           DrawableGroup* drawables,
                                           bool fixedBase = false,
                                           float globalScale = 1.0,
                                           float massScale = 1.0,
                                           bool forceReload = false) override;

  /** @brief Step the physical world forward in time. Time may only advance in
   * increments of @ref fixedTimeStep_. See @ref
   * btMultiBodyDynamicsWorld::stepSimulation.
   * @param dt The desired amount of time to advance the physical world.
   */
  void stepPhysics(double dt) override;

  /** @brief Set the gravity of the physical world.
   * @param gravity The desired gravity force of the physical world.
   */
  void setGravity(const Magnum::Vector3& gravity) override;

  /** @brief Get the current gravity in the physical world.
   * @return The current gravity vector in the physical world.
   */
  Magnum::Vector3 getGravity() const override;

  //============ Interacting with objects =============
  // NOTE: engine specifics for interaction are handled by the objects
  // themselves...

  //============ Bullet-specific Object Setter functions =============

  /** @brief Set the scalar collision margin of an object.
   * See @ref BulletRigidObject::setMargin.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param  margin The desired collision margin for the object.
   */
  void setMargin(const int physObjectID, const double margin) override;

  /** @brief Set the friction coefficient of the stage collision geometry. See
   * @ref staticStageObject_. See @ref
   * BulletRigidObject::setFrictionCoefficient.
   * @param frictionCoefficient The scalar friction coefficient of the stage
   * geometry.
   */
  void setStageFrictionCoefficient(const double frictionCoefficient) override;

  /** @brief Set the coefficient of restitution for the stage collision
   * geometry. See @ref staticStageObject_. See @ref
   * BulletRigidObject::setRestitutionCoefficient.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  void setStageRestitutionCoefficient(
      const double restitutionCoefficient) override;

  //============ Bullet-specific Object Getter functions =============

  /** @brief Get the scalar collision margin of an object.
   * See @ref BulletRigidObject::getMargin.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar collision margin of the object or @ref
   * esp::PHYSICS_ATTR_UNDEFINED if failed..
   */
  double getMargin(const int physObjectID) const override;

  /** @brief Get the current friction coefficient of the stage collision
   * geometry. See @ref staticStageObject_ and @ref
   * BulletRigidObject::getFrictionCoefficient.
   * @return The scalar friction coefficient of the stage geometry.
   */
  double getStageFrictionCoefficient() const override;

  /** @brief Get the current coefficient of restitution for the stage
   * collision geometry. This determines the ratio of initial to final relative
   * velocity between the stage and collidiing object. See @ref
   * staticStageObject_ and BulletRigidObject::getRestitutionCoefficient.
   * @return The scalar coefficient of restitution for the stage geometry.
   */
  double getStageRestitutionCoefficient() const override;

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of a
   * rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The Aabb.
   */
  const Magnum::Range3D getCollisionShapeAabb(const int physObjectID) const;

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the static stage in its local space. See @ref btCompoundShape::getAabb.
   * @return The stage collision Aabb.
   */
  const Magnum::Range3D getStageCollisionShapeAabb() const;

  /** @brief Render the debugging visualizations provided by @ref
   * Magnum::BulletIntegration::DebugDraw. This draws wireframes for all
   * collision objects.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  void debugDraw(const Magnum::Matrix4& projTrans) const override;

  /**
   * @brief Check whether an object is in contact with any other objects or the
   * stage.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest(const int physObjectID) override;

  // TODO: document
  void overrideCollisionGroup(const int physObjectID,
                              CollisionGroup group) const override;

  /**
   * @brief Return ContactPointData objects describing the contacts from the
   * most recent physics substep. This implementation is roughly identical to
   * PyBullet's getContactPoints.
   */
  std::vector<ContactPointData> getContactPoints() const override;

  /**
   * @brief Cast a ray into the collision world and return a @ref RaycastResults
   * with hit information.
   *
   * @param ray The ray to cast. Need not be unit length, but returned hit
   * distances will be in units of ray length.
   * @param maxDistance The maximum distance along the ray direction to search.
   * In units of ray length.
   * @return The raycast results sorted by distance.
   */
  RaycastResults castRay(const esp::geo::Ray& ray,
                         double maxDistance = 100.0) override;

  //============ Point To Point Constraints =============

  /**
   * @brief Create a ball&socket joint to constrain a DYNAMIC RigidObject
   * provided a position in local or global coordinates.
   * @param objectId The id of the RigidObject to constrain.
   * @param position The position of the ball and socket joint pivot.
   * @param positionLocal Indicates whether the position is provided in global
   * or object local coordinates.
   * @return The unique id of the new constraint.
   */
  int createRigidP2PConstraint(int objectId,
                               const Magnum::Vector3& position,
                               bool positionLocal = true) override;

  // point2point constraint between multibody and rigid body
  int createArticulatedP2PConstraint(
      int articulatedObjectId,
      int linkId,
      int objectId,
      float maxImpulse,
      const Corrade::Containers::Optional<Magnum::Vector3>& pivotA,
      const Corrade::Containers::Optional<Magnum::Vector3>& pivotB) override;

  int createArticulatedFixedConstraint(
      int articulatedObjectId,
      int linkId,
      int objectId,
      float maxImpulse,
      const Corrade::Containers::Optional<Magnum::Vector3>& pivotA,
      const Corrade::Containers::Optional<Magnum::Vector3>& pivotB) override;

  /**
   * @brief Create a ball&socket joint to constrain two links of two
   * ArticulatedObjects to one another with local offsets for each.
   * @param articulatedObjectIdA The id of the first ArticulatedObject to
   * constrain.
   * @param linkIdA The local id of the first ArticulatedLink to constrain.
   * @param linkOffsetA The position of the first ball and socket joint pivot in
   * link A local space.
   * @param articulatedObjectIdB The id of the second ArticulatedObject to
   * constrain.
   * @param linkIdB The local id of the second ArticulatedLink to constrain.
   * @param linkOffsetB The position of the ball and socket joint pivot in link
   * B local space.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(int articulatedObjectIdA,
                                             int linkIdA,
                                             const Magnum::Vector3& linkOffsetA,
                                             int articulatedObjectIdB,
                                             int linkIdB,
                                             const Magnum::Vector3& linkOffsetB,
                                             float maxImpulse = 2.0) override;

  /**
   * @brief Create a ball&socket joint to constrain two links of two
   * ArticulatedObjects to one another at some global point.
   * @param articulatedObjectIdA The id of the first ArticulatedObject to
   * constrain.
   * @param linkIdA The local id of the first ArticulatedLink to constrain.
   * @param articulatedObjectIdB The id of the second ArticulatedObject to
   * constrain.
   * @param linkIdB The local id of the second ArticulatedLink to constrain.
   * @param globalConstraintPoint The position of the ball and socket joint
   * pivot in global space.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(
      int articulatedObjectIdA,
      int linkIdA,
      int articulatedObjectIdB,
      int linkIdB,
      const Magnum::Vector3& globalConstraintPoint,
      float maxImpulse = 2.0) override;

  /**
   * @brief Create a ball&socket joint to constrain a single link of an
   * ArticulatedObject provided a position in global coordinates and a local
   * offset.
   * @param articulatedObjectId The id of the ArticulatedObject to constrain.
   * @param linkId The local id of the ArticulatedLink to constrain.
   * @param linkOffset The position of the ball and socket joint pivot in link
   * local coordinates.
   * @param pickPos The global position of the ball and socket joint pivot.
   * @return The unique id of the new constraint.
   */
  int createArticulatedP2PConstraint(int articulatedObjectId,
                                     int linkId,
                                     const Magnum::Vector3& linkOffset,
                                     const Magnum::Vector3& pickPos,
                                     float maxImpulse = 2.0) override;

  /**
   * @brief Create a ball&socket joint to constrain a single link of an
   * ArticulatedObject provided a position in global coordinates.
   * @param articulatedObjectId The id of the ArticulatedObject to constrain.
   * @param linkId The local id of the ArticulatedLink to constrain.
   * @param pickPos The global position of the ball and socket joint pivot.
   * @return The unique id of the new constraint.
   */
  int createArticulatedP2PConstraint(int articulatedObjectId,
                                     int linkId,
                                     const Magnum::Vector3& pickPos,
                                     float maxImpulse = 2.0) override;

  /**
   * @brief Update the position target (pivot) of a constraint. Note: intended
   * only for use with (object -> world) constraints, rather than (object <->
   * object) constraints.
   * @param p2pId The id of the constraint to update.
   * @param pivot The new position target of the constraint.
   */
  void updateP2PConstraintPivot(int p2pId,
                                const Magnum::Vector3& pivot) override;

  /**
   * @brief Remove a constraint by id.
   * @param constraintId The id of the constraint to remove.
   */
  void removeConstraint(int constraintId) override;

  int nextConstraintId_ = 0;
  std::map<int, btMultiBodyPoint2Point*> articulatedP2ps;
  std::map<int, btMultiBodyFixedConstraint*> articulatedFixedConstraints;
  std::map<int, btPoint2PointConstraint*> rigidP2ps;

  int getNumActiveContactPoints() override {
    return BulletDebugManager::get().getNumActiveContactPoints(bWorld_.get());
  }
  int getNumActiveOverlappingPairs() override {
    return BulletDebugManager::get().getNumActiveOverlappingPairs(
        bWorld_.get());
  }
  std::string getStepCollisionSummary() override {
    return BulletDebugManager::get().getStepCollisionSummary(bWorld_.get());
  }

  /**
   * @brief Perform discrete collision detection for the scene.
   */
  virtual void performDiscreteCollisionDetection() override {
    bWorld_->getCollisionWorld()->performDiscreteCollisionDetection();
    m_recentNumSubStepsTaken = -1;  // TODO: handle this more gracefully
  };

 protected:
  //============ Initialization =============
  /**
   * @brief Finalize physics initialization: Setup staticStageObject_ and
   * initialize any other physics-related values.
   */
  bool initPhysicsFinalize() override;

  //============ Object/Stage Instantiation =============
  /**
   * @brief Finalize stage initialization. Checks that the collision
   * mesh can be used by Bullet. See @ref BulletRigidObject::initializeStage.
   * Bullet mesh conversion adapted from:
   * https://github.com/mosra/magnum-integration/issues/20
   * @param handle The handle of the attributes structure defining physical
   * properties of the stage.
   * @return true if successful and false otherwise
   */
  bool addStageFinalize(const std::string& handle) override;

  /** @brief Create and initialize an @ref RigidObject and add
   * it to existingObjects_ map keyed with newObjectID
   * @param newObjectID valid object ID for the new object
   * @param meshGroup The object's mesh.
   * @param handle The handle to the physical object's template defining its
   * physical parameters.
   * @param objectNode Valid, existing scene node
   * @return whether the object has been successfully initialized and added to
   * existingObjects_ map
   */
  bool makeAndAddRigidObject(int newObjectID,
                             const std::string& handle,
                             scene::SceneNode* objectNode) override;

  btDbvtBroadphase bBroadphase_;
  btDefaultCollisionConfiguration bCollisionConfig_;

  btMultiBodyConstraintSolver bSolver_;
  btCollisionDispatcher bDispatcher_{&bCollisionConfig_};

  /** @brief A pointer to the Bullet world. See @ref btMultiBodyDynamicsWorld.*/
  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  mutable Magnum::BulletIntegration::DebugDraw debugDrawer_;

  //! keep a map of collision objects to object ids for quick lookups from
  //! Bullet collision checking.
  std::shared_ptr<std::map<const btCollisionObject*, int>>
      collisionObjToObjIds_;

  int m_recentNumSubStepsTaken = -1;  // for recent call to stepPhysics

 private:
  /** @brief Check if a particular mesh can be used as a collision mesh for
   * Bullet.
   * @param meshData The mesh to validate. Only a triangle mesh is valid. Checks
   * that the only #ref Magnum::MeshPrimitive are @ref
   * Magnum::MeshPrimitive::Triangles.
   * @return true if valid, false otherwise.
   */
  bool isMeshPrimitiveValid(const assets::CollisionMeshData& meshData) override;

  void lookUpObjectIdAndLinkId(const btCollisionObject* colObj,
                               int* objectId,
                               int* linkId) const;

  ESP_SMART_POINTERS(BulletPhysicsManager)

};  // end class BulletPhysicsManager
}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETPHYSICSMANAGER_H_
