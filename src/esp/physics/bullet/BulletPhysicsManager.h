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
   * resources assets.
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
   * BulletArticulatedObject in the world.  This version does not require
   * drawables to be specified.
   * @param filepath The fully-qualified filename for the URDF file describing
   * the model the articulated object is to be built from.
   * @param fixedBase Whether the base of the @ref ArticulatedObject should be
   * fixed.
   * @param globalScale A scale multiplier to be applied uniformly in 3
   * dimensions to the entire @ref ArticulatedObject.
   * @param massScale A scale multiplier to be applied to the mass of the all
   * the components of the @ref ArticulatedObject.
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return A unique id for the @ref ArticulatedObject, allocated from the same
   * id set as rigid objects.
   */
  int addArticulatedObjectFromURDF(
      const std::string& filepath,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) override;

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * BulletArticulatedObject in the world.
   * @param filepath The fully-qualified filename for the URDF file describing
   * the model the articulated object is to be built from.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized @ref ArticulatedObject.
   * @param fixedBase Whether the base of the @ref ArticulatedObject should be
   * fixed.
   * @param globalScale A scale multiplier to be applied uniformly in 3
   * dimensions to the entire @ref ArticulatedObject.
   * @param massScale A scale multiplier to be applied to the mass of the all
   * the components of the @ref ArticulatedObject.
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return A unique id for the @ref ArticulatedObject, allocated from the same
   * id set as rigid objects.
   */
  int addArticulatedObjectFromURDF(
      const std::string& filepath,
      DrawableGroup* drawables,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) override;

  /**
   * @brief Use the metadata stored in io::URDF::Link to instance all visual
   * shapes for a link into the SceneGraph.
   *
   * @param linkObject The Habitat-side ArticulatedLink to which visual shapes
   * will be attached.
   * @param link The io::URDF::Model's link with visual shape and transform
   * metadata.
   * @param drawables The SceneGraph's DrawableGroup with which the visual
   * shapes will be rendered.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return Whether or not the render shape instancing was successful.
   */
  bool attachLinkGeometry(ArticulatedLink* linkObject,
                          const std::shared_ptr<io::URDF::Link>& link,
                          gfx::DrawableGroup* drawables,
                          const std::string& lightSetup);

  /**
   * @brief Override of @ref PhysicsManager::removeObject to also remove any
   * active Bullet physics constraints for the object.
   */
  void removeObject(const int physObjectID,
                    bool deleteObjectNode = true,
                    bool deleteVisualNode = true) override;

  /**
   * @brief Override of @ref PhysicsManager::removeArticulatedObject to also
   * remove any active Bullet physics constraints for the object.
   */
  void removeArticulatedObject(int id) override;

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
  Magnum::Range3D getCollisionShapeAabb(const int physObjectID) const;

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the static stage in its local space. See @ref btCompoundShape::getAabb.
   * @return The stage collision Aabb.
   */
  Magnum::Range3D getStageCollisionShapeAabb() const;

  /** @brief Render the debugging visualizations provided by @ref
   * Magnum::BulletIntegration::DebugDraw. This draws wireframes for all
   * collision objects.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  void debugDraw(const Magnum::Matrix4& projTrans) const override;

  /**
   * @brief Return ContactPointData objects describing the contacts from the
   * most recent physics substep.
   *
   * This implementation is roughly identical to PyBullet's getContactPoints.
   * @return a vector with each entry corresponding to a single contact point.
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

  /**
   * @brief Query the number of contact points that were active during the
   * collision detection check.
   *
   * An object resting on another object will involve several active contact
   * points. Once both objects are asleep, the contact points are inactive. This
   * count can be used as a metric for the complexity/cost of collision-handling
   * in the current scene.
   *
   * @return the number of active contact points.
   */
  int getNumActiveContactPoints() override;

  /**
   * @brief Perform discrete collision detection for the scene.
   */
  void performDiscreteCollisionDetection() override {
    bWorld_->getCollisionWorld()->performDiscreteCollisionDetection();
    recentNumSubStepsTaken_ = -1;  // TODO: handle this more gracefully
  }

  /**
   * @brief utilize PhysicsManager's enable shared
   */
  BulletPhysicsManager::ptr shared_from_this() {
    return esp::shared_from(this);
  }

 protected:
  //============ Initialization =============
  /**
   * @brief Finalize physics initialization: Setup staticStageObject_ and
   * initialize any other physics-related values.
   */
  bool initPhysicsFinalize() override;

  /**
   * @brief Create an object wrapper appropriate for this physics manager.
   * Overridden if called by dynamics-library-enabled PhysicsManager
   */
  esp::physics::ManagedRigidObject::ptr getRigidObjectWrapper() override;

  /**
   * @brief Create an articulated object wrapper appropriate for this physics
   * manager. Overridden if called by dynamics-library-enabled PhysicsManager
   */
  esp::physics::ManagedArticulatedObject::ptr getArticulatedObjectWrapper()
      override;

  //============ Object/Stage Instantiation =============
  /**
   * @brief Finalize stage initialization. Checks that the collision
   * mesh can be used by Bullet. See @ref BulletRigidObject::initializeStage.
   * Bullet mesh conversion adapted from:
   * https://github.com/mosra/magnum-integration/issues/20
   * @param initAttributes The attributes structure defining physical
   * properties of the stage.
   * @return true if successful and false otherwise
   */
  bool addStageFinalize(const metadata::attributes::StageAttributes::ptr&
                            initAttributes) override;

  /** @brief Create and initialize an @ref RigidObject and add
   * it to existingObjects_ map keyed with newObjectID
   * @param newObjectID valid object ID for the new object
   * @param objectAttributes The object's template
   * @param objectNode Valid, existing scene node
   * @return whether the object has been successfully initialized and added to
   * existingObjects_ map
   */
  bool makeAndAddRigidObject(
      int newObjectID,
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
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

  //! necessary to acquire forces from impulses
  double recentTimeStep_ = fixedTimeStep_;
  //! for recent call to stepPhysics
  int recentNumSubStepsTaken_ = -1;

 private:
  /** @brief Check if a particular mesh can be used as a collision mesh for
   * Bullet.
   * @param meshData The mesh to validate. Only a triangle mesh is valid. Checks
   * that the only #ref Magnum::MeshPrimitive are @ref
   * Magnum::MeshPrimitive::Triangles.
   * @return true if valid, false otherwise.
   */
  bool isMeshPrimitiveValid(const assets::CollisionMeshData& meshData) override;

  /**
   * @brief Helper function for getting object and link unique ids from
   * btCollisionObject cache
   *
   * @param colObj The query collision object for which a corresponding id is
   * desired.
   * @param objectId write found RigidObject or ArticulatedObject id or -1 if
   * failed.
   * @param linkId write found linkId or -1 if failed or not an ArticulatedLink.
   */
  void lookUpObjectIdAndLinkId(const btCollisionObject* colObj,
                               int* objectId,
                               int* linkId) const;

 public:
  ESP_SMART_POINTERS(BulletPhysicsManager)

};  // end class BulletPhysicsManager
}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETPHYSICSMANAGER_H_
