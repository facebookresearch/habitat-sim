// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

#include "BulletDynamics/ConstraintSolver/btFixedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h"
#include "BulletDynamics/Featherstone/btMultiBodyPoint2Point.h"

#include "BulletCollisionHelper.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletArticulatedObject.h"

namespace esp {
namespace physics {
class BulletArticulatedObject;

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
      const std::shared_ptr<
          const metadata::attributes::PhysicsManagerAttributes>&
          _physicsManagerAttributes);

  /** @brief Destructor which destructs necessary Bullet physics structures.*/
  ~BulletPhysicsManager() override;

  //============ Simulator functions =============

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * ArticulatedObject in the world based on the urdf filepath specified in @ref
   * esp::metadata::attributes::ArticulatedObjectAttributes. This version
   * requires drawables to be provided.
   *
   * @param artObjAttributes The @ref ArticulatedObject's template to use to create it.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized @ref ArticulatedObject.
   * @param forceReload If true, force the reload of the source URDF from file,
   * replacing the cached model if it exists.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObject(
      const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
          artObjAttributes,
      DrawableGroup* drawables,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) override;

  /**
   * @brief Use the metadata stored in metadata::URDF::Link to instance all
   * visual shapes for a link into the SceneGraph.
   *
   * @param linkObject The Habitat-side ArticulatedLink to which visual shapes
   * will be attached.
   * @param link The metadata::URDF::Model's link with visual shape and
   * transform metadata.
   * @param drawables The SceneGraph's DrawableGroup with which the visual
   * shapes will be rendered.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return Whether or not the render shape instancing was successful.
   */
  bool attachLinkGeometry(ArticulatedLink* linkObject,
                          const std::shared_ptr<metadata::URDF::Link>& link,
                          gfx::DrawableGroup* drawables,
                          const std::string& lightSetup,
                          int semanticId);

  /**
   * @brief Override of @ref PhysicsManager::removeObject to also remove any
   * active Bullet physics constraints for the object.
   */
  void removeObject(int objectId,
                    bool deleteObjectNode = true,
                    bool deleteVisualNode = true) override;

  /**
   * @brief Override of @ref PhysicsManager::removeArticulatedObject to also
   * remove any active Bullet physics constraints for the object.
   */
  void removeArticulatedObject(int objectId) override;

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
  void setStageFrictionCoefficient(double frictionCoefficient) override;

  /** @brief Set the coefficient of restitution for the stage collision
   * geometry. See @ref staticStageObject_. See @ref
   * BulletRigidObject::setRestitutionCoefficient.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  void setStageRestitutionCoefficient(double restitutionCoefficient) override;

  //============ Bullet-specific Object Getter functions =============
  /** @brief Get the current friction coefficient of the stage collision
   * geometry. See @ref staticStageObject_ and @ref
   * BulletRigidObject::getFrictionCoefficient.
   * @return The scalar friction coefficient of the stage geometry.
   */
  double getStageFrictionCoefficient() const override;

  /** @brief Get the current coefficient of restitution for the stage
   * collision geometry. This determines the ratio of initial to final relative
   * velocity between the stage and colliding object. See @ref
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
  Magnum::Range3D getCollisionShapeAabb(int physObjectID) const;

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
  int getNumActiveContactPoints() override {
    return BulletCollisionHelper::get().getNumActiveContactPoints(
        bWorld_.get());
  }

  /**
   * @brief Query the number of overlapping pairs that were active during the
   * collision detection check.
   *
   * When object bounding boxes overlap and either object is active, additional
   * "narrowphase" collision-detection must be run. This count is a proxy for
   * complexity/cost of collision-handling in the current scene. See also
   * getNumActiveContactPoints.
   *
   * @return the number of active overlapping pairs.
   */
  int getNumActiveOverlappingPairs() override {
    return BulletCollisionHelper::get().getNumActiveOverlappingPairs(
        bWorld_.get());
  }

  /**
   * @brief Get a summary of collision-processing from the last physics step.
   */
  std::string getStepCollisionSummary() override {
    return BulletCollisionHelper::get().getStepCollisionSummary(bWorld_.get());
  }

  /**
   * @brief Perform discrete collision detection for the scene.
   */
  void performDiscreteCollisionDetection() override {
    bWorld_->getCollisionWorld()->performDiscreteCollisionDetection();
    recentNumSubStepsTaken_ = -1;  // TODO: handle this more gracefully
  }

  //============ Rigid Constraints =============

  /**
   * @brief Create a rigid constraint between two objects or an object and the
   * world.
   *
   * @param settings The datastructure defining the constraint parameters.
   *
   * @return The id of the newly created constraint or ID_UNDEFINED if failed.
   */
  int createRigidConstraint(const RigidConstraintSettings& settings) override;

  /**
   * @brief Update the settings of a rigid constraint.
   *
   * @param constraintId The id of the constraint to update.
   * @param settings The new settings of the constraint.
   */
  void updateRigidConstraint(int constraintId,
                             const RigidConstraintSettings& settings) override;

  /**
   * @brief Remove a rigid constraint by id.
   *
   * @param constraintId The id of the constraint to remove.
   */
  void removeRigidConstraint(int constraintId) override;

  /**
   * @brief utilize PhysicsManager's enable shared
   */
  BulletPhysicsManager::ptr shared_from_this() {
    return esp::shared_from(this);
  }

 protected:
  //! counter for constraint id generation
  int nextConstraintId_ = 0;
  //! caches for various types of Bullet rigid constraint objects.
  std::unordered_map<int, std::unique_ptr<btMultiBodyPoint2Point>>
      articulatedP2PConstraints_;
  std::unordered_map<int, std::unique_ptr<btMultiBodyFixedConstraint>>
      articulatedFixedConstraints_;
  std::unordered_map<int, std::unique_ptr<btPoint2PointConstraint>>
      rigidP2PConstraints_;
  std::unordered_map<int, std::unique_ptr<btFixedConstraint>>
      rigidFixedConstraints_;
  //! when constraining objects to the global frame, a dummy object with 0 mass
  //! is required.
  std::unique_ptr<btRigidBody> globalFrameObject = nullptr;

  //! Maps object ids to a list of active constraints referencing the object for
  //! use in constraint clean-up and object sleep state management.
  std::unordered_map<int, std::vector<int>> objectConstraints_;

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

  mutable std::unique_ptr<Magnum::BulletIntegration::DebugDraw> debugDrawer_;

  //! keep a map of collision objects to object ids for quick lookups from
  //! Bullet collision checking.
  std::shared_ptr<std::map<const btCollisionObject*, int>>
      collisionObjToObjIds_;

  //! necessary to acquire forces from impulses
  double recentTimeStep_ = fixedTimeStep_;
  //! for recent call to stepPhysics
  int recentNumSubStepsTaken_ = -1;

 private:
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

  /**
   * @brief Helper function for removing all rigid constraints referencing an
   * object.
   *
   * @param objectId The unique id for the rigid or articulated object.
   */
  void removeObjectRigidConstraints(int objectId) {
    auto objConstraintIter = objectConstraints_.find(objectId);
    if (objConstraintIter != objectConstraints_.end()) {
      for (auto c_id : objConstraintIter->second) {
        removeRigidConstraint(c_id);
      }
      objectConstraints_.erase(objectId);
    }
  };

  /**
   * @brief Helper function for instantiating a skinned model associated to the
   * articulated object. The model bones are driven by parenting them to their
   * associated articulated object links by string-matching.
   *
   * @param ao Articulated object upon which the skinned model instance is
   * attached.
   * @param renderAssetPath Path of the skinned model.
   * @param parentNode Scene node that will be the parent of the skinned model
   * instance.
   * @param drawables Drawable group associated with the skinned model instance.
   * @param lightSetupKey Light setup associated with the skinned model
   * instance.
   */
  void instantiateSkinnedModel(
      const std::shared_ptr<BulletArticulatedObject>& ao,
      const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
          artObjAttributes,
      const std::string& renderAssetPath,
      scene::SceneNode* parentNode,
      DrawableGroup* drawables,
      const std::string& lightSetupKey);

 public:
  ESP_SMART_POINTERS(BulletPhysicsManager)

};  // end class BulletPhysicsManager
}  // end namespace physics
}  // end namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETPHYSICSMANAGER_H_
