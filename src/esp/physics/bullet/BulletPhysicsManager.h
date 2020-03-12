// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::BulletPhysicsManager
 */

/* Bullet Physics Integration */
#include <Magnum/BulletIntegration/DebugDraw.h>
#include <Magnum/BulletIntegration/Integration.h>
#include <Magnum/BulletIntegration/MotionState.h>
#include <btBulletDynamicsCommon.h>

#include "BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"

#include "BulletRigidObject.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/physics/bullet/BulletRigidObject.h"

namespace esp {
namespace physics {

/**
@brief Dynamic scene and object manager interfacing with Bullet physics engine:
https://github.com/bulletphysics/bullet3.

See @ref btDiscreteDynamicsWorld.

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
  explicit BulletPhysicsManager(assets::ResourceManager* _resourceManager)
      : PhysicsManager(_resourceManager){};

  /** @brief Destructor which destructs necessary Bullet physics structures.*/
  virtual ~BulletPhysicsManager();

  //============ Initialization =============

  /**
   * @brief Initialization: load physical properties and setup the world.
   * @param node The scene graph node which will act as the parent of all
   * physical scene and object nodes.
   * @param physicsManagerAttributes A structure containing values for physical
   * parameters necessary to initialize the physical scene and simulator.
   */
  bool initPhysics(
      scene::SceneNode* node,
      const assets::PhysicsManagerAttributes& physicsManagerAttributes);

  //============ Object/Scene Instantiation =============

  /**
   * @brief Initialize static scene collision geometry from loaded mesh data.
   * Only one 'scene' may be initialized per simulated world, but this scene may
   * contain several components (e.g. GLB heirarchy). Checks that the collision
   * mesh can be used by Bullet. See @ref BulletRigidObject::initializeScene.
   * Bullet mesh conversion adapted from:
   * https://github.com/mosra/magnum-integration/issues/20
   * @param physicsSceneAttributes a structure defining physical properties of
   * the scene.
   * @param meshGroup collision meshs for the scene.
   * @return true if successful and false otherwise
   */
  bool addScene(const assets::PhysicsSceneAttributes& physicsSceneAttributes,
                const std::vector<assets::CollisionMeshData>& meshGroup);

  virtual int addObject(
      const int objectLibIndex,
      DrawableGroup* drawables,
      const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{
          assets::ResourceManager::DEFAULT_LIGHTING_KEY}) override;

  //============ Simulator functions =============

  /** @brief Step the physical world forward in time. Time may only advance in
   * increments of @ref fixedTimeStep_. See @ref
   * btDiscreteDynamicsWorld::stepSimulation.
   * @param dt The desired amount of time to advance the physical world.
   */
  void stepPhysics(double dt);

  /** @brief Set the gravity of the physical world.
   * @param gravity The desired gravity force of the physical world.
   */
  void setGravity(const Magnum::Vector3& gravity);

  /** @brief Get the current gravity in the physical world.
   * @return The current gravity vector in the physical world.
   */
  Magnum::Vector3 getGravity() const;

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
  void setMargin(const int physObjectID, const double margin);

  /** @brief Set the friction coefficient of the scene collision geometry. See
   * @ref staticSceneObject_. See @ref
   * BulletRigidObject::setFrictionCoefficient.
   * @param frictionCoefficient The scalar friction coefficient of the scene
   * geometry.
   */
  void setSceneFrictionCoefficient(const double frictionCoefficient);

  /** @brief Set the coefficient of restitution for the scene collision
   * geometry. See @ref staticSceneObject_. See @ref
   * BulletRigidObject::setRestitutionCoefficient.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  void setSceneRestitutionCoefficient(const double restitutionCoefficient);

  //============ Bullet-specific Object Getter functions =============

  /** @brief Get the scalar collision margin of an object.
   * See @ref BulletRigidObject::getMargin.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar collision margin of the object or @ref
   * esp::PHYSICS_ATTR_UNDEFINED if failed..
   */
  double getMargin(const int physObjectID) const;

  /** @brief Get the current friction coefficient of the scene collision
   * geometry. See @ref staticSceneObject_ and @ref
   * BulletRigidObject::getFrictionCoefficient.
   * @return The scalar friction coefficient of the scene geometry.
   */
  double getSceneFrictionCoefficient() const;

  /** @brief Get the current coefficient of restitution for the scene collision
   * geometry. This determines the ratio of initial to final relative velocity
   * between the scene and collidiing object. See @ref staticSceneObject_ and
   * BulletRigidObject::getRestitutionCoefficient.
   * @return The scalar coefficient of restitution for the scene geometry.
   */
  double getSceneRestitutionCoefficient() const;

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
   * the static scene in its local space. See @ref btCompoundShape::getAabb.
   * @return The scene collision Aabb.
   */
  const Magnum::Range3D getSceneCollisionShapeAabb() const;

  /** @brief Render the debugging visualizations provided by @ref
   * Magnum::BulletIntegration::DebugDraw. This draws wireframes for all
   * collision objects.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  virtual void debugDraw(const Magnum::Matrix4& projTrans) const override;

  /**
   * @brief Check whether an object is in contact with any other objects or the
   * scene.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest(const int physObjectID);

 protected:
  btDbvtBroadphase bBroadphase_;
  btDefaultCollisionConfiguration bCollisionConfig_;

  btMultiBodyConstraintSolver bSolver_;
  btCollisionDispatcher bDispatcher_{&bCollisionConfig_};

  /** @brief A pointer to the Bullet world. See @ref btMultiBodyDynamicsWorld.*/
  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  mutable Magnum::BulletIntegration::DebugDraw debugDrawer_;

 private:
  /** @brief Check if a particular mesh can be used as a collision mesh for
   * Bullet.
   * @param meshData The mesh to validate. Only a triangle mesh is valid. Checks
   * that the only #ref Magnum::MeshPrimitive are @ref
   * Magnum::MeshPrimitive::Triangles.
   * @return true if valid, false otherwise.
   */
  bool isMeshPrimitiveValid(const assets::CollisionMeshData& meshData);

  /** @brief Create and initialize an @ref RigidObject and assign it an ID. See
   * @ref allocateObjectID and @ref BulletRigidObject::initializeObject.
   * @param meshGroup The object's mesh.
   * @param physicsObjectAttributes The physical object's template defining its
   * physical parameters.
   */
  int makeRigidObject(
      const std::vector<assets::CollisionMeshData>& meshGroup,
      assets::PhysicsObjectAttributes physicsObjectAttributes) override;

  ESP_SMART_POINTERS(BulletPhysicsManager)

};  // end class BulletPhysicsManager
}  // end namespace physics
}  // end namespace esp
