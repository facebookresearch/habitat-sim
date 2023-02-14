// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETRIGIDSTAGE_H_
#define ESP_PHYSICS_BULLET_BULLETRIGIDSTAGE_H_

#include "esp/physics/RigidStage.h"
#include "esp/physics/bullet/BulletBase.h"

/** @file
 * @brief Class @ref esp::physics::BulletRigidStage
 */
namespace esp {
namespace physics {

/**
 * @brief An individual rigid stage instance implementing an interface with
 * Bullet physics to enable dynamics. See @ref btCollisionObject
 */

class BulletRigidStage : public BulletBase, public RigidStage {
 public:
  BulletRigidStage(scene::SceneNode* rigidBodyNode,
                   const assets::ResourceManager& resMgr,
                   std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                   std::shared_ptr<std::map<const btCollisionObject*, int>>
                       collisionObjToObjIds);

  /**
   * @brief Destructor cleans up simulation structures for the stage object.
   */
  ~BulletRigidStage() override;

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidScene
   * geometry.  This holds bullet-specific functionality for stages.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override;

  /**
   * @brief Recursively construct the static collision mesh objects from
   * imported assets.
   * @param transformFromParentToWorld The cumulative parent-to-world
   * transformation matrix constructed by composition down the @ref
   * MeshTransformNode tree to the current node.
   * @param meshGroup Access structure for collision mesh data.
   * @param node The current @ref MeshTransformNode in the recursion.
   */
  void constructBulletSceneFromMeshes(
      const Magnum::Matrix4& transformFromParentToWorld,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      const assets::MeshTransformNode& node);

  /**
   * @brief Adds static stage collision objects to the simulation world after
   * contracting them if necessary.
   */
  void constructAndAddCollisionObjects();

  /**
   * @brief used with BulletCollisionHelper
   */
  std::string getCollisionDebugName(int subpartId);

  /**
   * @brief Set the stage to collidable or not by adding/removing the static
   * collision shapes from the simulation world.
   */
  void setCollidable(bool collidable) override;

 public:
  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @return The Aabb.
   */
  Magnum::Range3D getCollisionShapeAabb() const override;

  /** @brief Get the scalar friction coefficient of the stage object. Only
   * used for dervied dynamic implementations of @ref RigidStage.
   * @return The scalar friction coefficient of the stage object.
   */
  double getFrictionCoefficient() const override;

  /** @brief Get the scalar coefficient of restitution  of the stage object.
   * Only used for dervied dynamic implementations of @ref RigidStage.
   * @return The scalar coefficient of restitution  of the stage object.
   */
  double getRestitutionCoefficient() const override;

  /** @brief Set the scalar friction coefficient of the stage object.
   * See @ref btCollisionObject::setFriction.
   * @param frictionCoefficient The new scalar friction coefficient of the
   * stage object.
   */
  void setFrictionCoefficient(double frictionCoefficient) override;

  /** @brief Set the scalar coefficient of restitution of the stage object.
   * See @ref btCollisionObject::setRestitution.
   * @param restitutionCoefficient The new scalar coefficient of restitution of
   * the stage object.
   */
  void setRestitutionCoefficient(double restitutionCoefficient) override;

 private:
  // === Physical stage ===

  //! Stage data: Bullet triangular mesh vertices
  std::vector<std::unique_ptr<btTriangleIndexVertexArray>> bStageArrays_;

  //! Stage data: Bullet triangular mesh shape
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bStageShapes_;

 public:
  ESP_SMART_POINTERS(BulletRigidStage)

};  // class BulletRigidStage

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_BULLET_BULLETRIGIDSTAGE_H_
