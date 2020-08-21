// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETRIGIDSCENERY_H_
#define ESP_PHYSICS_BULLET_BULLETRIGIDSCENERY_H_

#include "esp/physics/RigidScenery.h"
#include "esp/physics/bullet/BulletBase.h"

/** @file
 * @brief Class @ref esp::physics::bullet::BulletRigidScenery
 */
namespace esp {
namespace physics {

/**
 * @brief An individual rigid scenery instance implementing an interface with
 * Bullet physics to enable dynamics. See @ref btCollisionObject
 */

class BulletRigidScenery : public BulletBase, public RigidScenery {
 public:
  BulletRigidScenery(scene::SceneNode* rigidBodyNode,
                     std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                     std::shared_ptr<std::map<const btCollisionObject*, int>>
                         collisionObjToObjIds);

  /**
   * @brief Destructor cleans up simulation structures for the scenery object.
   */
  virtual ~BulletRigidScenery();

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidScene
   * geometry.  This holds bullet-specific functionality for scenerys.
   * @param resMgr Reference to resource manager, to access relevant components
   * pertaining to the scenery object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific(
      const assets::ResourceManager& resMgr) override;

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

 public:
  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @return The Aabb.
   */
  virtual const Magnum::Range3D getCollisionShapeAabb() const override;

  /** @brief Get the scalar friction coefficient of the scenery object. Only
   * used for dervied dynamic implementations of @ref RigidScenery.
   * @return The scalar friction coefficient of the scenery object.
   */
  virtual double getFrictionCoefficient() const override;

  /** @brief Get the scalar coefficient of restitution  of the scenery object.
   * Only used for dervied dynamic implementations of @ref RigidScenery.
   * @return The scalar coefficient of restitution  of the scenery object.
   */
  virtual double getRestitutionCoefficient() const override;

  /** @brief Set the scalar friction coefficient of the scenery object.
   * See @ref btCollisionObject::setFriction.
   * @param frictionCoefficient The new scalar friction coefficient of the
   * scenery object.
   */
  void setFrictionCoefficient(const double frictionCoefficient) override;

  /** @brief Set the scalar coefficient of restitution of the scenery object.
   * See @ref btCollisionObject::setRestitution.
   * @param restitutionCoefficient The new scalar coefficient of restitution of
   * the scenery object.
   */
  void setRestitutionCoefficient(const double restitutionCoefficient) override;

 private:
  // === Physical scenery ===

  //! Scenery data: Bullet triangular mesh vertices
  std::vector<std::unique_ptr<btTriangleIndexVertexArray>> bSceneryArrays_;

  //! Scenery data: Bullet triangular mesh shape
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneryShapes_;

 public:
  ESP_SMART_POINTERS(BulletRigidScenery)

};  // class BulletRigidScenery

}  // namespace physics
}  // namespace esp
#endif  // ESP_PHYSICS_BULLET_BULLETRIGIDSCENERY_H_
