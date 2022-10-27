// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETBASE_H_
#define ESP_PHYSICS_BULLET_BULLETBASE_H_

#include <Magnum/BulletIntegration/MotionState.h>
#include <btBulletDynamicsCommon.h>

#include <utility>

#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/core/Esp.h"
#include "esp/scene/SceneNode.h"

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
  bool bCollision{false};

  /**
   * @brief Constructor.
   */
  SimulationContactResultCallback() = default;

  /**
   * @brief Called when a contact is detected.
   *
   * Sets a collision flag on every detected collision. Can be updated to do
   * more.
   * @param cp Contains detailed information about the contact point being
   * added.
   */
  btScalar addSingleResult(
      CORRADE_UNUSED btManifoldPoint& cp,
      CORRADE_UNUSED const btCollisionObjectWrapper* colObj0Wrap,
      CORRADE_UNUSED int partId0,
      CORRADE_UNUSED int index0,
      CORRADE_UNUSED const btCollisionObjectWrapper* colObj1Wrap,
      CORRADE_UNUSED int partId1,
      CORRADE_UNUSED int index1) override {
    bCollision = true;
    return 0;  // not used
  }
};

/**
 * @brief This class is intended to implement bullet-specific
 */

class BulletBase {
 public:
  BulletBase(std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
             std::shared_ptr<std::map<const btCollisionObject*, int>>
                 collisionObjToObjIds)
      : bWorld_(std::move(bWorld)),
        collisionObjToObjIds_(std::move(collisionObjToObjIds)) {}

  /**
   * @brief Destructor cleans up simulation structures for the object.
   */
  virtual ~BulletBase() { bWorld_.reset(); }

  /** @brief Get the scalar collision margin of an object. Retun 0.0 for a @ref
   * RigidObjectType::SCENE. See @ref btCompoundShape::getMargin.
   * @return The scalar collision margin of the object.
   */
  virtual double getMargin() const { return 0.0; }

  /** @brief Set the scalar collision margin of an object. Does not affect @ref
   * RigidObjectType::SCENE. See @ref btCompoundShape::setMargin.
   * @param margin The new scalar collision margin of the object.
   */
  virtual void setMargin(CORRADE_UNUSED const double margin) {}

  /**
   * @brief Query the Aabb from bullet physics for the root compound shape of
   * the rigid body in its local space. See @ref btCompoundShape::getAabb.
   * @return The Aabb.
   */
  virtual Magnum::Range3D getCollisionShapeAabb() const = 0;

  /**
   * @brief Recursively construct a @ref btConvexHullShape for collision by
   * joining loaded mesh assets.
   * @param transformFromParentToWorld The cumulative parent-to-world
   * transformation matrix constructed by composition down the @ref
   * MeshTransformNode tree to the current node.
   * @param meshGroup Access structure for collision mesh data.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param bConvexShape The convex we are building. Should be a new, empty
   * shape when passed into entry point.
   */
  static void constructJoinedConvexShapeFromMeshes(
      const Magnum::Matrix4& transformFromParentToWorld,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      const assets::MeshTransformNode& node,
      btConvexHullShape* bConvexShape);

  /**
   * @brief Recursively construct a @ref btCompoundShape for collision from
   * loaded mesh assets. A @ref btConvexHullShape is constructed for each
   * sub-component, transformed to object-local space and added to the compound
   * in a flat manner for efficiency.
   * @param transformFromParentToWorld The cumulative parent-to-world
   * transformation matrix constructed by composition down the @ref
   * MeshTransformNode tree to the current node.
   * @param meshGroup Access structure for collision mesh data.
   * @param node The current @ref MeshTransformNode in the recursion.
   * @param bObjectShape The compound shape parent for all generated convexes
   * @param bObjectConvexShapes Datastructure to cache generated convex shapes
   */
  static void constructConvexShapesFromMeshes(
      const Magnum::Matrix4& transformFromParentToWorld,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      const assets::MeshTransformNode& node,
      btCompoundShape* bObjectShape,
      std::vector<std::unique_ptr<btConvexHullShape>>& bObjectConvexShapes);

 protected:
  /** @brief A pointer to the Bullet world to which this object belongs. See
   * @ref btMultiBodyDynamicsWorld.*/
  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  /** @brief Static data: All components of a @ref RigidObjectType::SCENE are
   * stored here. Also, all objects set to STATIC are stored here.
   */
  std::vector<std::unique_ptr<btRigidBody>> bStaticCollisionObjects_;

  //! Object data: Composite convex collision shape
  std::vector<std::unique_ptr<btConvexHullShape>> bObjectConvexShapes_;

  //! list of @ref btCollisionShape for storing arbitrary collision shapes
  //! referenced within the @ref bObjectShape_.
  std::vector<std::unique_ptr<btCollisionShape>> bGenericShapes_;

  //! keep a map of collision objects to object ids for quick lookups from
  //! Bullet collision checking.
  std::shared_ptr<std::map<const btCollisionObject*, int>>
      collisionObjToObjIds_;

 public:
  ESP_SMART_POINTERS(BulletBase)
};  // class BulletBase

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETBASE_H_
