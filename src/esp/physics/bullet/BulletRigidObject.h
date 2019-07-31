// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <btBulletDynamicsCommon.h>
#include "esp/assets/Asset.h"
#include "esp/assets/PhysicsObjectMetaData.h"
#include "esp/core/esp.h"

#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

class BulletRigidObject : public RigidObject {
 public:
  BulletRigidObject(scene::SceneNode* parent);

  ~BulletRigidObject();

  bool initializeScene(std::vector<assets::CollisionMeshData> meshGroup,
                       std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  bool initializeObject(assets::PhysicsObjectMetaData& metaData,
                        std::vector<assets::CollisionMeshData> meshGroup,
                        std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  //! Check whether object is being actively simulated, or sleeping
  bool isActive();

  //! Force interaction
  void applyForce(Magnum::Vector3 force, Magnum::Vector3 relPos);

  // Impulse interaction
  void applyImpulse(Magnum::Vector3 impulse, Magnum::Vector3 relPos);

  bool removeObject();

 protected:
  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  void syncPose();

 private:
  //! One object can only exist in one world
  std::shared_ptr<btDiscreteDynamicsWorld> bWorld_;

  //! Physical scene
  //! Scene data: triangular mesh shape
  //! All components are stored as a vector of bCollisionBody_
  std::unique_ptr<btTriangleIndexVertexArray> bSceneArray_;
  std::vector<std::unique_ptr<btBvhTriangleMeshShape>> bSceneShapes_;
  std::vector<std::unique_ptr<btCollisionObject>> bSceneCollisionObjects_;

  // Physical object
  //! Object data: Composite convex collision shape
  //! All components are wrapped into one rigidBody_
  std::vector<std::unique_ptr<btConvexHullShape>> bObjectConvexShapes_;
  std::unique_ptr<btCompoundShape> bObjectShape_;
  std::unique_ptr<btRigidBody> bObjectRigidBody_;
  Magnum::BulletIntegration::MotionState* bObjectMotionState_;
};

}  // namespace physics
}  // namespace esp
