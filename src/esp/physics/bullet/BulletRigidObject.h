// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <btBulletDynamicsCommon.h>
#include "esp/assets/Asset.h"
#include "esp/core/esp.h"

#include "esp/physics/RigidObject.h"

namespace esp {
namespace physics {

class BulletRigidObject : public RigidObject {
 public:
  BulletRigidObject(scene::SceneNode* parent);

  ~BulletRigidObject();

  bool initializeScene(assets::PhysicsSceneAttributes& physicsSceneAttributes,
                       std::vector<assets::CollisionMeshData> meshGroup,
                       std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  bool initializeObject(
      assets::PhysicsObjectAttributes& physicsObjectAttributes,
      std::vector<assets::CollisionMeshData> meshGroup,
      std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  //! Check whether object is being actively simulated, or sleeping
  bool isActive();
  void setActive();

  //! Force interaction
  void applyForce(Magnum::Vector3 force, Magnum::Vector3 relPos);
  //! Impulse interaction
  void applyImpulse(Magnum::Vector3 impulse, Magnum::Vector3 relPos);

  bool removeObject();

  //============ Getter/setter function =============
  const double getMass();
  const Magnum::Vector3d getCOM();
  const Magnum::Vector3 getInertia();
  const double getScale();
  const double getFrictionCoefficient();
  const double getRestitutionCoeffcient();
  const double getLinearDamping();
  const double getAngularDamping();
  //! Return margin for object, -1.0 for scene
  const double getMargin();

  void setMass(const double mass);
  void setCOM(const Magnum::Vector3d COM);
  void setInertia(const Magnum::Vector3 inertia);
  void setScale(const double scale);
  void setFrictionCoefficient(const double frictionCoefficient);
  void setRestitutionCoeffcient(const double restitutionCoeffcient);
  void setLinearDamping(const double linearDamping);
  void setAngularDamping(const double angularDamping);
  //! Set Margin only works for objects
  void setMargin(const double margin);

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
