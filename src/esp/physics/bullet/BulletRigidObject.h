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

  bool initializeScene(
      const assets::PhysicsSceneAttributes& physicsSceneAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  bool initializeObject(
      const assets::PhysicsObjectAttributes& physicsObjectAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup,
      std::shared_ptr<btDiscreteDynamicsWorld> bWorld);

  //! Check whether object is being actively simulated, or sleeping
  bool isActive();
  void setActive();

  // attempt to set the motion type. Return false=failure, true=success.
  virtual bool setMotionType(MotionType mt);

  //! Force interaction
  void applyForce(const Magnum::Vector3& force, const Magnum::Vector3& relPos);
  //! Impulse interaction
  void applyImpulse(const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos);

  //! Torque interaction
  void applyTorque(const Magnum::Vector3& torque);
  // Impulse Torque interaction
  void applyImpulseTorque(const Magnum::Vector3& impulse);

  bool removeObject();

  //============ Getter/setter function =============
  const double getMass();
  const Magnum::Vector3 getCOM();
  const Magnum::Vector3 getInertiaVector();
  const Magnum::Matrix3 getInertiaMatrix();
  const double getScale();
  const double getFrictionCoefficient();
  const double getRestitutionCoefficient();
  const double getLinearDamping();
  const double getAngularDamping();
  //! Return margin for object, -1.0 for scene
  const double getMargin();

  void setMass(const double mass);
  void setCOM(const Magnum::Vector3& COM);
  void setInertia(const Magnum::Vector3& inertia);
  void setScale(const double scale);
  void setFrictionCoefficient(const double frictionCoefficient);
  void setRestitutionCoefficient(const double restitutionCoefficient);
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
