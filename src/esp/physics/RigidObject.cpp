// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObject.h"
#include <Magnum/Math/Range.h>

namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* parent)
    : scene::SceneNode{*parent} {}

bool RigidObject::initializeScene(
    assets::PhysicsSceneAttributes& physicsSceneAttributes,
    std::vector<assets::CollisionMeshData>& meshGroup) {
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isObject_) {
    return false;
  }
  isScene_ = true;
  objectMotionType_ = STATIC;

  initialized_ = true;
  return true;
}

bool RigidObject::initializeObject(
    assets::PhysicsObjectAttributes& physicsObjectAttributes,
    std::vector<assets::CollisionMeshData>& meshGroup) {
  // TODO (JH): Handling static/kinematic object type
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isScene_) {
    return false;
  }
  isObject_ = true;
  objectMotionType_ =
      KINEMATIC;  // default kineamtic unless a simulator is initialized...

  initialized_ = true;
  return true;
}

bool RigidObject::removeObject() {
  return true;
}

bool RigidObject::isActive() {
  // Alex NOTE: no active objects without a physics engine... (kinematics don't
  // count)
  return false;
}

RigidObject::~RigidObject() {
  if (initialized_) {
    LOG(INFO) << "Deleting object ";
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

void RigidObject::applyForce(Magnum::Vector3& force, Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::applyImpulse(Magnum::Vector3& impulse,
                               Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

//! Synchronize Physics transformations
//! Needed after changing the pose from Magnum side
//! Not needed if no physics engine to sync
void RigidObject::syncPose() {
  return;
}

scene::SceneNode& RigidObject::setTransformation(
    const Magnum::Math::Matrix4<float>& transformation) {
  scene::SceneNode::setTransformation(transformation);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setTranslation(
    const Magnum::Math::Vector3<float>& vector) {
  scene::SceneNode::setTranslation(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setRotation(
    const Magnum::Math::Quaternion<float>& quaternion) {
  scene::SceneNode::setRotation(quaternion);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::resetTransformation() {
  scene::SceneNode::resetTransformation();
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translate(
    const Magnum::Math::Vector3<float>& vector) {
  scene::SceneNode::translate(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translateLocal(
    const Magnum::Math::Vector3<float>& vector) {
  scene::SceneNode::translateLocal(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotate(
    const Magnum::Math::Rad<float> angleInRad,
    const Magnum::Math::Vector3<float>& normalizedAxis) {
  scene::SceneNode::rotate(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateLocal(
    const Magnum::Math::Rad<float> angleInRad,
    const Magnum::Math::Vector3<float>& normalizedAxis) {
  scene::SceneNode::rotateLocal(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateX(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateX(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateXLocal(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateXLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateY(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateY(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateYLocal(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateYLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZ(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateZ(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZLocal(
    const Magnum::Math::Rad<float> angleInRad) {
  scene::SceneNode::rotateZLocal(angleInRad);
  syncPose();
  return *this;
}

const Magnum::Vector3& RigidObject::getCOM() {
  const Magnum::Vector3 com = Magnum::Vector3();
  return com;
}

const Magnum::Vector3& RigidObject::getInertia() {
  const Magnum::Vector3 inertia = Magnum::Vector3();
  return inertia;
}

}  // namespace physics
}  // namespace esp
