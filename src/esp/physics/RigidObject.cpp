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
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  if (rigidObjectType_ != NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = SCENE;
  objectMotionType_ = STATIC;

  return true;
}

bool RigidObject::initializeObject(
    assets::PhysicsObjectAttributes& physicsObjectAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // TODO (JH): Handling static/kinematic object type
  if (rigidObjectType_ != NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = OBJECT;
  // default kineamtic unless a simulator is initialized...
  objectMotionType_ = KINEMATIC;

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

void RigidObject::applyForce(Magnum::Vector3& force, Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::applyImpulse(Magnum::Vector3& impulse,
                               Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::syncPose() {
  return;
}

scene::SceneNode& RigidObject::setTransformation(
    const Magnum::Matrix4& transformation) {
  scene::SceneNode::setTransformation(transformation);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setTranslation(const Magnum::Vector3& vector) {
  scene::SceneNode::setTranslation(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setRotation(
    const Magnum::Quaternion& quaternion) {
  scene::SceneNode::setRotation(quaternion);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::resetTransformation() {
  scene::SceneNode::resetTransformation();
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translate(const Magnum::Vector3& vector) {
  scene::SceneNode::translate(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translateLocal(const Magnum::Vector3& vector) {
  scene::SceneNode::translateLocal(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotate(const Magnum::Rad angleInRad,
                                      const Magnum::Vector3& normalizedAxis) {
  scene::SceneNode::rotate(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateLocal(
    const Magnum::Rad angleInRad,
    const Magnum::Vector3& normalizedAxis) {
  scene::SceneNode::rotateLocal(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateX(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateX(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateXLocal(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateXLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateY(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateY(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateYLocal(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateYLocal(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZ(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateZ(angleInRad);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateZLocal(const Magnum::Rad angleInRad) {
  scene::SceneNode::rotateZLocal(angleInRad);
  syncPose();
  return *this;
}

const Magnum::Vector3 RigidObject::getCOM() {
  const Magnum::Vector3 com = Magnum::Vector3();
  return com;
}

const Magnum::Vector3 RigidObject::getInertia() {
  const Magnum::Vector3 inertia = Magnum::Vector3();
  return inertia;
}

}  // namespace physics
}  // namespace esp
