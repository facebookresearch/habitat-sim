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
    const assets::PhysicsSceneAttributes& physicsSceneAttributes,
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
    const assets::PhysicsObjectAttributes& physicsObjectAttributes,
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
  // NOTE: no active objects without a physics engine... (kinematics don't
  // count)
  return false;
}

bool RigidObject::setMotionType(MotionType mt) {
  if (rigidObjectType_ == OBJECT) {
    if (mt != DYNAMIC) {
      objectMotionType_ = mt;
      return true;
    } else {
      return false;  // can't set DYNAMIC without a dynamics engine.
    }
  } else if (rigidObjectType_ == SCENE) {
    return mt == STATIC;  // only option and default option
  }
  return false;
}

void RigidObject::applyForce(const Magnum::Vector3& force,
                             const Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::applyImpulse(const Magnum::Vector3& impulse,
                               const Magnum::Vector3& relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

//! Torque interaction
void RigidObject::applyTorque(const Magnum::Vector3& torque) {
  return;
}
// Impulse Torque interaction
void RigidObject::applyImpulseTorque(const Magnum::Vector3& impulse) {
  return;
}

void RigidObject::syncPose() {
  return;
}

scene::SceneNode& RigidObject::setTransformation(
    const Magnum::Matrix4& transformation) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::setTransformation(transformation);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::setTranslation(const Magnum::Vector3& vector) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::setTranslation(vector);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::setRotation(
    const Magnum::Quaternion& quaternion) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::setRotation(quaternion);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::resetTransformation() {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::resetTransformation();
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::translate(const Magnum::Vector3& vector) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::translate(vector);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::translateLocal(const Magnum::Vector3& vector) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::translateLocal(vector);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotate(const Magnum::Rad angleInRad,
                                      const Magnum::Vector3& normalizedAxis) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotate(angleInRad, normalizedAxis);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateLocal(
    const Magnum::Rad angleInRad,
    const Magnum::Vector3& normalizedAxis) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateLocal(angleInRad, normalizedAxis);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateX(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateX(angleInRad);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateXLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateXLocal(angleInRad);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateY(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateY(angleInRad);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateYLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateYLocal(angleInRad);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateZ(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateZ(angleInRad);
    syncPose();
  }
  return *this;
}

scene::SceneNode& RigidObject::rotateZLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != STATIC) {
    scene::SceneNode::rotateZLocal(angleInRad);
    syncPose();
  }
  return *this;
}

const Magnum::Vector3 RigidObject::getCOM() {
  const Magnum::Vector3 com = Magnum::Vector3();
  return com;
}

const Magnum::Vector3 RigidObject::getInertiaVector() {
  const Magnum::Vector3 inertia = Magnum::Vector3();
  return inertia;
}

const Magnum::Matrix3 RigidObject::getInertiaMatrix() {
  const Magnum::Matrix3 inertia = Magnum::Matrix3();
  return inertia;
}

}  // namespace physics
}  // namespace esp
