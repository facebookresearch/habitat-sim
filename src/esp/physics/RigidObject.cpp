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
    const assets::PhysicsSceneAttributes&,
    const std::vector<assets::CollisionMeshData>&) {
  if (rigidObjectType_ != RigidObjectType::NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = RigidObjectType::SCENE;
  objectMotionType_ = MotionType::STATIC;

  return true;
}

bool RigidObject::initializeObject(
    const assets::PhysicsObjectAttributes&,
    const std::vector<assets::CollisionMeshData>&) {
  // TODO (JH): Handling static/kinematic object type
  if (rigidObjectType_ != RigidObjectType::NONE) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  rigidObjectType_ = RigidObjectType::OBJECT;
  // default kineamtic unless a simulator is initialized...
  objectMotionType_ = MotionType::KINEMATIC;

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
  if (rigidObjectType_ == RigidObjectType::OBJECT) {
    if (mt != MotionType::DYNAMIC) {
      objectMotionType_ = mt;
      return true;
    } else {
      return false;  // can't set DYNAMIC without a dynamics engine.
    }
  } else if (rigidObjectType_ == RigidObjectType::SCENE) {
    return mt == MotionType::STATIC;  // only option and default option
  }
  return false;
}

void RigidObject::shiftOrigin(const Magnum::Vector3& shift) {
  // shift each child node
  for (auto& child : children()) {
    child.translate(shift);
  }
  computeCumulativeBB();
}

void RigidObject::shiftOriginToBBCenter() {
  shiftOrigin(-cumulativeBB_.center());
}

void RigidObject::applyForce(const Magnum::Vector3&, const Magnum::Vector3&) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::applyImpulse(const Magnum::Vector3&, const Magnum::Vector3&) {
  // without a physics engine we can't apply any forces...
  return;
}

//! Torque interaction
void RigidObject::applyTorque(const Magnum::Vector3&) {
  return;
}
// Impulse Torque interaction
void RigidObject::applyImpulseTorque(const Magnum::Vector3&) {
  return;
}

void RigidObject::syncPose() {
  return;
}

void RigidObject::setTransformation(const Magnum::Matrix4& transformation) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::setTransformation(transformation);
    syncPose();
  }
}

void RigidObject::setTranslation(const Magnum::Vector3& vector) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::setTranslation(vector);
    syncPose();
  }
}

void RigidObject::setRotation(const Magnum::Quaternion& quaternion) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::setRotation(quaternion);
    syncPose();
  }
}

void RigidObject::resetTransformation() {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::resetTransformation();
    syncPose();
  }
}

void RigidObject::translate(const Magnum::Vector3& vector) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::translate(vector);
    syncPose();
  }
}

void RigidObject::translateLocal(const Magnum::Vector3& vector) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::translateLocal(vector);
    syncPose();
  }
}

void RigidObject::rotate(const Magnum::Rad angleInRad,
                         const Magnum::Vector3& normalizedAxis) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotate(angleInRad, normalizedAxis);
    syncPose();
  }
}

void RigidObject::rotateLocal(const Magnum::Rad angleInRad,
                              const Magnum::Vector3& normalizedAxis) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateLocal(angleInRad, normalizedAxis);
    syncPose();
  }
}

void RigidObject::rotateX(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateX(angleInRad);
    syncPose();
  }
}

void RigidObject::rotateXLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateXLocal(angleInRad);
    syncPose();
  }
}

void RigidObject::rotateY(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateY(angleInRad);
    syncPose();
  }
}

void RigidObject::rotateYLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateYLocal(angleInRad);
    syncPose();
  }
}

void RigidObject::rotateZ(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateZ(angleInRad);
    syncPose();
  }
}

void RigidObject::rotateZLocal(const Magnum::Rad angleInRad) {
  if (objectMotionType_ != MotionType::STATIC) {
    scene::SceneNode::rotateZLocal(angleInRad);
    syncPose();
  }
}

Magnum::Vector3 RigidObject::getCOM() {
  const Magnum::Vector3 com = Magnum::Vector3();
  return com;
}

Magnum::Vector3 RigidObject::getInertiaVector() {
  const Magnum::Vector3 inertia = Magnum::Vector3();
  return inertia;
}

Magnum::Matrix3 RigidObject::getInertiaMatrix() {
  const Magnum::Matrix3 inertia = Magnum::Matrix3();
  return inertia;
}

}  // namespace physics
}  // namespace esp
