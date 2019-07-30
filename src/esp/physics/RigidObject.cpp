// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObject.h"

namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* parent)
    : scene::SceneNode{*parent} {}

bool RigidObject::initializeScene(
    std::vector<assets::CollisionMeshData> meshGroup) {
  if (initialized_) {
    LOG(ERROR) << "Cannot initialized a RigidObject more than once";
    return false;
  }

  //! Turn on scene flag
  if (isObject_) {
    return false;
  }
  isScene_ = true;
  objectMotionType = STATIC;

  initialized_ = true;
  return true;
}

bool RigidObject::initializeObject(
    assets::PhysicsObjectMetaData& metaData,
    std::vector<assets::CollisionMeshData> meshGroup) {
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
  objectMotionType =
      KINEMATIC;  // default kineamtic unless a simulator is initialized...

  initialized_ = true;
  return true;
}

// Helper function to find object center
void RigidObject::getDimensions(assets::CollisionMeshData& meshData,
                                float* x,
                                float* y,
                                float* z) {
  float minX = 999999.9f;
  float maxX = -999999.9f;
  float minY = 999999.9f;
  float maxY = -999999.9f;
  float minZ = 999999.9f;
  float maxZ = -999999.9f;
  for (uint vi = 0; vi < meshData.positions.size(); vi++) {
    Magnum::Vector3 pos = meshData.positions[vi];
    if (pos.x() < minX) {
      minX = pos.x();
    }
    if (pos.x() > maxX) {
      maxX = pos.x();
    }
    if (pos.y() < minY) {
      minY = pos.y();
    }
    if (pos.y() > maxY) {
      maxY = pos.y();
    }
    if (pos.z() < minZ) {
      minZ = pos.z();
    }
    if (pos.z() > maxZ) {
      maxZ = pos.z();
    }
  }
  *x = maxX - minX;
  *y = maxY - minY;
  *z = maxZ - minZ;
  LOG(INFO) << "Dimensions minX " << minX << " maxX " << maxX << " minY "
            << minY << " maxY " << maxY << " minZ " << minZ << " maxZ " << maxZ;
}

bool RigidObject::isActive() {
  // Alex NOTE: no active objects without a physics engine... (kinematics don't
  // count)
  return false;
}

void RigidObject::debugForce(
    Magnum::SceneGraph::DrawableGroup3D& debugDrawables) {
  //! DEBUG draw
  debugRender_ = new Magnum::DebugTools::ForceRenderer3D(
      *this, {0.0f, 0.0f, 0.0f}, debugExternalForce_, "bulletForce",
      &debugDrawables);
  LOG(INFO) << "Force render" << debugExternalForce_.x();
}

void RigidObject::setDebugForce(Magnum::Vector3 force) {
  debugExternalForce_ = force;
}

RigidObject::~RigidObject() {
  if (initialized_) {
    LOG(INFO) << "Deleting object ";
  } else {
    LOG(INFO) << "Object not initialized";
  }
}

void RigidObject::applyForce(Magnum::Vector3 force, Magnum::Vector3 relPos) {
  // without a physics engine we can't apply any forces...
  return;
}

void RigidObject::applyImpulse(Magnum::Vector3 impulse,
                               Magnum::Vector3 relPos) {
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
    const Magnum::Math::Matrix4<float> transformation) {
  scene::SceneNode::setTransformation(transformation);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::setTranslation(
    const Magnum::Math::Vector3<float> vector) {
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
    const Magnum::Math::Vector3<float> vector) {
  scene::SceneNode::translate(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::translateLocal(
    const Magnum::Math::Vector3<float> vector) {
  scene::SceneNode::translateLocal(vector);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotate(
    const Magnum::Math::Rad<float> angleInRad,
    const Magnum::Math::Vector3<float> normalizedAxis) {
  scene::SceneNode::rotate(angleInRad, normalizedAxis);
  syncPose();
  return *this;
}

scene::SceneNode& RigidObject::rotateLocal(
    const Magnum::Math::Rad<float> angleInRad,
    const Magnum::Math::Vector3<float> normalizedAxis) {
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

}  // namespace physics
}  // namespace esp
