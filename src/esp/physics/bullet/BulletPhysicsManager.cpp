// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

//#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
//#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletRigidObject.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

bool BulletPhysicsManager::initPhysics(
    scene::SceneNode* node,
    assets::PhysicsManagerAttributes physicsManagerAttributes) {
  activePhysSimLib_ = BULLET;

  //! We can potentially use other collision checking algorithms, by
  //! uncommenting the line below
  // btGImpactCollisionAlgorithm::registerAlgorithm(&bDispatcher_);
  bWorld_ = std::make_shared<btDiscreteDynamicsWorld>(
      &bDispatcher_, &bBroadphase_, &bSolver_, &bCollisionConfig_);
  // currently GLB meshes are y-up
  bWorld_->setGravity(
      btVector3(physicsManagerAttributes.getMagnumVec3("gravity")));

  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = std::dynamic_pointer_cast<physics::RigidObject,
                                         physics::BulletRigidObject>(
      std::make_shared<physics::BulletRigidObject>(physicsNode_));

  initialized_ = true;
  return true;
}

BulletPhysicsManager::~BulletPhysicsManager() {}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addScene(
    const assets::AssetInfo& info,
    assets::PhysicsSceneAttributes& physicsSceneAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Initialize scene
  bool sceneSuccess =
      std::dynamic_pointer_cast<physics::BulletRigidObject,
                                physics::RigidObject>(sceneNode_)
          ->initializeScene(physicsSceneAttributes, meshGroup, bWorld_);

  return sceneSuccess;
}

int BulletPhysicsManager::makeRigidObject(
    const std::vector<assets::CollisionMeshData>& meshGroup,
    assets::PhysicsObjectAttributes physicsObjectAttributes) {
  //! Create new physics object (child node of sceneNode_)
  int newObjectID = allocateObjectID();
  existingObjects_.emplace(
      newObjectID,
      std::make_shared<physics::BulletRigidObject>(sceneNode_.get()));

  //! Instantiate with mesh pointer
  bool objectSuccess =
      dynamic_cast<physics::BulletRigidObject*>(
          existingObjects_.at(newObjectID).get())
          ->initializeObject(physicsObjectAttributes, meshGroup, bWorld_);
  if (!objectSuccess) {
    LOG(ERROR) << "Object load failed";
    deallocateObjectID(newObjectID);
    existingObjects_.erase(newObjectID);
    return -1;
  }
  return newObjectID;
}

//! Check if mesh primitive is compatible with physics
bool BulletPhysicsManager::isMeshPrimitiveValid(
    const assets::CollisionMeshData& meshData) {
  if (meshData.primitive == Magnum::MeshPrimitive::Triangles) {
    //! Only triangle mesh works
    return true;
  } else {
    switch (meshData.primitive) {
      case Magnum::MeshPrimitive::Lines:
        LOG(ERROR) << "Invalid primitive: Lines";
        break;
      case Magnum::MeshPrimitive::Points:
        LOG(ERROR) << "Invalid primitive: Points";
        break;
      case Magnum::MeshPrimitive::LineLoop:
        LOG(ERROR) << "Invalid primitive Line loop";
        break;
      case Magnum::MeshPrimitive::LineStrip:
        LOG(ERROR) << "Invalid primitive Line Strip";
        break;
      case Magnum::MeshPrimitive::TriangleStrip:
        LOG(ERROR) << "Invalid primitive Triangle Strip";
        break;
      case Magnum::MeshPrimitive::TriangleFan:
        LOG(ERROR) << "Invalid primitive Triangle Fan";
        break;
      default:
        LOG(ERROR) << "Invalid primitive " << int(meshData.primitive);
    }
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return false;
  }
}

void BulletPhysicsManager::setGravity(const Magnum::Vector3& gravity) {
  bWorld_->setGravity(btVector3(gravity));
  // After gravity change, need to reactive all bullet objects
  for (std::map<int, std::shared_ptr<physics::RigidObject>>::iterator it =
           existingObjects_.begin();
       it != existingObjects_.end(); ++it) {
    it->second->setActive();
  }
}

const Magnum::Vector3 BulletPhysicsManager::getGravity() {
  return Magnum::Vector3(bWorld_->getGravity());
}

void BulletPhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_) {
    return;
  }
  if (dt < 0) {
    dt = fixedTimeStep_;
  }

  // ==== Physics stepforward ======

  // NOTE: worldTime_ will always be a multiple of sceneMetaData_.timestep
  int numSubStepsTaken =
      bWorld_->stepSimulation(dt, maxSubSteps_, fixedTimeStep_);
  worldTime_ += numSubStepsTaken * fixedTimeStep_;
}

void BulletPhysicsManager::setMargin(const int physObjectID,
                                     const double margin) {
  if (existingObjects_.count(physObjectID) > 0) {
    std::dynamic_pointer_cast<BulletRigidObject>(
        existingObjects_.at(physObjectID))
        ->setMargin(margin);
  }
}

void BulletPhysicsManager::setSceneFrictionCoefficient(
    const double frictionCoefficient) {
  std::dynamic_pointer_cast<physics::BulletRigidObject, physics::RigidObject>(
      sceneNode_)
      ->setFrictionCoefficient(frictionCoefficient);
}

void BulletPhysicsManager::setSceneRestitutionCoefficient(
    const double restitutionCoefficient) {
  std::dynamic_pointer_cast<physics::BulletRigidObject, physics::RigidObject>(
      sceneNode_)
      ->setRestitutionCoefficient(restitutionCoefficient);
}

const double BulletPhysicsManager::getMargin(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return std::dynamic_pointer_cast<BulletRigidObject>(
               existingObjects_.at(physObjectID))
        ->getMargin();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

const double BulletPhysicsManager::getSceneFrictionCoefficient() {
  return std::dynamic_pointer_cast<physics::BulletRigidObject,
                                   physics::RigidObject>(sceneNode_)
      ->getFrictionCoefficient();
}

const double BulletPhysicsManager::getSceneRestitutionCoefficient() {
  return std::dynamic_pointer_cast<physics::BulletRigidObject,
                                   physics::RigidObject>(sceneNode_)
      ->getRestitutionCoefficient();
}

}  // namespace physics
}  // namespace esp
