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
  LOG(INFO) << "Initializing Bullet Physics Engine...";
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

BulletPhysicsManager::~BulletPhysicsManager() {
  LOG(INFO) << "Deconstructing BulletPhysicsManager";
}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addScene(
    const assets::AssetInfo& info,
    assets::PhysicsSceneAttributes& physicsSceneAttributes,
    std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  switch (info.type) {
    case assets::AssetType::INSTANCE_MESH:
      LOG(INFO) << "Initialize instance scene";
      break;  // ._semantic.ply mesh data
    case assets::AssetType::FRL_INSTANCE_MESH:
      LOG(INFO) << "Initialize FRL scene";
      break;  // FRL mesh
    default:
      LOG(INFO) << "Initialize GLB scene";  // GLB mesh data
  }

  //! Initialize scene
  bool sceneSuccess =
      std::dynamic_pointer_cast<physics::BulletRigidObject,
                                physics::RigidObject>(sceneNode_)
          ->initializeScene(physicsSceneAttributes, meshGroup, bWorld_);
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}

int BulletPhysicsManager::makeRigidObject(
    std::vector<assets::CollisionMeshData>& meshGroup,
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
    LOG(INFO) << "deleted";
    deallocateObjectID(newObjectID);
    existingObjects_.erase(newObjectID);
    return -1;
  }
  LOG(INFO) << "Allocate new object id " << newObjectID << " "
            << existingObjects_.at(newObjectID);
  return newObjectID;
}

//! Check if mesh primitive is compatible with physics
bool BulletPhysicsManager::isMeshPrimitiveValid(
    assets::CollisionMeshData& meshData) {
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
  LOG(INFO) << "Gravity " << gravity[0] << ", " << gravity[1] << ", "
            << gravity[2];

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
  if (!initialized_)
    return;
  if (dt < 0)
    dt = fixedTimeStep_;

  // ==== Physics stepforward ======
  // NOTE: chrono breaks devfair build
  // auto start = std::chrono::system_clock::now();

  // Alex NOTE: worldTime_ will always be a multiple of sceneMetaData_.timestep
  int numSubStepsTaken =
      bWorld_->stepSimulation(dt, maxSubSteps_, fixedTimeStep_);
  worldTime_ += numSubStepsTaken * fixedTimeStep_;

  // auto end = std::chrono::system_clock::now();

  // std::chrono::duration<float> elapsed_seconds = end - start;
  // LOG(INFO) << "Step physics dt | compute time: " << dt << " | " <<
  // elapsed_seconds.count();
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
    return -1.0;
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
