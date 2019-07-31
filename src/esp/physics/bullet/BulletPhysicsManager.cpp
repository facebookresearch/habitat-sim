// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletRigidObject.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

bool BulletPhysicsManager::initPhysics(scene::SceneNode* node,
                                       Magnum::Vector3d gravity,
                                       bool do_profile) {
  LOG(INFO) << "Initializing Bullet Physics Engine...";
  activePhysSimLib_ = BULLET;

  //! We can potentially use other collision checking algorithms, by
  //! uncommenting the line below
  // btGImpactCollisionAlgorithm::registerAlgorithm(&bDispatcher_);
  bWorld_ = std::make_shared<btDiscreteDynamicsWorld>(
      &bDispatcher_, &bBroadphase_, &bSolver_, &bCollisionConfig_);
  // currently GLB meshes are y-up
  bWorld_->setGravity({gravity[0], gravity[1], gravity[2]});

  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = std::dynamic_pointer_cast<physics::RigidObject,
                                         physics::BulletRigidObject>(
      std::make_shared<physics::BulletRigidObject>(physicsNode_));

  initialized_ = true;
  do_profile_ = do_profile;

  // LOG(INFO) << "Initialized Bullet Physics Engine.";
  return true;
}

BulletPhysicsManager::~BulletPhysicsManager() {
  LOG(INFO) << "Deconstructing BulletPhysicsManager";
}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool BulletPhysicsManager::addScene(
    const assets::AssetInfo& info,
    std::vector<assets::CollisionMeshData> meshGroup) {
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
  // bool sceneSuccess = sceneNode_->initializeScene(meshGroup, *bWorld_);
  bool sceneSuccess =
      std::dynamic_pointer_cast<physics::BulletRigidObject,
                                physics::RigidObject>(sceneNode_)
          ->initializeScene(meshGroup, *bWorld_);
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}

const int BulletPhysicsManager::makeRigidObject(
    std::vector<assets::CollisionMeshData> meshGroup,
    assets::PhysicsObjectMetaData metaData) {
  //! Create new physics object (child node of sceneNode_)
  existingObjects_.emplace_back(
      std::make_unique<physics::BulletRigidObject>(sceneNode_.get()));

  const int nextObjectID_ = existingObjects_.size();
  //! Instantiate with mesh pointer
  bool objectSuccess =
      dynamic_cast<physics::BulletRigidObject*>(existingObjects_.back().get())
          ->initializeObject(metaData, meshGroup, *bWorld_);
  if (!objectSuccess) {
    return -1;
  }
  return nextObjectID_ - 1;
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

void BulletPhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_)
    return;

  if (dt < 0)
    dt = fixedTimeStep_;

  // ==== Physics stepforward ======
  auto start = std::chrono::system_clock::now();

  // Alex NOTE: worldTime_ will always be a multiple of fixedTimeStep_
  int numSubStepsTaken =
      bWorld_->stepSimulation(dt, maxSubSteps_, fixedTimeStep_);
  worldTime_ += numSubStepsTaken * fixedTimeStep_;

  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end - start;
  // LOG(INFO) << "Step physics dt | compute time: " << dt << " | " <<
  // elapsed_seconds.count();
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  // TODO (JH): hacky way to do physics profiling.
  // Should later move to example.py
  if (do_profile_) {
    total_frames_ += 1;
    total_time_ += static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Step physics fps: "
              << 1.0f / static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Average physics fps: "
              << 1.0f / (total_time_ / total_frames_);
  }

  // int numObjects = bWorld_->getNumCollisionObjects();
}

}  // namespace physics
}  // namespace esp
