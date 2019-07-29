// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "BulletPhysicsManager.h"
#include "BulletRigidObject.h"

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

  timeline_.start();
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

int BulletPhysicsManager::addObject(const std::string configFile,
                                    PhysicalObjectType objectType,
                                    DrawableGroup* drawables) {
  std::vector<assets::CollisionMeshData> meshGroup =
      resourceManager->getCollisionMesh(configFile);

  assets::PhysicsObjectMetaData metaData =
      resourceManager->getPhysicsMetaData(configFile);

  LOG(INFO) << "Add object: before check";
  //! Test Mesh primitive is valid
  for (assets::CollisionMeshData& meshData : meshGroup) {
    LOG(INFO) << "    meshData>>>>>>>>>>";
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }
  LOG(INFO) << "Add object: before child node";

  //! Create new physics object (child node of sceneNode_)
  std::shared_ptr<physics::BulletRigidObject> physObject =
      std::make_shared<physics::BulletRigidObject>(sceneNode_.get());
  objectNodes_.emplace_back(physObject);

  LOG(INFO) << "Add object: before initialize";

  //! Instantiate with mesh pointer
  bool objectSuccess =
      physObject->initializeObject(metaData, objectType, meshGroup, *bWorld_);
  if (!objectSuccess) {
    LOG(ERROR) << "Initialize unsuccessful";
    return -1;
  }

  LOG(INFO) << "Add object: before render stack";

  //! Enable force debugging
  // physObject->debugForce(debugDrawables);

  //! Maintain object resource
  existingObjects_[nextObjectID_] = physObject;
  existingObjNames_[nextObjectID_] = configFile;
  existingObjTypes_[nextObjectID_] = objectType;
  //! Increment simple object ID tracker
  nextObjectID_ += 1;

  //! IMPORTANT: invoke resourceManager to draw object

  int resObjectID =
      resourceManager->addObject(configFile, physObject.get(), drawables);
  // int resObjectID = resourceManager->addObject(configFile, physicsNode_,
  // drawables);

  if (resObjectID < 0) {
    return -1;
  }

  LOG(INFO) << "Add object: after render stack";
  return nextObjectID_ - 1;
}

void BulletPhysicsManager::stepPhysics() {
  // We don't step uninitialized physics sim...
  if (!initialized_)
    return;

  // ==== Physics stepforward ======
  auto start = std::chrono::system_clock::now();
  bWorld_->stepSimulation(timeline_.previousFrameDuration(), maxSubSteps_,
                          fixedTimeStep_);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end - start;
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

  int numObjects = bWorld_->getNumCollisionObjects();
}

}  // namespace physics
}  // namespace esp
