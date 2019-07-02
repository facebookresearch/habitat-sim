// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>
#include <ctime>
#include <chrono>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#include "esp/geo/geo.h"
#include "esp/gfx/GenericDrawable.h"
#include "esp/gfx/GenericShader.h"
#include "esp/gfx/PTexMeshDrawable.h"
#include "esp/gfx/PTexMeshShader.h"
#include "esp/io/io.h"
#include "esp/io/json.h"
#include "esp/scene/SceneConfiguration.h"
#include "esp/scene/SceneGraph.h"

#include "FRLInstanceMeshData.h"
#include "GenericInstanceMeshData.h"
#include "GltfMeshData.h"
#include "Mp3dInstanceMeshData.h"
#include "PTexMeshData.h"
#include "PhysicsManager.h"

namespace esp {
namespace assets {

bool PhysicsManager::initPhysics(
      scene::SceneNode*     node,
      bool                  do_profile) {
  LOG(INFO) << "Initializing Physics Engine...";

  // TODO (JH): use unique pointer
  bCollisionConfig_ = new btDefaultCollisionConfiguration();
  bDispatcher_ = new btCollisionDispatcher(bCollisionConfig_);
  btGImpactCollisionAlgorithm::registerAlgorithm(bDispatcher_);
  bBroadphase_ = new btDbvtBroadphase();
  bSolver_ = new btSequentialImpulseConstraintSolver();
  bWorld_ = new btDiscreteDynamicsWorld(bDispatcher_, bBroadphase_, bSolver_,
                                        bCollisionConfig_);

  // btCollisionDispatcher *dispatcher = static_cast<btCollisionDispatcher
  // *>(bWorld_.getDispatcher());
  // btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

  debugDraw_.setMode(Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe);
  bWorld_->setGravity({0.0f, -10.0f, 0.0f});
  //bWorld_.setGravity({0.0f, 0.0f, -10.0f});
  //bWorld_.setDebugDrawer(&_debugDraw);

  physicsNode = node;

  timeline_.start();
  initialized_ = true;
  do_profile_ = do_profile;
  LOG(INFO) << "Initialized Physics Engine.";

  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
  if (initialized_) {
    delete bCollisionConfig_;
    delete bDispatcher_;
    delete bBroadphase_;
    delete bSolver_;
    delete bWorld_;
  }
}

void PhysicsManager::getPhysicsEngine() {}

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool PhysicsManager::initScene(
    const AssetInfo& info,
    const MeshMetaData& metaData,
    std::vector<CollisionMeshData> meshGroup,
    physics::BulletRigidObject* physObject) {

  // Test Mesh primitive is valid
  for (CollisionMeshData& meshData: meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {return false;}
  }

  float mass = 0.0f;    // TODO (JH) weight is currently hardcoded
  bool objectSuccess;
  if (info.type == AssetType::INSTANCE_MESH) {
    // ._semantic.ply mesh data
    LOG(INFO) << "Initialize instance scene: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize instance scene: after";
    //physObject->syncPose();
  } 
  else if (info.type == AssetType::FRL_INSTANCE_MESH) {
    // FRL mesh
    LOG(INFO) << "Initialize FRL scene: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize FRL scene: after, success " << objectSuccess;
    //physObject->syncPose();
  }
  else {
    // GLB mesh data
    LOG(INFO) << "Initialize GLB scene: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize GLB scene: after, success " <<objectSuccess;
    //physObject->syncPose();
  }

  LOG(INFO) << "Init scene done";
  return objectSuccess;
}


bool PhysicsManager::initObject(
    const AssetInfo& info,
    const MeshMetaData& metaData,
    std::vector<CollisionMeshData> meshGroup,
    physics::BulletRigidObject* physObject) {

  // Test Mesh primitive is valid
  for (CollisionMeshData& meshData: meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {return false;}
  }

  bool objectSuccess;
  float mass;
  if (info.type == AssetType::INSTANCE_MESH) {
    // _semantic.ply mesh
    // TODO (JH): hacked mass value
    mass = meshGroup[0].indices.size() * 0.001f;
    LOG(INFO) << "Nonzero mass";
    LOG(INFO) << "Initialize: before";
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize: after";
  } 
  else if (info.type == AssetType::FRL_INSTANCE_MESH) {
    // FRL mesh
    mass = meshGroup[0].indices.size() * 0.001f;
    LOG(INFO) << "Initialize FRL: before";
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize FRL: after";
  }
  else {
    // GLB mesh data
    mass = meshGroup[0].indices.size() * 0.001f;
    LOG(INFO) << "Initialize GLB: before";
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *bWorld_);
    LOG(INFO) << "Initialize GLB: after";
  }

  physObject->syncPose();
  return objectSuccess;
}

//! Check if mesh primitive is compatible with physics
bool PhysicsManager::isMeshPrimitiveValid(CollisionMeshData& meshData) {
  if (meshData.primitive == Magnum::MeshPrimitive::Triangles) {
    // Only triangle mesh works
    return true;
  } else {
    switch(meshData.primitive) {
      case Magnum::MeshPrimitive::Lines:
        LOG(ERROR) << "Invalid primitive: Lines";
      case Magnum::MeshPrimitive::Points:
        LOG(ERROR) << "Invalid primitive: Points";
      case Magnum::MeshPrimitive::LineLoop:
        LOG(ERROR) << "Invalid primitive Line loop";
      case Magnum::MeshPrimitive::LineStrip:
        LOG(ERROR) << "Invalid primitive Line Strip";
      case Magnum::MeshPrimitive::TriangleStrip:
        LOG(ERROR) << "Invalid primitive Triangle Strip";
      case Magnum::MeshPrimitive::TriangleFan:
        LOG(ERROR) << "Invalid primitive Triangle Fan";
      default:
        LOG(ERROR) << "Invalid primitive " << int(meshData.primitive);
    }
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return false;
  }
}

void PhysicsManager::debugSceneGraph(const MagnumObject* root) {
  auto& children = root->children();
  const scene::SceneNode* root_ = static_cast<const scene::SceneNode*>(root);
  LOG(INFO) << "SCENE NODE " << root_->getId() << " Position "
            << root_->getAbsolutePosition();
  if (!children.isEmpty()) {
    for (const MagnumObject& child : children) {
      PhysicsManager::debugSceneGraph(&(child));
    }
  } else {
    LOG(INFO) << "SCENE NODE is leaf node.";
  }

  // TODO (JH) Bottom up search gives bus error because scene"s rootnode does
  // not point to nullptr, but something strange
}

void PhysicsManager::stepPhysics() {
  // ==== Physics stepforward ======
  //bWorld_->stepSimulation(timeline_.previousFrameDuration(), maxSubSteps_,
  //                        fixedTimeStep_);

  auto start = std::chrono::system_clock::now();
  bWorld_->stepSimulation(timeline_.previousFrameDuration(), maxSubSteps_,
                          fixedTimeStep_);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  if (do_profile_) {
    total_frames_ += 1;
    total_time_ += static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Step physics fps: " << 1.0f / static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Average physics fps: " << 1.0f / (total_time_ / total_frames_);
  }
  
  int numObjects = bWorld_->getNumCollisionObjects();
  // LOG(INFO) << "Num collision objects" << numObjects;
}

void PhysicsManager::nextFrame() {
  timeline_.nextFrame();
  checkActiveObjects();
}

void PhysicsManager::checkActiveObjects() {
  if (physicsNode == nullptr) {
    return;
  }
  int numActive = 0;
  int numTotal = 0;
  for(auto& child: physicsNode->children()) {
    physics::BulletRigidObject* childNode = dynamic_cast<
        physics::BulletRigidObject*>(&child);
    if (childNode == nullptr) {
      //LOG(INFO) << "Child is null";
    } else {
      //LOG(INFO) << "Child is active: " << childNode->isActive();
      numTotal += 1;
      if (childNode->isActive()) {
        numActive += 1;
      }
    }
  }
  LOG(INFO) << "Nodes total " << numTotal << " active " << numActive;
}

}  // namespace assets
}  // namespace esp
