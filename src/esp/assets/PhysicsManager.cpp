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
#include <Magnum/Math/Color.h>
#include <Magnum/BulletIntegration/Integration.h>
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

bool PhysicsManager::initPhysics(scene::SceneNode* node,
                                 bool do_profile) {
  LOG(INFO) << "Initializing Physics Engine...";

  //! We can potentially use other collision checking algorithms, by 
  //! uncommenting the line below
  //btGImpactCollisionAlgorithm::registerAlgorithm(&bDispatcher_);
  bWorld_ = std::make_shared<btDiscreteDynamicsWorld>(&bDispatcher_, 
      &bBroadphase_, &bSolver_, &bCollisionConfig_);

  // TODO (JH): currently GLB meshes are y-up, the gravity direction is hardcoded
  bWorld_->setGravity({0.0f, -10.0f, 0.0f});
  //bWorld_->setGravity({0.0f, 0.0f, -10.0f});

  // TODO (JH): debugDrawer is currently not compatible with our example cpp
  //debugDraw_.setMode(Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe);
  //bWorld_->setDebugDrawer(&_debugDraw);

  physicsNode = node;

  timeline_.start();
  initialized_ = true;
  do_profile_ = do_profile;

  // Initialize debugger
  LOG(INFO) << "Debug drawing";
  Magnum::DebugTools::ResourceManager::instance()
    .set("bulletForce", Magnum::DebugTools::ForceRendererOptions{}
    .setSize(5.0f)
    .setColor(Magnum::Color3(1.0f, 0.1f, 0.1f)));

  LOG(INFO) << "Initialized Physics Engine.";
  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
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

  float mass = 0.0f;
  bool sceneSuccess;
  if (info.type == AssetType::INSTANCE_MESH) {              // ._semantic.ply mesh data
    LOG(INFO) << "Initialize instance scene";
  } else if (info.type == AssetType::FRL_INSTANCE_MESH) {   // FRL mesh
    LOG(INFO) << "Initialize FRL scene";
  } else {                                                  // GLB mesh data
    LOG(INFO) << "Initialize GLB scene";
  }
  sceneSuccess = physObject->initializeScene(info, mass, meshGroup, *bWorld_.get());
  //physObject->syncPose();
  
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}


int PhysicsManager::initObject(
    const AssetInfo& info,
    const MeshMetaData& metaData,
    std::vector<CollisionMeshData> meshGroup,
    physics::BulletRigidObject* physObject) {

  // Test Mesh primitive is valid
  for (CollisionMeshData& meshData: meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {return false;}
  }

  // TODO (JH): hacked mass value
  float mass = meshGroup[0].indices.size() * 0.001f;;
  switch (info.type) {
    case AssetType::INSTANCE_MESH:      // _semantic.ply
      LOG(INFO) << "Initialize PLY object"; break;  
    case AssetType::FRL_INSTANCE_MESH:  // FRL mesh
      LOG(INFO) << "Initialize FRL object"; break;
    default:                            // GLB mesh
      LOG(INFO) << "Initialize GLB object";
  }

  bool objectSuccess = physObject->initializeObject(info, mass, meshGroup, *bWorld_.get());
  physObject->syncPose();
  // Enable force debugging
  //physObject->debugForce(debugDrawables);

  if (!objectSuccess) {return -1;}

  // Maintain object resource
  dynamicObjects_[nextObjectID_] = physObject;

  // Increment simple object ID tracker
  nextObjectID_ += 1;
  return nextObjectID_-1;
}

//! Check if mesh primitive is compatible with physics
bool PhysicsManager::isMeshPrimitiveValid(CollisionMeshData& meshData) {
  if (meshData.primitive == Magnum::MeshPrimitive::Triangles) {
    // Only triangle mesh works
    return true;
  } else {
    switch(meshData.primitive) {
      case Magnum::MeshPrimitive::Lines:
        LOG(ERROR) << "Invalid primitive: Lines"; break;
      case Magnum::MeshPrimitive::Points:
        LOG(ERROR) << "Invalid primitive: Points"; break;
      case Magnum::MeshPrimitive::LineLoop:
        LOG(ERROR) << "Invalid primitive Line loop"; break;
      case Magnum::MeshPrimitive::LineStrip:
        LOG(ERROR) << "Invalid primitive Line Strip"; break;
      case Magnum::MeshPrimitive::TriangleStrip:
        LOG(ERROR) << "Invalid primitive Triangle Strip"; break;
      case Magnum::MeshPrimitive::TriangleFan:
        LOG(ERROR) << "Invalid primitive Triangle Fan"; break;
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
  if (children.isEmpty()) {
    LOG(INFO) << "SCENE NODE is leaf node.";
  } else {
    for (const MagnumObject& child : children) {
      PhysicsManager::debugSceneGraph(&(child));
    }    
  }
  // TODO (JH) Bottom up search gives bus error because scene"s rootnode does
  // not point to nullptr, but something strange
}

void PhysicsManager::stepPhysics() {
  // ==== Physics stepforward ======
  auto start = std::chrono::system_clock::now();
  bWorld_->stepSimulation(timeline_.previousFrameDuration(), maxSubSteps_,
                          fixedTimeStep_);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  // TODO (JH): hacky way to do physics profiling. 
  // Should later move to example.py
  if (do_profile_) {
    total_frames_ += 1;
    total_time_ += static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Step physics fps: " << 1.0f / 
        static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Average physics fps: " << 1.0f / 
        (total_time_ / total_frames_);
  }
  
  int numObjects = bWorld_->getNumCollisionObjects();

}

void PhysicsManager::nextFrame() {
  timeline_.nextFrame();
  checkActiveObjects();
}

//! Profile function. In BulletPhysics stationery objects are
//! marked as inactive to speed up simulation. This function
//! helps checking how many objects are active/inactive at any
//! time step
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
  //LOG(INFO) << "Nodes total " << numTotal << " active " << numActive;
}


void PhysicsManager::applyForce(
      const int objectID,
      Magnum::Vector3 force,
      Magnum::Vector3 relPos) {
  physics::BulletRigidObject* physObject = dynamicObjects_[objectID];

  physObject->rigidBody().applyForce(btVector3(force), btVector3(relPos));
  //physObject->setDebugForce(force);
}

void PhysicsManager::applyImpulse(
      const int objectID,
      Magnum::Vector3 impulse,
      Magnum::Vector3 relPos) {
  physics::BulletRigidObject* physObject = dynamicObjects_[objectID];

  physObject->rigidBody().applyImpulse(btVector3(impulse), btVector3(relPos));
}


}  // namespace assets
}  // namespace esp
