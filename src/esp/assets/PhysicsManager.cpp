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

  _bCollisionConfig = new btDefaultCollisionConfiguration();
  _bDispatcher = new btCollisionDispatcher(_bCollisionConfig);
  btGImpactCollisionAlgorithm::registerAlgorithm(_bDispatcher);
  _bBroadphase = new btDbvtBroadphase();
  _bSolver = new btSequentialImpulseConstraintSolver();
  _bWorld = new btDiscreteDynamicsWorld(_bDispatcher, _bBroadphase, _bSolver,
                                        _bCollisionConfig);

  // btCollisionDispatcher *dispatcher = static_cast<btCollisionDispatcher
  // *>(_bWorld.getDispatcher());
  // btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

  _debugDraw.setMode(Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe);
  _bWorld->setGravity({0.0f, -10.0f, 0.0f});
  //_bWorld.setGravity({0.0f, 0.0f, -10.0f});
  //_bWorld.setDebugDrawer(&_debugDraw);

  physicsNode = node;

  _timeline.start();
  _initialized = true;
  _do_profile = do_profile;
  LOG(INFO) << "Initialized Physics Engine.";

  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
  if (_initialized) {
    delete _bCollisionConfig;
    delete _bDispatcher;
    delete _bBroadphase;
    delete _bSolver;
    delete _bWorld;
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
    LOG(INFO) << "Initialize instance: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize instance: after";
    physObject->syncPose();
  } 
  else if (info.type == AssetType::FRL_INSTANCE_MESH) {
    // FRL mesh
    LOG(INFO) << "Initialize FRL: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize FRL: after, success " << objectSuccess;
    physObject->syncPose();
  }
  else {
    // GLB mesh data
    LOG(INFO) << "Initialize GLB: before";
    objectSuccess = physObject->initializeScene(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize GLB: after, success " <<objectSuccess;
    physObject->syncPose();
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
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize: after";
  } 
  else if (info.type == AssetType::FRL_INSTANCE_MESH) {
    // FRL mesh
    mass = meshGroup[0].indices.size() * 0.001f;
    LOG(INFO) << "Initialize FRL: before";
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize FRL: after";
  }
  else {
    // GLB mesh data
    mass = meshGroup[0].indices.size() * 0.001f;
    LOG(INFO) << "Initialize FRL: before";
    objectSuccess = physObject->initializeObject(info, mass, meshGroup, *_bWorld);
    LOG(INFO) << "Initialize FRL: after";
  }

  physObject->syncPose();
  return objectSuccess;
}


bool PhysicsManager::isMeshPrimitiveValid(CollisionMeshData& meshData) {
  if (meshData.primitive != Magnum::MeshPrimitive::Triangles) {
    if (meshData.primitive == Magnum::MeshPrimitive::Lines) {
      LOG(INFO) << "Primitive Lines";
    }
    if (meshData.primitive == Magnum::MeshPrimitive::Points) {
      LOG(INFO) << "Primitive Points";
    }
    if (meshData.primitive == Magnum::MeshPrimitive::LineLoop) {
      LOG(INFO) << "Primitive Line loop";
    }
    if (meshData.primitive == Magnum::MeshPrimitive::LineStrip) {
      LOG(INFO) << "Primitive Line Strip";
    }
    if (meshData.primitive == Magnum::MeshPrimitive::TriangleStrip) {
      LOG(INFO) << "Primitive Triangle Strip";
    }
    if (meshData.primitive == Magnum::MeshPrimitive::TriangleFan) {
      LOG(INFO) << "Primitive Triangle Fan";
    }
    LOG(ERROR) << "Primitive " << int(meshData.primitive);
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return false;
  } else {
    return true;
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
  //_bWorld->stepSimulation(_timeline.previousFrameDuration(), _maxSubSteps,
  //                        _fixedTimeStep);

  auto start = std::chrono::system_clock::now();
  _bWorld->stepSimulation(_timeline.previousFrameDuration(), _maxSubSteps,
                          _fixedTimeStep);
  auto end = std::chrono::system_clock::now();

  std::chrono::duration<float> elapsed_seconds = end-start;
  std::time_t end_time = std::chrono::system_clock::to_time_t(end);

  if (_do_profile) {
    _total_frames += 1;
    _total_time += static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Step physics fps: " << 1.0f / static_cast<float>(elapsed_seconds.count());
    LOG(INFO) << "Average physics fps: " << 1.0f / (_total_time / _total_frames);
  }
  
  int numObjects = _bWorld->getNumCollisionObjects();
  // LOG(INFO) << "Num collision objects" << numObjects;
}

void PhysicsManager::nextFrame() {
  _timeline.nextFrame();
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
