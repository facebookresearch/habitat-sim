// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

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
#include "LinearMath/btQuickprof.h"

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

bool PhysicsManager::initPhysics(scene::SceneNode* node) {
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
bool PhysicsManager::initObject(
    const AssetInfo& info,
    const MeshMetaData& metaData,
    Magnum::Trade::MeshData3D& meshData,
    physics::BulletRigidObject* physObject,
    const std::string& shapeType, /* = "TriangleMeshShape" */
    bool zero_mass /* = false */) {
  // TODO (JH) should meshData better be a pointer?
  // such that if (!meshData) return

  if (meshData.primitive() != Magnum::MeshPrimitive::Triangles) {
    if (meshData.primitive() == Magnum::MeshPrimitive::Lines) {
      LOG(INFO) << "Primitive Lines";
    }
    if (meshData.primitive() == Magnum::MeshPrimitive::Points) {
      LOG(INFO) << "Primitive Points";
    }
    if (meshData.primitive() == Magnum::MeshPrimitive::LineLoop) {
      LOG(INFO) << "Primitive Line loop";
    }
    if (meshData.primitive() == Magnum::MeshPrimitive::LineStrip) {
      LOG(INFO) << "Primitive Line Strip";
    }
    if (meshData.primitive() == Magnum::MeshPrimitive::TriangleStrip) {
      LOG(INFO) << "Primitive Triangle Strip";
    }
    if (meshData.primitive() == Magnum::MeshPrimitive::TriangleFan) {
      LOG(INFO) << "Primitive Triangle Fan";
    }
    LOG(ERROR) << "Primitive " << int(meshData.primitive());
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return false;
  }

  // TODO (JH) weight is currently hardcoded, later should load from some config
  // file
  float mass = 0.0f;
  // metaData.mass
  if (!zero_mass) {
    mass = meshData.indices().size() * 0.001f;
    LOG(INFO) << "Nonzero mass";
  } else {
    LOG(INFO) << "Zero mass";
  }

  LOG(INFO) << "Initialize: before";
  bool objectSuccess = physObject->initialize(mass, meshData, *_bWorld);
  LOG(INFO) << "Initialize: after";
  physObject->syncPose();

  return objectSuccess;
}

bool PhysicsManager::initFRLObject(
    const AssetInfo& info,
    const MeshMetaData& metaData,
    FRLInstanceMeshData* meshData,
    physics::BulletRigidObject* physObject,
    const std::string& shapeType, /* = "TriangleMeshShape" */
    bool zero_mass /* = false */) {
  Magnum::GL::Mesh* mesh = &meshData->getRenderingBuffer()->mesh;
  float mass = 0.0f;
  if (!zero_mass) {
    mass = mesh->count() * 0.001f;
    LOG(INFO) << "Nonzero mass";
  } else {
    LOG(INFO) << "Zero mass";
  }

  LOG(INFO) << "Initialize FRL: before";
  bool objectSuccess = physObject->initializeFRL(mass, meshData, *_bWorld);
  LOG(INFO) << "Initialize FRL: after";
  return objectSuccess;
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
  _bWorld->stepSimulation(_timeline.previousFrameDuration(), _maxSubSteps,
                          _fixedTimeStep);
  //_bWorld.debugDrawWorld();
  // LOG(INFO) << "Step physics fps: " << 1.0f /
  // _timeline.previousFrameDuration();
  int numObjects = _bWorld->getNumCollisionObjects();
  // LOG(INFO) << "Num collision objects" << numObjects;
}

void PhysicsManager::nextFrame() {
  _timeline.nextFrame();
  checkActiveObjects();
  //CProfileManager::dumpAll();
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
