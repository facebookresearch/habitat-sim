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
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

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


bool PhysicsManager::initPhysics() {
  LOG(INFO) << "Initializing Physics Engine...";  

  _bCollisionConfig = new btDefaultCollisionConfiguration();
  _bDispatcher = new btCollisionDispatcher(_bCollisionConfig);
  //btGImpactCollisionAlgorithm::registerAlgorithm(_bDispatcher);
  _bBroadphase = new btDbvtBroadphase();
  _bSolver = new btSequentialImpulseConstraintSolver();
  _bWorld = new btDiscreteDynamicsWorld(_bDispatcher, _bBroadphase, _bSolver, _bCollisionConfig);


  //btCollisionDispatcher *dispatcher = static_cast<btCollisionDispatcher *>(_bWorld.getDispatcher());
  //btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

  _debugDraw.setMode(Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe);
  _bWorld->setGravity({0.0f, -10.0f, 0.0f});
  //_bWorld.setGravity({0.0f, 0.0f, -10.0f});
  //_bWorld.setDebugDrawer(&_debugDraw);

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


// Bullet Mesh conversion adapted from: https://github.com/mosra/magnum-integration/issues/20
void PhysicsManager::initObject(const AssetInfo& info,
                                const MeshMetaData& metaData,
                                Magnum::Trade::MeshData3D& meshData,
                                physics::BulletRigidObject* physObject,
                                const std::string& shapeType, /* = "TriangleMeshShape" */
                                bool zero_mass /* = false */) {
  // TODO (JH) should meshData better be a pointer?
  // such that if (!meshData) return
  if(meshData.primitive() != Magnum::MeshPrimitive::Triangles) {
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return;
  }

  // TODO (JH) weight is currently hardcoded, later should load from some config file
  float mass = 0.0f;
  // metaData.mass
  if (! zero_mass) {
    mass = meshData.indices().size() * 0.001f;    
    LOG(INFO) << "Nonzero mass";
  } else {
    LOG(INFO) << "Zero mass";    
  }

  LOG(INFO) << "Initialize: before";
  physObject->initialize(mass, meshData, *_bWorld);
  LOG(INFO) << "Initialize: after";
  //object->syncPose();
  //new ColoredDrawable{*ground, _shader, _box, 0xffffff_rgbf,
  //      Matrix4::scaling({4.0f, 0.5f, 4.0f}), _drawables};

  // TODO: DEBUGGING PURPOSE, (JH) strangely causes segfault with redraw()
  // maybe due to ground not being in drawables
  //btBoxShape _bGroundShape{{4.0f, 0.5f, 4.0f}};
}

void PhysicsManager::debugSceneGraph(const MagnumObject* root) {
  auto& children = root -> children();
  const scene::SceneNode* root_ = static_cast<const scene::SceneNode*>(root);
  LOG(INFO) << "SCENE NODE " << root_->getId() << " Position " << root_->getAbsolutePosition();       
  if (!children.isEmpty()) {
    for (const MagnumObject& child: children) {
      PhysicsManager::debugSceneGraph(&(child));
    }
  } else {
    LOG(INFO) << "SCENE NODE is leaf node."; 
  }

  // TODO (JH) Bottom up search gives bus error because scene's rootnode does
  // not point to nullptr, but something strange
  /*LOG(INFO) << "INSPECTING NODE " << root;
  const scene::SceneNode* parent = static_cast<const scene::SceneNode*>(root->parent());
  if (parent != NULL) {
    LOG(INFO) << "SCENE NODE " << root->getId() << " parent " << parent;
    LOG(INFO) << "SCENE NODE " << parent->getId() << " parent " << parent;
    //debugSceneGraph(parent);
  } else {
    LOG(INFO) << "SCENE NODE " << root->getId() << "IS ROOT NODE";      
  }*/
}

void PhysicsManager::stepPhysics() {
  // ==== Physics stepforward ======
  _bWorld->stepSimulation(_timeline.previousFrameDuration(), _maxSubSteps, _fixedTimeStep);
  //_bWorld.debugDrawWorld();
  //LOG(INFO) << "Step physics forward previous: " << _timeline.previousFrameDuration();
  int numObjects = _bWorld->getNumCollisionObjects();
  //LOG(INFO) << "Num collision objects" << numObjects;
}

void PhysicsManager::nextFrame() {
  _timeline.nextFrame();
}

}
}  // namespace esp
