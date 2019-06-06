// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <functional>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/PixelFormat.h>
#include <Magnum/Trade/AbstractImporter.h>
#include <Magnum/Trade/ImageData.h>
#include <Magnum/Trade/MeshObjectData3D.h>
#include <Magnum/Trade/PhongMaterialData.h>
#include <Magnum/Trade/SceneData.h>
#include <Magnum/Trade/TextureData.h>
//#include <Magnum/Platform/Sdl2Application.h>    ## Tried to make physics example work

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


bool PhysicsManager::initPhysics(scene::SceneNode* scene) {
  LOG(INFO) << "Initializing Physics Engine...";  
  _debugDraw.setMode(Magnum::BulletIntegration::DebugDraw::Mode::DrawWireframe);
  _bWorld.setGravity({0.0f, -10.0f, 0.0f});
  _bWorld.setDebugDrawer(&_debugDraw);

  _scene = scene;  
  _timeline.start();

  LOG(INFO) << "Initialized Physics Engine.";  
  return true;
}

Magnum::GL::AbstractShaderProgram* PhysicsManager::getPhysicsEngine(
    ShaderType type) {
  if (shaderPrograms_.count(type) == 0) {
    switch (type) {
      case INSTANCE_MESH_SHADER: {
        shaderPrograms_[INSTANCE_MESH_SHADER] =
            std::make_shared<gfx::GenericShader>(
                gfx::GenericShader::Flag::VertexColored |
                gfx::GenericShader::Flag::PrimitiveIDTextured);
      } break;

      case PTEX_MESH_SHADER: {
        shaderPrograms_[PTEX_MESH_SHADER] =
            std::make_shared<gfx::PTexMeshShader>();
      } break;

      case COLORED_SHADER: {
        shaderPrograms_[COLORED_SHADER] =
            std::make_shared<gfx::GenericShader>();
      } break;

      case VERTEX_COLORED_SHADER: {
        shaderPrograms_[VERTEX_COLORED_SHADER] =
            std::make_shared<gfx::GenericShader>(
                gfx::GenericShader::Flag::VertexColored);
      } break;

      case TEXTURED_SHADER: {
        shaderPrograms_[TEXTURED_SHADER] = std::make_shared<gfx::GenericShader>(
            gfx::GenericShader::Flag::Textured);
      } break;

      default:
        return nullptr;
        break;
    }
  }
  return shaderPrograms_[type].get();
}


// Bullet Mesh conversion adapted from: https://github.com/mosra/magnum-integration/issues/20
void PhysicsManager::initObject(Importer& importer,
                                const AssetInfo& info,
                                const MeshMetaData& metaData,
                                Magnum::Trade::MeshData3D& meshData,
                                const std::string& shapeType /* = "TriangleMeshShape" */) {
  // TODO (JH) should meshData better be a pointer?
  // such that if (!meshData) return
  if(meshData.primitive() != Magnum::MeshPrimitive::Triangles) {
    LOG(ERROR) << "Cannot load collision mesh, skipping";
    return;
  }
  /* this is a collision mesh, convert to bullet mesh */
  // TODO (JH) would this introduce memory leak?
  btIndexedMesh bulletMesh;
  LOG(INFO) << "Mesh indices count " << meshData.indices().size();

  // TODO (JH) weight is currently hardcoded, later should load from some config file
  float weight = meshData.indices().size() * 0.001f;

  bulletMesh.m_numTriangles = meshData.indices().size()/3;
  bulletMesh.m_triangleIndexBase = reinterpret_cast<const unsigned char *>(meshData.indices().data());
  bulletMesh.m_triangleIndexStride = 3 * sizeof(Magnum::UnsignedInt);
  bulletMesh.m_numVertices = meshData.positions(0).size();
  bulletMesh.m_vertexBase = reinterpret_cast<const unsigned char *>(meshData.positions(0).data());
  bulletMesh.m_vertexStride = sizeof(Magnum::Vector3);
  bulletMesh.m_indexType = PHY_INTEGER;
  bulletMesh.m_vertexType = PHY_FLOAT;


  btCollisionShape* shape = nullptr;
  auto tivArray = new btTriangleIndexVertexArray();
  tivArray->addIndexedMesh(bulletMesh, PHY_INTEGER);
  if(shapeType == "TriangleMeshShape") {
      /* exact shape, but worse performance */
      shape = new btBvhTriangleMeshShape(tivArray, true);
  } else {
      /* convex hull, but better performance */
      shape = new btConvexTriangleMeshShape(tivArray, true);
  } /* btConvexHullShape can be even more performant */

  LOG(INFO) << "Making rigid body: before";
  //auto* object = new RigidBody{static_cast<MagnumObject&>(*_scene), weight, shape, _bWorld};
  //auto* object = new RigidBody{static_cast<MagnumObject*>(_scene), weight, shape, _bWorld};

  auto* object = new RigidBody{_scene, weight, shape, _bWorld};
  LOG(INFO) << "Making rigid body: after";
  //object->syncPose();            
  //new ColoredDrawable{*ground, _shader, _box, 0xffffff_rgbf,
  //      Matrix4::scaling({4.0f, 0.5f, 4.0f}), _drawables};

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
  _bWorld.stepSimulation(_timeline.previousFrameDuration(), _maxSubSteps, _fixedTimeStep);
  //_bWorld.debugDrawWorld();
  LOG(INFO) << "Step physics forward previous: " << _timeline.previousFrameDuration();
}

void PhysicsManager::nextFrame() {
  _timeline.nextFrame();
}

// TODO (JH) what role does Object3D{parent} play in example?
RigidBody::RigidBody(scene::SceneNode* parent, Magnum::Float mass, btCollisionShape* bShape, btDynamicsWorld& bWorld): 
  //MagnumObject{nullptr}, 
  scene::SceneNode{*parent},
  _bWorld(bWorld) {

  //scene::SceneNode::setParent(*parent);   // not compile: `setParent` is non-static
  //this->setParent(parent);                // segfault
  //MagnumObject::setParent(parent);        // segfault

  LOG(INFO) << "Done inheriting parent ";
  /* Calculate inertia so the object reacts as it should with
     rotation and everything */
  //btVector3 bInertia(0.0f, 0.0f, 0.0f);
  btVector3 bInertia(2.0f, 2.0f, 2.0f);
  //if(mass != 0.0f) bShape->calculateLocalInertia(mass, bInertia);
  bShape->calculateLocalInertia(300.0f, bInertia);

  /* Bullet rigid body setup */
  auto* motionState = new Magnum::BulletIntegration::MotionState{*this};
  _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
      mass, &motionState->btMotionState(), bShape, bInertia});
  _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
  bWorld.addRigidBody(_bRigidBody.get());
}

RigidBody::~RigidBody() {
    _bWorld.removeRigidBody(_bRigidBody.get());
}

btRigidBody& RigidBody::rigidBody() { return *_bRigidBody; }

/* needed after changing the pose from Magnum side */
void RigidBody::syncPose() {
    _bRigidBody->setWorldTransform(btTransform(transformationMatrix()));
}


}
}  // namespace esp
