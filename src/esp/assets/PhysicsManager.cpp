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


void PhysicsManager::initObject(Importer& importer,
                                   const AssetInfo& info,
                                   const MeshMetaData& metaData,
                                   scene::SceneNode& parent,
                                   DrawableGroup* drawables,
                                   int objectID) {
  std::unique_ptr<Magnum::Trade::ObjectData3D> objectData =
      importer.object3D(objectID);
  if (!objectData) {
    LOG(ERROR) << "Cannot import object " << importer.object3DName(objectID)
               << ", skipping";
    return;
  }

  // Add the object to the scene and set its transformation
  scene::SceneNode& node = parent.createChild();
  node.MagnumObject::setTransformation(objectData->transformation());

  const int meshStart = metaData.meshIndex.first;
  const int materialStart = metaData.materialIndex.first;
  const int meshID = meshStart + objectData->instance();

  // Add a drawable if the object has a mesh and the mesh is loaded
  if (objectData->instanceType() == Magnum::Trade::ObjectInstanceType3D::Mesh &&
      objectData->instance() != ID_UNDEFINED && meshes_[meshID]) {
    const int materialIDLocal =
        static_cast<Magnum::Trade::MeshObjectData3D*>(objectData.get())
            ->material();

    Magnum::GL::Mesh& mesh = *meshes_[meshID]->getMagnumGLMesh();
    const int materialID = materialStart + materialIDLocal;

    Magnum::GL::Texture2D* texture = nullptr;
    // Material not available / not loaded, use a default material
  }    // add a drawable

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

    // TODO (JK) Bottom up search gives bus error because scene's rootnode does
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


// TODO (JH) _bWorld(bWorld) syntax?
RigidBody::RigidBody(MagnumObject* parent, Magnum::Float mass, btCollisionShape* bShape, btDynamicsWorld& bWorld): MagnumObject{parent}, _bWorld(bWorld) {
      /* Calculate inertia so the object reacts as it should with
         rotation and everything */
      btVector3 bInertia(0.0f, 0.0f, 0.0f);
      if(mass != 0.0f) bShape->calculateLocalInertia(mass, bInertia);

      /* Bullet rigid body setup */
      auto* motionState = new Magnum::BulletIntegration::MotionState{*this};
      _bRigidBody.emplace(btRigidBody::btRigidBodyConstructionInfo{
          mass, &motionState->btMotionState(), bShape, bInertia});
      _bRigidBody->forceActivationState(DISABLE_DEACTIVATION);
      bWorld.addRigidBody(_bRigidBody.get());
  }

btRigidBody& RigidBody::rigidBody() {
  return *_bRigidBody;
  }

}
}  // namespace esp
