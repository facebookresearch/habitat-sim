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

#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/GltfMeshData.h"
#include "esp/assets/Mp3dInstanceMeshData.h"
#include "esp/assets/PTexMeshData.h"
#include "PhysicsManager.h"



namespace esp {
namespace physics {

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

// Bullet Mesh conversion adapted from:
// https://github.com/mosra/magnum-integration/issues/20
bool PhysicsManager::addScene(
    const assets::AssetInfo& info,
    scene::SceneNode* parent,
    std::vector<assets::CollisionMeshData> meshGroup) {

  // Test Mesh primitive is valid
  for (assets::CollisionMeshData& meshData: meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {return false;}
  }

  bool sceneSuccess;
  if (info.type == assets::AssetType::INSTANCE_MESH) {              
    // ._semantic.ply mesh data
    LOG(INFO) << "Initialize instance scene";
  } else if (info.type == assets::AssetType::FRL_INSTANCE_MESH) {   
    // FRL mesh
    LOG(INFO) << "Initialize FRL scene";
  } else {                                                  
    // GLB mesh data
    LOG(INFO) << "Initialize GLB scene";
  }

  //! Create new physics object (child node of parent)
  physScene = std::make_shared<physics::RigidObject>(parent);

  //! Initialize scene
  sceneSuccess = physScene->initializeScene(meshGroup, *bWorld_);
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}


int PhysicsManager::addObject(
    const int objectID,
    scene::SceneNode* parent,
    PhysicalObjectType objectType,
    DrawableGroup* drawables) 
{
  std::string objectName = resourceManager.getObjectKeyName(objectID);
  return addObject(objectName, parent, objectType, drawables);
}

int PhysicsManager::addObject(
    const std::string objectName,
    scene::SceneNode* parent,
    PhysicalObjectType objectType,
    DrawableGroup* drawables) 
{
  std::vector<assets::CollisionMeshData> meshGroup
      = resourceManager.getCollisionMesh(objectName);

  assets::PhysicsObjectMetaData metaData
      = resourceManager.getPhysicsMetaData(objectName);

  //! Test Mesh primitive is valid
  for (assets::CollisionMeshData& meshData: meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {return false;}
  }

  //! TODO (JH): hacked mass value
  //float mass = meshGroup[0].indices.size() * 0.001f;;

  //! Create new physics object (child node of parent)
  std::shared_ptr<physics::RigidObject> physObject =
      std::make_shared<physics::RigidObject>(parent);
  physObjects.emplace_back(physObject);

  //! Instantiate with mesh pointer
  bool objectSuccess = physObject->initializeObject(
      metaData, objectType, meshGroup, *bWorld_);
  if (!objectSuccess) {return -1;}
  
  //! Enable force debugging
  //physObject->debugForce(debugDrawables);

  //! Maintain object resource
  existingObjects_[nextObjectID_]  = physObject;
  existingObjNames_[nextObjectID_] = objectName;
  existingObjTypes_[nextObjectID_] = objectType;
  //! Increment simple object ID tracker
  nextObjectID_ += 1;

  //! IMPORTANT: invoke resourceManager to draw object
  resourceManager.addObject(objectName, physObject.get(), drawables);

  return nextObjectID_-1;
}

//! Check if mesh primitive is compatible with physics
bool PhysicsManager::isMeshPrimitiveValid(assets::CollisionMeshData& meshData) {
  if (meshData.primitive == Magnum::MeshPrimitive::Triangles) {
    //! Only triangle mesh works
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
    physics::RigidObject* childNode = dynamic_cast<
        physics::RigidObject*>(&child);
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


void PhysicsManager::applyForce(
    const int objectID,
    Magnum::Vector3 force,
    Magnum::Vector3 relPos) 
{
  std::shared_ptr<physics::RigidObject> physObject = existingObjects_[objectID];
  PhysicalObjectType objectType = existingObjTypes_[objectID];
  if (objectType == physics::PhysicalObjectType::DYNAMIC) {
    physObject->applyForce(force, relPos);
  }
  //physObject->setDebugForce(force);
}

void PhysicsManager::applyImpulse(
    const int objectID,
    Magnum::Vector3 impulse,
    Magnum::Vector3 relPos) 
{
  std::shared_ptr<physics::RigidObject> physObject = existingObjects_[objectID];
  PhysicalObjectType objectType = existingObjTypes_[objectID];
  if (objectType == physics::PhysicalObjectType::DYNAMIC) {
    physObject->applyImpulse(impulse, relPos);
  }
}

void PhysicsManager::setTransformation(
    const int objectID, 
    const Magnum::Math::Matrix4<float> trans) {
  existingObjects_[objectID]->setTransformation(trans);
}
void PhysicsManager::setTranslation(
    const int objectID, 
    const Magnum::Math::Vector3<float> vector) {
  existingObjects_[objectID]->setTranslation(vector); 
}
void PhysicsManager::setRotation(
    const int objectID, 
    const Magnum::Math::Quaternion<float>& quaternion) {
  existingObjects_[objectID]->setRotation(quaternion);
}
void PhysicsManager::resetTransformation(const int objectID) {
  existingObjects_[objectID]->resetTransformation();
}
void PhysicsManager::translate(
    const int objectID, 
    const Magnum::Math::Vector3<float> vector) {
  existingObjects_[objectID]->translate(vector);
}
void PhysicsManager::translateLocal(
    const int objectID, 
    const Magnum::Math::Vector3<float> vector) {
  existingObjects_[objectID]->translateLocal(vector);
}
void PhysicsManager::rotate(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad,
    const Magnum::Math::Vector3<float> normalizedAxis) {
  existingObjects_[objectID]->rotate(angleInRad, normalizedAxis); 
}
void PhysicsManager::rotateX(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateX(angleInRad); 
}
void PhysicsManager::rotateY(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateY(angleInRad); 
}
void PhysicsManager::rotateXLocal(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateXLocal(angleInRad); 
}
void PhysicsManager::rotateYLocal(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateYLocal(angleInRad); 
}
void PhysicsManager::rotateZ(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateZ(angleInRad); 
}
void PhysicsManager::rotateZLocal(
    const int objectID, 
    const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateZLocal(angleInRad); 
}

}  // namespace physics
}  // namespace esp
