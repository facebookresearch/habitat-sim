// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <chrono>
#include <ctime>
#include <functional>

#include <Corrade/PluginManager/Manager.h>
#include <Corrade/Utility/String.h>
#include <Magnum/Math/Color.h>
#include <Magnum/PixelFormat.h>
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

#include "PhysicsManager.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/GltfMeshData.h"
#include "esp/assets/Mp3dInstanceMeshData.h"
#include "esp/assets/PTexMeshData.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

bool PhysicsManager::initPhysics(scene::SceneNode* node,
                                 Magnum::Vector3d gravity,
                                 bool do_profile) {
  LOG(INFO) << "Initializing Base Physics Engine...";

  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = std::make_shared<physics::RigidObject>(physicsNode_);
  initialized_ = true;
  do_profile_ = do_profile;

  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
}

bool PhysicsManager::addScene(
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
  bool sceneSuccess = sceneNode_->initializeScene(meshGroup);
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}

int PhysicsManager::addObject(const int objectID, DrawableGroup* drawables) {
  std::string configFile = resourceManager->getObjectConfig(objectID);
  return addObject(configFile, drawables);
}

int PhysicsManager::addObject(const std::string configFile,
                              DrawableGroup* drawables) {
  std::vector<assets::CollisionMeshData> meshGroup =
      resourceManager->getCollisionMesh(configFile);

  assets::PhysicsObjectMetaData metaData =
      resourceManager->getPhysicsMetaData(configFile);

  LOG(INFO) << "Add object: before check";
  //! Test Mesh primitive is valid
  for (assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }
  LOG(INFO) << "Add object: before child node";

  //! Create new physics object (child node of sceneNode_)
  std::shared_ptr<physics::RigidObject> physObject =
      std::make_shared<physics::RigidObject>(sceneNode_.get());
  objectNodes_.emplace_back(physObject);

  LOG(INFO) << "Add object: before initialize";

  //! Instantiate with mesh pointer
  bool objectSuccess = physObject->initializeObject(metaData, meshGroup);
  if (!objectSuccess) {
    LOG(ERROR) << "Initialize unsuccessful";
    return -1;
  }

  LOG(INFO) << "Add object: before render stack";

  //! Maintain object resource
  existingObjects_[nextObjectID_] = physObject;
  existingObjNames_[nextObjectID_] = configFile;
  //! Increment simple object ID tracker
  nextObjectID_ += 1;

  //! IMPORTANT: invoke resourceManager to draw object
  int resObjectID =
      resourceManager->addObject(configFile, physObject.get(), drawables);
  if (resObjectID < 0) {
    return -1;
  }

  LOG(INFO) << "Add object: after render stack";
  return nextObjectID_ - 1;
}

//! Check if mesh primitive is compatible with physics
bool PhysicsManager::isMeshPrimitiveValid(assets::CollisionMeshData& meshData) {
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

// ALEX TODO: this function should do any engine specific setting which is
// necessary to change the timestep
void PhysicsManager::setTimestep(double dt) {
  fixedTimeStep_ = dt;
}

void PhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_)
    return;

  // ==== Physics stepforward ======
  auto start = std::chrono::system_clock::now();
  // Alex NOTE: simulator step goes here in derived classes...
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

  if (dt < 0)
    dt = fixedTimeStep_;

  // Alex TODO: handle in-between step times? Ideally dt is a multiple of
  // fixedTimeStep_
  double targetTime = worldTime_ + dt;
  while (worldTime_ < targetTime)
    worldTime_ += fixedTimeStep_;

  // Alex NOTE: removed numObjects count from Bullet...
}

//! Profile function. In BulletPhysics stationery objects are
//! marked as inactive to speed up simulation. This function
//! helps checking how many objects are active/inactive at any
//! time step
int PhysicsManager::checkActiveObjects() {
  if (sceneNode_.get() == nullptr) {
    return 0;
  }

  // We don't check uninitialized physics sim...
  if (!initialized_)
    return 0;

  int numActive = 0;
  int numTotal = 0;
  for (auto& child : sceneNode_->children()) {
    physics::RigidObject* childNode =
        dynamic_cast<physics::RigidObject*>(&child);
    if (childNode != nullptr) {
      numTotal += 1;
      if (childNode->isActive()) {
        numActive += 1;
      }
    }
  }
  return numActive;
}

void PhysicsManager::applyForce(const int objectID,
                                Magnum::Vector3 force,
                                Magnum::Vector3 relPos) {
  std::shared_ptr<physics::RigidObject> physObject = existingObjects_[objectID];
  physObject->applyForce(force, relPos);
  // physObject->setDebugForce(force);
}

void PhysicsManager::applyImpulse(const int objectID,
                                  Magnum::Vector3 impulse,
                                  Magnum::Vector3 relPos) {
  std::shared_ptr<physics::RigidObject> physObject = existingObjects_[objectID];
  physObject->applyImpulse(impulse, relPos);
}

void PhysicsManager::setTransformation(
    const int objectID,
    const Magnum::Math::Matrix4<float> trans) {
  existingObjects_[objectID]->setTransformation(trans);
}
void PhysicsManager::setTranslation(const int objectID,
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
void PhysicsManager::translate(const int objectID,
                               const Magnum::Math::Vector3<float> vector) {
  existingObjects_[objectID]->translate(vector);
}
void PhysicsManager::translateLocal(const int objectID,
                                    const Magnum::Math::Vector3<float> vector) {
  existingObjects_[objectID]->translateLocal(vector);
}
void PhysicsManager::rotate(const int objectID,
                            const Magnum::Math::Rad<float> angleInRad,
                            const Magnum::Math::Vector3<float> normalizedAxis) {
  existingObjects_[objectID]->rotate(angleInRad, normalizedAxis);
}
void PhysicsManager::rotateX(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateX(angleInRad);
}
void PhysicsManager::rotateY(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateY(angleInRad);
}
void PhysicsManager::rotateXLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateXLocal(angleInRad);
}
void PhysicsManager::rotateYLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateYLocal(angleInRad);
}
void PhysicsManager::rotateZ(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateZ(angleInRad);
}
void PhysicsManager::rotateZLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  existingObjects_[objectID]->rotateZLocal(angleInRad);
}

}  // namespace physics
}  // namespace esp
