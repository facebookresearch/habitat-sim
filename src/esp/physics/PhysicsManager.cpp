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
                                 assets::PhysicsSceneMetaData sceneMetaData,
                                 bool do_profile) {
  LOG(INFO) << "Initializing Base Physics Engine...";

  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = std::make_shared<physics::RigidObject>(physicsNode_);
  initialized_ = true;
  do_profile_ = do_profile;

  sceneMetaData_ = sceneMetaData;
  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
}

bool PhysicsManager::addScene(
    const assets::AssetInfo& info,
    assets::PhysicsSceneMetaData& sceneMetaData,
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
  bool sceneSuccess = sceneNode_->initializeScene(sceneMetaData, meshGroup);
  LOG(INFO) << "Init scene done";

  return sceneSuccess;
}

int PhysicsManager::addObject(const int resObjectID, DrawableGroup* drawables) {
  const std::string configFile = resourceManager->getObjectConfig(resObjectID);

  //! Test Mesh primitive is valid
  std::vector<assets::CollisionMeshData> meshGroup =
      resourceManager->getCollisionMesh(configFile);
  assets::PhysicsObjectMetaData metaData =
      resourceManager->getPhysicsMetaData(configFile);
  for (assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Instantiate with mesh pointer
  const int nextObjectID_ = makeRigidObject(meshGroup, metaData);
  if (nextObjectID_ < 0) {
    LOG(ERROR) << "Initialize unsuccessful";
    return -1;
  }
  //! Maintain object resource
  existingObjNames_[nextObjectID_] = configFile;

  //! Draw object via resource manager
  //! Render node as child of physics node
  resourceManager->loadObject(configFile, existingObjects_[nextObjectID_].get(),
                              drawables);

  return nextObjectID_;
}

int PhysicsManager::addObject(const std::string configFile,
                              DrawableGroup* drawables) {
  int resObjectID = resourceManager->getObjectID(configFile);
  //! Invoke resourceManager to draw object
  int physObjectID = addObject(resObjectID, drawables);
  return physObjectID;
}

int PhysicsManager::removeObject(const int physObjectID) {
  LOG(ERROR) << "Removing object " << physObjectID;
  if (physObjectID < 0 || physObjectID >= existingObjects_.size() ||
      existingObjects_[physObjectID] == nullptr) {
    LOG(ERROR) << "Failed to remove object " << physObjectID;
    return -1;
  }
  // LOG(INFO) physObject;
  existingObjects_[physObjectID]->removeObject();
  existingObjects_[physObjectID] = nullptr;
  return physObjectID;
}

//! Create and initialize rigid object
const int PhysicsManager::makeRigidObject(
    std::vector<assets::CollisionMeshData> meshGroup,
    assets::PhysicsObjectMetaData metaData) {
  //! Create new physics object (child node of sceneNode_)
  existingObjects_.emplace_back(
      std::make_unique<physics::RigidObject>(sceneNode_.get()));

  const int nextObjectID_ = existingObjects_.size();
  //! Instantiate with mesh pointer
  bool objectSuccess =
      existingObjects_.back()->initializeObject(metaData, meshGroup);
  if (!objectSuccess) {
    return -1;
  }
  return nextObjectID_ - 1;
}

//! Base physics manager has no requirement for mesh primitive
bool PhysicsManager::isMeshPrimitiveValid(assets::CollisionMeshData& meshData) {
  return true;
}

// ALEX TODO: this function should do any engine specific setting which is
// necessary to change the timestep
void PhysicsManager::setTimestep(double dt) {
  sceneMetaData_.timestep = dt;
}

void PhysicsManager::setGravity(const Magnum::Vector3d gravity) {
  sceneMetaData_.gravity = gravity;
}

const Magnum::Vector3d PhysicsManager::getGravity() {
  return sceneMetaData_.gravity;
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
    dt = sceneMetaData_.timestep;

  // Alex TODO: handle in-between step times? Ideally dt is a multiple of
  // sceneMetaData_.timestep
  double targetTime = worldTime_ + dt;
  while (worldTime_ < targetTime)
    worldTime_ += sceneMetaData_.timestep;

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
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->applyForce(force, relPos);
  }
  // physObject->setDebugForce(force);
}

void PhysicsManager::applyImpulse(const int objectID,
                                  Magnum::Vector3 impulse,
                                  Magnum::Vector3 relPos) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->applyImpulse(impulse, relPos);
  }
}

void PhysicsManager::setTransformation(
    const int objectID,
    const Magnum::Math::Matrix4<float> trans) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->setTransformation(trans);
  }
}
void PhysicsManager::setTranslation(const int objectID,
                                    const Magnum::Math::Vector3<float> vector) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->setTranslation(vector);
  }
}
void PhysicsManager::setRotation(
    const int objectID,
    const Magnum::Math::Quaternion<float>& quaternion) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->setRotation(quaternion);
  }
}
void PhysicsManager::resetTransformation(const int objectID) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->resetTransformation();
  }
}
void PhysicsManager::translate(const int objectID,
                               const Magnum::Math::Vector3<float> vector) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->translate(vector);
  }
}
void PhysicsManager::translateLocal(const int objectID,
                                    const Magnum::Math::Vector3<float> vector) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->translateLocal(vector);
  }
}
void PhysicsManager::rotate(const int objectID,
                            const Magnum::Math::Rad<float> angleInRad,
                            const Magnum::Math::Vector3<float> normalizedAxis) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotate(angleInRad, normalizedAxis);
  }
}
void PhysicsManager::rotateX(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateX(angleInRad);
  }
}
void PhysicsManager::rotateY(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateY(angleInRad);
  }
}
void PhysicsManager::rotateXLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateXLocal(angleInRad);
  }
}
void PhysicsManager::rotateYLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateYLocal(angleInRad);
  }
}
void PhysicsManager::rotateZ(const int objectID,
                             const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateZ(angleInRad);
  }
}
void PhysicsManager::rotateZLocal(const int objectID,
                                  const Magnum::Math::Rad<float> angleInRad) {
  if (existingObjects_[objectID] != nullptr) {
    existingObjects_[objectID]->rotateZLocal(angleInRad);
  }
}

}  // namespace physics
}  // namespace esp
