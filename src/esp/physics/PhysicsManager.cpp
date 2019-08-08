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

bool PhysicsManager::initPhysics(
    scene::SceneNode* node,
    assets::PhysicsManagerAttributes physicsManagerAttributes) {
  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = std::make_shared<physics::RigidObject>(physicsNode_);
  initialized_ = true;

  return true;
}

PhysicsManager::~PhysicsManager() {}

bool PhysicsManager::addScene(
    const assets::AssetInfo& info,
    assets::PhysicsSceneAttributes& physicsSceneAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  switch (info.type) {
    case assets::AssetType::INSTANCE_MESH:
      break;  // ._semantic.ply mesh data
    case assets::AssetType::FRL_INSTANCE_MESH:
      break;  // FRL mesh
    default:
      break;  // GLB mesh data
  }

  //! Initialize scene
  bool sceneSuccess =
      sceneNode_->initializeScene(physicsSceneAttributes, meshGroup);

  return sceneSuccess;
}

int PhysicsManager::addObject(const int resObjectID, DrawableGroup* drawables) {
  const std::string configFile = resourceManager_->getObjectConfig(resObjectID);

  //! Test Mesh primitive is valid
  const std::vector<assets::CollisionMeshData>& meshGroup =
      resourceManager_->getCollisionMesh(configFile);
  assets::PhysicsObjectAttributes physicsObjectAttributes =
      resourceManager_->getPhysicsObjectAttributes(configFile);
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Instantiate with mesh pointer
  int nextObjectID_ = makeRigidObject(meshGroup, physicsObjectAttributes);
  if (nextObjectID_ < 0) {
    LOG(ERROR) << "makeRigidObject unsuccessful";
    return -1;
  }

  //! Draw object via resource manager
  //! Render node as child of physics node
  resourceManager_->loadObject(
      configFile, existingObjects_.at(nextObjectID_).get(), drawables);

  return nextObjectID_;
}

int PhysicsManager::addObject(const std::string configFile,
                              DrawableGroup* drawables) {
  int resObjectID = resourceManager_->getObjectID(configFile);
  //! Invoke resourceManager to draw object
  int physObjectID = addObject(resObjectID, drawables);
  return physObjectID;
}

int PhysicsManager::removeObject(const int physObjectID) {
  if (existingObjects_.count(physObjectID) == 0) {
    LOG(ERROR) << "Failed to remove object: no object with ID " << physObjectID;
    return -1;
  }
  existingObjects_.at(physObjectID)->removeObject();
  existingObjects_.erase(physObjectID);
  deallocateObjectID(physObjectID);
  return physObjectID;
}

int PhysicsManager::allocateObjectID() {
  if (!recycledObjectIDs_.empty()) {
    int recycledID = recycledObjectIDs_.back();
    recycledObjectIDs_.pop_back();
    return recycledID;
  }

  return nextObjectID_++;
}

int PhysicsManager::deallocateObjectID(int physObjectID) {
  recycledObjectIDs_.push_back(physObjectID);
  return physObjectID;
}

//! Create and initialize rigid object
int PhysicsManager::makeRigidObject(
    const std::vector<assets::CollisionMeshData>& meshGroup,
    assets::PhysicsObjectAttributes physicsObjectAttributes) {
  //! Create new physics object (child node of sceneNode_)

  int newObjectID = allocateObjectID();
  existingObjects_.emplace(
      newObjectID, std::make_shared<physics::RigidObject>(sceneNode_.get()));

  //! Instantiate with mesh pointer
  bool objectSuccess =
      existingObjects_.at(newObjectID)
          ->initializeObject(physicsObjectAttributes, meshGroup);
  if (!objectSuccess) {
    deallocateObjectID(newObjectID);
    existingObjects_.erase(newObjectID);
    return -1;
  }
  return newObjectID;
}

//! Base physics manager has no requirement for mesh primitive
bool PhysicsManager::isMeshPrimitiveValid(
    const assets::CollisionMeshData& meshData) {
  return true;
}

// ALEX TODO: this function should do any engine specific setting which is
// necessary to change the timestep
void PhysicsManager::setTimestep(double dt) {
  fixedTimeStep_ = dt;
}

void PhysicsManager::setGravity(const Magnum::Vector3& gravity) {
  // Can't do this for kinematic simulator
}

const Magnum::Vector3 PhysicsManager::getGravity() {
  return Magnum::Vector3(0);
}

void PhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_)
    return;

  // ==== Physics stepforward ======
  // NOTE: simulator step goes here in derived classes...

  if (dt < 0)
    dt = fixedTimeStep_;

  // handle in-between step times? Ideally dt is a multiple of
  // sceneMetaData_.timestep
  double targetTime = worldTime_ + dt;
  while (worldTime_ < targetTime)  // per fixed-step operations can be added
                                   // here
    worldTime_ += fixedTimeStep_;
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

void PhysicsManager::applyForce(const int physObjectID,
                                Magnum::Vector3& force,
                                Magnum::Vector3& relPos) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyForce(force, relPos);
  }
}

void PhysicsManager::applyImpulse(const int physObjectID,
                                  Magnum::Vector3& impulse,
                                  Magnum::Vector3& relPos) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyImpulse(impulse, relPos);
  }
}

void PhysicsManager::setTransformation(const int physObjectID,
                                       const Magnum::Matrix4& trans) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setTransformation(trans);
  }
}
void PhysicsManager::setTranslation(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setTranslation(vector);
  }
}
void PhysicsManager::setRotation(const int physObjectID,
                                 const Magnum::Quaternion& quaternion) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setRotation(quaternion);
  }
}
void PhysicsManager::resetTransformation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->resetTransformation();
  }
}
void PhysicsManager::translate(const int physObjectID,
                               const Magnum::Vector3& vector) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->translate(vector);
  }
}
void PhysicsManager::translateLocal(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->translateLocal(vector);
  }
}
void PhysicsManager::rotate(const int physObjectID,
                            const Magnum::Rad angleInRad,
                            const Magnum::Vector3& normalizedAxis) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotate(angleInRad, normalizedAxis);
  }
}
void PhysicsManager::rotateX(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateX(angleInRad);
  }
}
void PhysicsManager::rotateY(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateY(angleInRad);
  }
}
void PhysicsManager::rotateXLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateXLocal(angleInRad);
  }
}
void PhysicsManager::rotateYLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateYLocal(angleInRad);
  }
}
void PhysicsManager::rotateZ(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateZ(angleInRad);
  }
}
void PhysicsManager::rotateZLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateZLocal(angleInRad);
  }
}

const Magnum::Matrix4 PhysicsManager::getTransformation(
    const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->transformation();
  } else {
    return Magnum::Matrix4();
  }
}

const Magnum::Vector3 PhysicsManager::getTranslation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->translation();
  } else {
    return Magnum::Vector3();
  }
}

const Magnum::Quaternion PhysicsManager::getRotation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->rotation();
  } else {
    return Magnum::Quaternion();
  }
}

//============ Object Setter functions =============
void PhysicsManager::setMass(const int physObjectID, const double mass) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setMass(mass);
  }
}
void PhysicsManager::setCOM(const int physObjectID,
                            const Magnum::Vector3& COM) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setCOM(COM);
  }
}
void PhysicsManager::setInertia(const int physObjectID,
                                const Magnum::Vector3& inertia) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setInertia(inertia);
  }
}
void PhysicsManager::setScale(const int physObjectID, const double scale) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setScale(scale);
  }
}
void PhysicsManager::setFrictionCoefficient(const int physObjectID,
                                            const double frictionCoefficient) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setFrictionCoefficient(frictionCoefficient);
  }
}
void PhysicsManager::setRestitutionCoefficient(
    const int physObjectID,
    const double restitutionCoefficient) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setRestitutionCoefficient(
        restitutionCoefficient);
  }
}
void PhysicsManager::setLinearDamping(const int physObjectID,
                                      const double linDamping) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setLinearDamping(linDamping);
  }
}
void PhysicsManager::setAngularDamping(const int physObjectID,
                                       const double angDamping) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setAngularDamping(angDamping);
  }
}

//============ Object Getter functions =============
const double PhysicsManager::getMass(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getMass();
  } else {
    return -1.0;
  }
}

const Magnum::Vector3 PhysicsManager::getCOM(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getCOM();
  } else {
    return Magnum::Vector3();
  }
}
const Magnum::Vector3 PhysicsManager::getInertia(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getInertia();
  } else {
    return Magnum::Vector3();
  }
}
const double PhysicsManager::getScale(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getScale();
  } else {
    return -1.0;
  }
}
const double PhysicsManager::getFrictionCoefficient(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getFrictionCoefficient();
  } else {
    return -1.0;
  }
}
const double PhysicsManager::getRestitutionCoefficient(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getRestitutionCoefficient();
  } else {
    return -1.0;
  }
}
const double PhysicsManager::getLinearDamping(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getLinearDamping();
  } else {
    return -1.0;
  }
}
const double PhysicsManager::getAngularDamping(const int physObjectID) {
  // (JH Note) TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getAngularDamping();
  } else {
    return -1.0;
  }
}

}  // namespace physics
}  // namespace esp
