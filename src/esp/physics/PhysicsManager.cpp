// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManager.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/ResourceManager.h"

namespace esp {
namespace physics {

bool PhysicsManager::initPhysics(scene::SceneNode* node,
                                 const assets::PhysicsManagerAttributes&) {
  physicsNode_ = node;
  //! Create new scene node
  sceneNode_ = new physics::RigidObject(physicsNode_);
  initialized_ = true;

  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
}

bool PhysicsManager::addScene(
    const assets::PhysicsSceneAttributes& physicsSceneAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Initialize scene
  bool sceneSuccess =
      sceneNode_->initializeScene(physicsSceneAttributes, meshGroup);
  return sceneSuccess;
}

int PhysicsManager::addObject(const int objectLibIndex,
                              DrawableGroup* drawables) {
  const std::string configFile =
      resourceManager_->getObjectConfig(objectLibIndex);

  //! Test Mesh primitive is valid
  const std::vector<assets::CollisionMeshData>& meshGroup =
      resourceManager_->getCollisionMesh(configFile);
  assets::PhysicsObjectAttributes physicsObjectAttributes =
      resourceManager_->getPhysicsObjectAttributes(configFile);
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return ID_UNDEFINED;
    }
  }

  //! Instantiate with mesh pointer
  int nextObjectID_ = makeRigidObject(meshGroup, physicsObjectAttributes);
  if (nextObjectID_ < 0) {
    LOG(ERROR) << "makeRigidObject unsuccessful";
    return ID_UNDEFINED;
  }

  //! Draw object via resource manager
  //! Render node as child of physics node
  resourceManager_->loadObject(configFile, existingObjects_.at(nextObjectID_),
                               drawables);

  return nextObjectID_;
}

int PhysicsManager::addObject(const std::string& configFile,
                              DrawableGroup* drawables) {
  int resObjectID = resourceManager_->getObjectID(configFile);
  //! Invoke resourceManager to draw object
  int physObjectID = addObject(resObjectID, drawables);
  return physObjectID;
}

int PhysicsManager::removeObject(const int physObjectID) {
  if (existingObjects_.count(physObjectID) == 0) {
    LOG(ERROR) << "Failed to remove object: no object with ID " << physObjectID;
    return ID_UNDEFINED;
  }
  existingObjects_.at(physObjectID)->removeObject();
  delete existingObjects_.at(physObjectID);
  existingObjects_.erase(physObjectID);
  deallocateObjectID(physObjectID);
  return physObjectID;
}

bool PhysicsManager::setObjectMotionType(const int physObjectID,
                                         MotionType mt) {
  if (existingObjects_.count(physObjectID) == 0) {
    return false;
  } else {
    return existingObjects_[physObjectID]->setMotionType(mt);
  }
}

MotionType PhysicsManager::getObjectMotionType(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getMotionType();
  }
  return MotionType::ERROR_MOTIONTYPE;
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
  existingObjects_[newObjectID] = new physics::RigidObject(sceneNode_);

  //! Instantiate with mesh pointer
  bool objectSuccess =
      existingObjects_.at(newObjectID)
          ->initializeObject(physicsObjectAttributes, meshGroup);
  if (!objectSuccess) {
    deallocateObjectID(newObjectID);
    delete existingObjects_.at(newObjectID);  // TODO: check this. Could be
                                              // null?
    existingObjects_.erase(newObjectID);
    return ID_UNDEFINED;
  }
  return newObjectID;
}

//! Base physics manager has no requirement for mesh primitive
bool PhysicsManager::isMeshPrimitiveValid(const assets::CollisionMeshData&) {
  return true;
}

// TODO: this function should do any engine specific setting which is
// necessary to change the timestep
void PhysicsManager::setTimestep(double dt) {
  fixedTimeStep_ = dt;
}

void PhysicsManager::setGravity(const Magnum::Vector3&) {
  // Can't do this for kinematic simulator
}

Magnum::Vector3 PhysicsManager::getGravity() {
  return Magnum::Vector3(0);
}

void PhysicsManager::stepPhysics(double dt) {
  // We don't step uninitialized physics sim...
  if (!initialized_) {
    return;
  }

  // ==== Physics stepforward ======
  // NOTE: simulator step goes here in derived classes...

  if (dt < 0) {
    dt = fixedTimeStep_;
  }

  // handle in-between step times? Ideally dt is a multiple of
  // sceneMetaData_.timestep
  double targetTime = worldTime_ + dt;
  while (worldTime_ < targetTime) {
    // per fixed-step operations can be added here
    worldTime_ += fixedTimeStep_;
  }
}

//! Profile function. In BulletPhysics stationery objects are
//! marked as inactive to speed up simulation. This function
//! helps checking how many objects are active/inactive at any
//! time step
int PhysicsManager::checkActiveObjects() {
  if (sceneNode_ == nullptr) {
    return 0;
  }

  // We don't check uninitialized physics sim...
  if (!initialized_) {
    return 0;
  }

  int numActive = 0;
  int numTotal = 0;
  for (auto& child : sceneNode_->children()) {
    physics::RigidObject* childNode =
        static_cast<physics::RigidObject*>(&child);
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
                                const Magnum::Vector3& force,
                                const Magnum::Vector3& relPos) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyForce(force, relPos);
  }
}

void PhysicsManager::applyImpulse(const int physObjectID,
                                  const Magnum::Vector3& impulse,
                                  const Magnum::Vector3& relPos) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyImpulse(impulse, relPos);
  }
}

void PhysicsManager::applyTorque(const int physObjectID,
                                 const Magnum::Vector3& torque) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyTorque(torque);
  }
}

void PhysicsManager::applyImpulseTorque(const int physObjectID,
                                        const Magnum::Vector3& impulse) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->applyImpulseTorque(impulse);
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

void PhysicsManager::rotateLocal(const int physObjectID,
                                 const Magnum::Rad angleInRad,
                                 const Magnum::Vector3& normalizedAxis) {
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->rotateLocal(angleInRad, normalizedAxis);
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

Magnum::Matrix4 PhysicsManager::getTransformation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->transformation();
  } else {
    return Magnum::Matrix4();
  }
}

Magnum::Vector3 PhysicsManager::getTranslation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->translation();
  } else {
    return Magnum::Vector3();
  }
}

Magnum::Quaternion PhysicsManager::getRotation(const int physObjectID) {
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->rotation();
  } else {
    return Magnum::Quaternion();
  }
}

//============ Object Setter functions =============
void PhysicsManager::setMass(const int physObjectID, const double mass) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setMass(mass);
  }
}
void PhysicsManager::setCOM(const int physObjectID,
                            const Magnum::Vector3& COM) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setCOM(COM);
  }
}
void PhysicsManager::setInertiaVector(const int physObjectID,
                                      const Magnum::Vector3& inertia) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setInertiaVector(inertia);
  }
}
void PhysicsManager::setScale(const int physObjectID, const double scale) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setScale(scale);
  }
}
void PhysicsManager::setFrictionCoefficient(const int physObjectID,
                                            const double frictionCoefficient) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setFrictionCoefficient(frictionCoefficient);
  }
}
void PhysicsManager::setRestitutionCoefficient(
    const int physObjectID,
    const double restitutionCoefficient) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setRestitutionCoefficient(
        restitutionCoefficient);
  }
}
void PhysicsManager::setLinearDamping(const int physObjectID,
                                      const double linDamping) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setLinearDamping(linDamping);
  }
}
void PhysicsManager::setAngularDamping(const int physObjectID,
                                       const double angDamping) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    existingObjects_[physObjectID]->setAngularDamping(angDamping);
  }
}

//============ Object Getter functions =============
double PhysicsManager::getMass(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getMass();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

Magnum::Vector3 PhysicsManager::getCOM(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getCOM();
  } else {
    return Magnum::Vector3();
  }
}

Magnum::Vector3 PhysicsManager::getInertiaVector(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getInertiaVector();
  } else {
    return Magnum::Vector3();
  }
}

Magnum::Matrix3 PhysicsManager::getInertiaMatrix(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getInertiaMatrix();
  } else {
    return Magnum::Matrix3();
  }
}

double PhysicsManager::getScale(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getScale();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

double PhysicsManager::getFrictionCoefficient(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getFrictionCoefficient();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

double PhysicsManager::getRestitutionCoefficient(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getRestitutionCoefficient();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

double PhysicsManager::getLinearDamping(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getLinearDamping();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

double PhysicsManager::getAngularDamping(const int physObjectID) {
  // TODO: talk to property library
  if (existingObjects_.count(physObjectID) > 0) {
    return existingObjects_[physObjectID]->getAngularDamping();
  } else {
    return PHYSICS_ATTR_UNDEFINED;
  }
}

}  // namespace physics
}  // namespace esp
