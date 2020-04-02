// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManager.h"
#include "esp/assets/CollisionMeshData.h"

#include <Magnum/Math/Range.h>

namespace esp {
namespace physics {

bool PhysicsManager::initPhysics(
    scene::SceneNode* node,
    const assets::PhysicsManagerAttributes& physicsManagerAttributes) {
  physicsNode_ = node;

  // Copy over relevant configuration
  fixedTimeStep_ = physicsManagerAttributes.getTimestep();

  //! Create new scene node
  staticSceneObject_ =
      std::make_unique<physics::RigidObject>(&physicsNode_->createChild());
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
      staticSceneObject_->initializeScene(physicsSceneAttributes, meshGroup);
  return sceneSuccess;
}

int PhysicsManager::addObject(const int objectLibIndex,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const Magnum::ResourceKey& lightSetup) {
  //! Test Mesh primitive is valid
  assets::PhysicsObjectAttributes physicsObjectAttributes =
      resourceManager_->getPhysicsObjectAttributes(objectLibIndex);
  const std::vector<assets::CollisionMeshData>& meshGroup =
      resourceManager_->getCollisionMesh(objectLibIndex);

  //! Make rigid object and add it to existingObjects
  int nextObjectID_ = allocateObjectID();
  scene::SceneNode* objectNode = attachmentNode;
  if (attachmentNode == nullptr) {
    objectNode = &staticSceneObject_->node().createChild();
  }

  bool objectSuccess = makeAndAddRigidObject(
      nextObjectID_, meshGroup, physicsObjectAttributes, objectNode);

  if (!objectSuccess) {
    deallocateObjectID(nextObjectID_);
    // existingObjects_.erase(newObjectID);
    if (attachmentNode == nullptr) {
      delete objectNode;
    }
    LOG(ERROR) << "makeRigidObject unsuccessful";
    return ID_UNDEFINED;
  }

  //! Draw object via resource manager
  //! Render node as child of physics node
  resourceManager_->addObjectToDrawables(
      objectLibIndex, existingObjects_.at(nextObjectID_)->visualNode_,
      drawables, lightSetup);
  existingObjects_.at(nextObjectID_)->node().computeCumulativeBB();

  if (physicsObjectAttributes.hasValue("COM_provided")) {
    // if the COM is provided, shift by that
    Magnum::Vector3 comShift = -physicsObjectAttributes.getCOM();
    // first apply scale
    comShift = physicsObjectAttributes.getScale() * comShift;
    existingObjects_.at(nextObjectID_)->shiftOrigin(comShift);
  } else {
    // otherwise use the bounding box center
    existingObjects_.at(nextObjectID_)->shiftOriginToBBCenter();
  }

  // finalize rigid object creation
  existingObjects_.at(nextObjectID_)->finalizeObject();

  return nextObjectID_;
}

int PhysicsManager::addObject(const std::string& configFile,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const Magnum::ResourceKey& lightSetup) {
  int resObjectID = resourceManager_->getObjectTemplateID(configFile);
  //! Invoke resourceManager to draw object
  int physObjectID =
      addObject(resObjectID, drawables, attachmentNode, lightSetup);
  return physObjectID;
}

void PhysicsManager::removeObject(const int physObjectID,
                                  bool deleteObjectNode,
                                  bool deleteVisualNode) {
  assertIDValidity(physObjectID);
  scene::SceneNode* objectNode = &existingObjects_.at(physObjectID)->node();
  scene::SceneNode* visualNode = existingObjects_.at(physObjectID)->visualNode_;
  existingObjects_.erase(physObjectID);
  deallocateObjectID(physObjectID);
  if (deleteObjectNode) {
    delete objectNode;
  } else if (deleteVisualNode && visualNode) {
    delete visualNode;
  }
}

bool PhysicsManager::setObjectMotionType(const int physObjectID,
                                         MotionType mt) {
  assertIDValidity(physObjectID);
  return existingObjects_[physObjectID]->setMotionType(mt);
}

MotionType PhysicsManager::getObjectMotionType(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getMotionType();
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

bool PhysicsManager::makeAndAddRigidObject(
    int newObjectID,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    assets::PhysicsObjectAttributes physicsObjectAttributes,
    scene::SceneNode* objectNode) {
  auto ptr = std::make_unique<physics::RigidObject>(objectNode);
  bool objSuccess = ptr->initializeObject(physicsObjectAttributes, meshGroup);
  if (objSuccess) {
    existingObjects_.emplace(newObjectID, std::move(ptr));
  }
  return objSuccess;
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

Magnum::Vector3 PhysicsManager::getGravity() const {
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

    // kinematic velocity control intergration
    for (auto& object : existingObjects_) {
      VelocityControl& velControl = object.second->getVelocityControl();
      if (velControl.controllingAngVel || velControl.controllingLinVel) {
        scene::SceneNode& objectSceneNode = object.second->node();
        objectSceneNode.setTransformation(velControl.integrateTransform(
            fixedTimeStep_, objectSceneNode.transformation()));
      }
    }
    worldTime_ += fixedTimeStep_;
  }
}

//! Profile function. In BulletPhysics stationery objects are
//! marked as inactive to speed up simulation. This function
//! helps checking how many objects are active/inactive at any
//! time step
int PhysicsManager::checkActiveObjects() {
  if (staticSceneObject_ == nullptr) {
    return 0;
  }

  // We don't check uninitialized physics sim...
  if (!initialized_) {
    return 0;
  }

  int numActive = 0;
  for (auto& itr : existingObjects_) {
    if (itr.second->isActive()) {
      numActive += 1;
    }
  }
  return numActive;
}

bool PhysicsManager::isActive(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->isActive();
}

void PhysicsManager::applyForce(const int physObjectID,
                                const Magnum::Vector3& force,
                                const Magnum::Vector3& relPos) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->applyForce(force, relPos);
}

void PhysicsManager::applyImpulse(const int physObjectID,
                                  const Magnum::Vector3& impulse,
                                  const Magnum::Vector3& relPos) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->applyImpulse(impulse, relPos);
}

void PhysicsManager::applyTorque(const int physObjectID,
                                 const Magnum::Vector3& torque) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->applyTorque(torque);
}

void PhysicsManager::applyImpulseTorque(const int physObjectID,
                                        const Magnum::Vector3& impulse) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->applyImpulseTorque(impulse);
}

void PhysicsManager::setTransformation(const int physObjectID,
                                       const Magnum::Matrix4& trans) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setTransformation(trans);
}
void PhysicsManager::setTranslation(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setTranslation(vector);
}
void PhysicsManager::setRotation(const int physObjectID,
                                 const Magnum::Quaternion& quaternion) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setRotation(quaternion);
}
void PhysicsManager::resetTransformation(const int physObjectID) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->resetTransformation();
}
void PhysicsManager::translate(const int physObjectID,
                               const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->translate(vector);
}
void PhysicsManager::translateLocal(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->translateLocal(vector);
}
void PhysicsManager::rotate(const int physObjectID,
                            const Magnum::Rad angleInRad,
                            const Magnum::Vector3& normalizedAxis) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotate(angleInRad, normalizedAxis);
}

void PhysicsManager::rotateLocal(const int physObjectID,
                                 const Magnum::Rad angleInRad,
                                 const Magnum::Vector3& normalizedAxis) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateLocal(angleInRad, normalizedAxis);
}

void PhysicsManager::rotateX(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateX(angleInRad);
}
void PhysicsManager::rotateY(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateY(angleInRad);
}
void PhysicsManager::rotateXLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateXLocal(angleInRad);
}
void PhysicsManager::rotateYLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateYLocal(angleInRad);
}
void PhysicsManager::rotateZ(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateZ(angleInRad);
}
void PhysicsManager::rotateZLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->rotateZLocal(angleInRad);
}

Magnum::Matrix4 PhysicsManager::getTransformation(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node().transformation();
}

Magnum::Vector3 PhysicsManager::getTranslation(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node().translation();
}

Magnum::Quaternion PhysicsManager::getRotation(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node().rotation();
}

void PhysicsManager::setLinearVelocity(const int physObjectID,
                                       const Magnum::Vector3& linVel) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setLinearVelocity(linVel);
}

void PhysicsManager::setAngularVelocity(const int physObjectID,
                                        const Magnum::Vector3& angVel) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setAngularVelocity(angVel);
}

Magnum::Vector3 PhysicsManager::getLinearVelocity(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getLinearVelocity();
}

Magnum::Vector3 PhysicsManager::getAngularVelocity(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getAngularVelocity();
}

VelocityControl& PhysicsManager::getVelocityControl(const int physObjectID) {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getVelocityControl();
}

//============ Object Setter functions =============
void PhysicsManager::setMass(const int physObjectID, const double mass) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setMass(mass);
}
void PhysicsManager::setCOM(const int physObjectID,
                            const Magnum::Vector3& COM) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setCOM(COM);
}
void PhysicsManager::setInertiaVector(const int physObjectID,
                                      const Magnum::Vector3& inertia) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setInertiaVector(inertia);
}
void PhysicsManager::setScale(const int physObjectID, const double scale) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setScale(scale);
}
void PhysicsManager::setFrictionCoefficient(const int physObjectID,
                                            const double frictionCoefficient) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setFrictionCoefficient(frictionCoefficient);
}
void PhysicsManager::setRestitutionCoefficient(
    const int physObjectID,
    const double restitutionCoefficient) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setRestitutionCoefficient(
      restitutionCoefficient);
}
void PhysicsManager::setLinearDamping(const int physObjectID,
                                      const double linDamping) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setLinearDamping(linDamping);
}
void PhysicsManager::setAngularDamping(const int physObjectID,
                                       const double angDamping) {
  assertIDValidity(physObjectID);
  existingObjects_[physObjectID]->setAngularDamping(angDamping);
}

//============ Object Getter functions =============
double PhysicsManager::getMass(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getMass();
}

Magnum::Vector3 PhysicsManager::getCOM(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getCOM();
}

Magnum::Vector3 PhysicsManager::getInertiaVector(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getInertiaVector();
}

Magnum::Matrix3 PhysicsManager::getInertiaMatrix(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getInertiaMatrix();
}

double PhysicsManager::getScale(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getScale();
}

double PhysicsManager::getFrictionCoefficient(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getFrictionCoefficient();
}

double PhysicsManager::getRestitutionCoefficient(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getRestitutionCoefficient();
}

double PhysicsManager::getLinearDamping(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getLinearDamping();
}

double PhysicsManager::getAngularDamping(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getAngularDamping();
}

void PhysicsManager::setObjectBBDraw(int physObjectID,
                                     DrawableGroup* drawables,
                                     bool drawBB) {
  assertIDValidity(physObjectID);
  if (existingObjects_[physObjectID]->BBNode_ && !drawBB) {
    // destroy the node
    delete existingObjects_[physObjectID]->BBNode_;
    existingObjects_[physObjectID]->BBNode_ = nullptr;
  } else if (drawBB && existingObjects_[physObjectID]->visualNode_) {
    // add a new BBNode
    Magnum::Vector3 scale =
        existingObjects_[physObjectID]->visualNode_->getCumulativeBB().size() /
        2.0;
    existingObjects_[physObjectID]->BBNode_ =
        &existingObjects_[physObjectID]->visualNode_->createChild();
    existingObjects_[physObjectID]->BBNode_->MagnumObject::setScaling(scale);
    existingObjects_[physObjectID]->BBNode_->MagnumObject::setTranslation(
        existingObjects_[physObjectID]
            ->visualNode_->getCumulativeBB()
            .center());
    resourceManager_->addPrimitiveToDrawables(
        0, *existingObjects_[physObjectID]->BBNode_, drawables);
  }
}

const scene::SceneNode& PhysicsManager::getObjectSceneNode(
    int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node();
}

scene::SceneNode& PhysicsManager::getObjectSceneNode(int physObjectID) {
  return const_cast<scene::SceneNode&>(
      const_cast<const PhysicsManager&>(*this).getObjectSceneNode(
          physObjectID));
}

const scene::SceneNode& PhysicsManager::getObjectVisualSceneNode(
    int physObjectID) const {
  assertIDValidity(physObjectID);
  return *existingObjects_.at(physObjectID)->visualNode_;
}

const assets::PhysicsObjectAttributes&
PhysicsManager::getInitializationAttributes(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getInitializationAttributes();
}

}  // namespace physics
}  // namespace esp
