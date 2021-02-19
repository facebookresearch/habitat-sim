// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManager.h"
#include "esp/assets/CollisionMeshData.h"

#include <Magnum/Math/Range.h>

namespace esp {
namespace physics {

bool PhysicsManager::initPhysics(scene::SceneNode* node) {
  physicsNode_ = node;

  // Copy over relevant configuration
  fixedTimeStep_ = physicsManagerAttributes_->getTimestep();

  //! Create new scene node and set up any physics-related variables
  // Overridden by specific physics-library-based class
  initialized_ = initPhysicsFinalize();
  return initialized_;
}

bool PhysicsManager::initPhysicsFinalize() {
  //! Create new scene node
  staticStageObject_ = physics::RigidStage::create_unique(
      &physicsNode_->createChild(), resourceManager_);
  return true;
}

PhysicsManager::~PhysicsManager() {
  LOG(INFO) << "Deconstructing PhysicsManager";
}

bool PhysicsManager::addStage(
    const std::string& handle,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Initialize scene
  bool sceneSuccess = addStageFinalize(handle);
  return sceneSuccess;
}

bool PhysicsManager::addStageFinalize(const std::string& handle) {
  //! Initialize scene
  bool sceneSuccess = staticStageObject_->initialize(handle);
  return sceneSuccess;
}

int PhysicsManager::addObject(const int objectLibId,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  const std::string& configHandle =
      resourceManager_.getObjectAttributesManager()->getObjectHandleByID(
          objectLibId);

  return addObject(configHandle, drawables, attachmentNode, lightSetup);
}

int PhysicsManager::addObject(const std::string& configFileHandle,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  //! Make rigid object and add it to existingObjects
  int nextObjectID_ = allocateObjectID();
  scene::SceneNode* objectNode = attachmentNode;
  if (attachmentNode == nullptr) {
    objectNode = &staticStageObject_->node().createChild();
  }
  // verify whether necessary assets exist, and if not, instantiate them
  // only make object if asset instantiation succeeds (short circuit)
  bool objectSuccess =
      resourceManager_.instantiateAssetsOnDemand(configFileHandle);
  if (!objectSuccess) {
    LOG(ERROR) << "PhysicsManager::addObject : "
                  "ResourceManager::instantiateAssetsOnDemand unsuccessful. "
                  "Aborting.";
    return ID_UNDEFINED;
  }

  objectSuccess =
      makeAndAddRigidObject(nextObjectID_, configFileHandle, objectNode);

  if (!objectSuccess) {
    deallocateObjectID(nextObjectID_);
    if (attachmentNode == nullptr) {
      delete objectNode;
    }
    LOG(ERROR) << "PhysicsManager::addObject : PhysicsManager::makeRigidObject "
                  "unsuccessful.  Aborting.";
    return ID_UNDEFINED;
  }

  // temp non-owning pointer to object
  esp::physics::RigidObject* const obj =
      (existingObjects_.at(nextObjectID_).get());

  obj->visualNodes_.push_back(obj->visualNode_);

  //! Draw object via resource manager
  //! Render node as child of physics node
  //! Verify we should make the object drawable
  if (obj->getInitializationAttributes()->getIsVisible()) {
    resourceManager_.addObjectToDrawables(obj->getInitializationAttributes(),
                                          obj->visualNode_, drawables,
                                          obj->visualNodes_, lightSetup);
  }

  // finalize rigid object creation
  objectSuccess = obj->finalizeObject();
  if (!objectSuccess) {
    removeObject(nextObjectID_, true, true);
    LOG(ERROR) << "PhysicsManager::addObject : PhysicsManager::finalizeObject "
                  "unsuccessful.  Aborting.";
    return ID_UNDEFINED;
  }

  return nextObjectID_;
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

void PhysicsManager::removeArticulatedObject(int physObjectID) {
  CHECK(existingArticulatedObjects_.count(physObjectID));
  scene::SceneNode* objectNode =
      &existingArticulatedObjects_.at(physObjectID)->node();
  for (auto linkObjId :
       existingArticulatedObjects_.at(physObjectID)->objectIdToLinkId_) {
    deallocateObjectID(linkObjId.first);
  }
  existingArticulatedObjects_.erase(physObjectID);
  deallocateObjectID(physObjectID);
  delete objectNode;
}

bool PhysicsManager::setObjectMotionType(const int physObjectID,
                                         MotionType mt) {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->setMotionType(mt);
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

bool PhysicsManager::makeAndAddRigidObject(int newObjectID,
                                           const std::string& handle,
                                           scene::SceneNode* objectNode) {
  auto ptr = physics::RigidObject::create_unique(objectNode, newObjectID,
                                                 resourceManager_);
  bool objSuccess = ptr->initialize(handle);
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
      VelocityControl::ptr velControl = object.second->getVelocityControl();
      if (velControl->controllingAngVel || velControl->controllingLinVel) {
        object.second->setRigidState(velControl->integrateTransform(
            fixedTimeStep_, object.second->getRigidState()));
      }
    }
    worldTime_ += fixedTimeStep_;
  }
}
void PhysicsManager::deferNodesUpdate() {
  for (auto& o : existingObjects_)
    o.second->deferUpdate();
}

void PhysicsManager::updateNodes() {
  for (auto& o : existingObjects_)
    o.second->updateNodes();

  for (auto& ao : existingArticulatedObjects_)
    ao.second->updateNodes();
}

//! Profile function. In BulletPhysics stationary objects are
//! marked as inactive to speed up simulation. This function
//! helps checking how many objects are active/inactive at any
//! time step
int PhysicsManager::checkActiveObjects() {
  if (staticStageObject_ == nullptr) {
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

bool PhysicsManager::isObjectAwake(const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->isActive();
}

void PhysicsManager::setObjectSleep(const int physObjectID, bool sleep) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setSleep(sleep);
}

void PhysicsManager::applyForce(const int physObjectID,
                                const Magnum::Vector3& force,
                                const Magnum::Vector3& relPos) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->applyForce(force, relPos);
}

void PhysicsManager::applyImpulse(const int physObjectID,
                                  const Magnum::Vector3& impulse,
                                  const Magnum::Vector3& relPos) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->applyImpulse(impulse, relPos);
}

void PhysicsManager::applyTorque(const int physObjectID,
                                 const Magnum::Vector3& torque) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->applyTorque(torque);
}

void PhysicsManager::applyImpulseTorque(const int physObjectID,
                                        const Magnum::Vector3& impulse) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->applyImpulseTorque(impulse);
}

void PhysicsManager::setTransformation(const int physObjectID,
                                       const Magnum::Matrix4& trans) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setTransformation(trans);
}
void PhysicsManager::setRigidState(const int physObjectID,
                                   const esp::core::RigidState& rigidState) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setRigidState(rigidState);
}
void PhysicsManager::setTranslation(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setTranslation(vector);
}
void PhysicsManager::setRotation(const int physObjectID,
                                 const Magnum::Quaternion& quaternion) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setRotation(quaternion);
}
void PhysicsManager::resetTransformation(const int physObjectID) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->resetTransformation();
}
void PhysicsManager::translate(const int physObjectID,
                               const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->translate(vector);
}
void PhysicsManager::translateLocal(const int physObjectID,
                                    const Magnum::Vector3& vector) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->translateLocal(vector);
}
void PhysicsManager::rotate(const int physObjectID,
                            const Magnum::Rad angleInRad,
                            const Magnum::Vector3& normalizedAxis) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotate(angleInRad, normalizedAxis);
}

void PhysicsManager::rotateLocal(const int physObjectID,
                                 const Magnum::Rad angleInRad,
                                 const Magnum::Vector3& normalizedAxis) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateLocal(angleInRad, normalizedAxis);
}

void PhysicsManager::rotateX(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateX(angleInRad);
}
void PhysicsManager::rotateY(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateY(angleInRad);
}
void PhysicsManager::rotateXLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateXLocal(angleInRad);
}
void PhysicsManager::rotateYLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateYLocal(angleInRad);
}
void PhysicsManager::rotateZ(const int physObjectID,
                             const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateZ(angleInRad);
}
void PhysicsManager::rotateZLocal(const int physObjectID,
                                  const Magnum::Rad angleInRad) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->rotateZLocal(angleInRad);
}

Magnum::Matrix4 PhysicsManager::getTransformation(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node().transformation();
}

esp::core::RigidState PhysicsManager::getRigidState(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getRigidState();
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

VelocityControl::ptr PhysicsManager::getVelocityControl(
    const int physObjectID) {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getVelocityControl();
}

//============ Object Setter functions =============
void PhysicsManager::setMass(const int physObjectID, const double mass) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setMass(mass);
}
void PhysicsManager::setCOM(const int physObjectID,
                            const Magnum::Vector3& COM) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setCOM(COM);
}
void PhysicsManager::setInertiaVector(const int physObjectID,
                                      const Magnum::Vector3& inertia) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setInertiaVector(inertia);
}
void PhysicsManager::setFrictionCoefficient(const int physObjectID,
                                            const double frictionCoefficient) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)
      ->setFrictionCoefficient(frictionCoefficient);
}
void PhysicsManager::setRestitutionCoefficient(
    const int physObjectID,
    const double restitutionCoefficient) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)
      ->setRestitutionCoefficient(restitutionCoefficient);
}
void PhysicsManager::setLinearDamping(const int physObjectID,
                                      const double linDamping) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setLinearDamping(linDamping);
}
void PhysicsManager::setAngularDamping(const int physObjectID,
                                       const double angDamping) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setAngularDamping(angDamping);
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

Magnum::Vector3 PhysicsManager::getScale(const int physObjectID) const {
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
  if (existingObjects_.at(physObjectID)->BBNode_ && !drawBB) {
    // destroy the node
    delete existingObjects_.at(physObjectID)->BBNode_;
    existingObjects_.at(physObjectID)->BBNode_ = nullptr;
  } else if (drawBB && existingObjects_.at(physObjectID)->visualNode_) {
    // add a new BBNode
    Magnum::Vector3 scale = existingObjects_.at(physObjectID)
                                ->visualNode_->getCumulativeBB()
                                .size() /
                            2.0;
    existingObjects_.at(physObjectID)->BBNode_ =
        &existingObjects_.at(physObjectID)->visualNode_->createChild();
    existingObjects_.at(physObjectID)->BBNode_->MagnumObject::setScaling(scale);
    existingObjects_.at(physObjectID)
        ->BBNode_->MagnumObject::setTranslation(
            existingObjects_[physObjectID]
                ->visualNode_->getCumulativeBB()
                .center());
    resourceManager_.addPrimitiveToDrawables(
        0, *existingObjects_.at(physObjectID)->BBNode_, drawables);
  }
}

const scene::SceneNode& PhysicsManager::getObjectSceneNode(
    int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->node();
}

scene::SceneNode& PhysicsManager::getObjectSceneNode(int physObjectID) {
  assertIDValidity(physObjectID);
  return const_cast<scene::SceneNode&>(
      const_cast<const PhysicsManager&>(*this).getObjectSceneNode(
          physObjectID));
}

const scene::SceneNode& PhysicsManager::getObjectVisualSceneNode(
    int physObjectID) const {
  assertIDValidity(physObjectID);
  return *existingObjects_.at(physObjectID)->visualNode_;
}

std::vector<scene::SceneNode*> PhysicsManager::getObjectVisualSceneNodes(
    const int physObjectID) const {
  assertIDValidity(physObjectID);
  return existingObjects_.at(physObjectID)->visualNodes_;
}

void PhysicsManager::setSemanticId(const int physObjectID,
                                   uint32_t semanticId) {
  assertIDValidity(physObjectID);
  existingObjects_.at(physObjectID)->setSemanticId(semanticId);
}

}  // namespace physics
}  // namespace esp
