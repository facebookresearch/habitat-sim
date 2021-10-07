// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManager.h"
#include <Magnum/Math/Range.h>
#include "esp/assets/CollisionMeshData.h"
#include "esp/physics/objectManagers/ArticulatedObjectManager.h"
#include "esp/physics/objectManagers/RigidObjectManager.h"
#include "esp/sim/Simulator.h"
namespace esp {
namespace physics {

PhysicsManager::PhysicsManager(
    assets::ResourceManager& _resourceManager,
    const metadata::attributes::PhysicsManagerAttributes::cptr&
        _physicsManagerAttributes)
    : resourceManager_(_resourceManager),
      physicsManagerAttributes_(_physicsManagerAttributes),
      rigidObjectManager_(RigidObjectManager::create()),
      articulatedObjectManager_(ArticulatedObjectManager::create()) {}

bool PhysicsManager::initPhysics(scene::SceneNode* node) {
  physicsNode_ = node;
  // set the rigidObjectManager's weak reference to physics manager to be based
  // on the same shared pointer that Simulator is using.
  rigidObjectManager_->setPhysicsManager(shared_from_this());

  // set articulated object manager here, and in
  articulatedObjectManager_->setPhysicsManager(shared_from_this());

  // Copy over relevant configuration
  fixedTimeStep_ = physicsManagerAttributes_->getTimestep();

  //! Create new scene node and set up any physics-related variables
  // Overridden by specific physics-library-based class
  initialized_ = initPhysicsFinalize();
  return initialized_;
}

bool PhysicsManager::initPhysicsFinalize() {
  //! Create new scene node
  staticStageObject_ = physics::RigidStage::create(&physicsNode_->createChild(),
                                                   resourceManager_);

  return true;
}

PhysicsManager::~PhysicsManager() {
  ESP_DEBUG() << "Deconstructing PhysicsManager";
}

bool PhysicsManager::addStage(
    const metadata::attributes::StageAttributes::ptr& initAttributes,
    const metadata::attributes::SceneObjectInstanceAttributes::cptr&
        stageInstanceAttributes,
    const std::vector<assets::CollisionMeshData>& meshGroup) {
  // Test Mesh primitive is valid
  for (const assets::CollisionMeshData& meshData : meshGroup) {
    if (!isMeshPrimitiveValid(meshData)) {
      return false;
    }
  }

  //! Initialize stage
  bool sceneSuccess = addStageFinalize(initAttributes);
  // add/merge stageInstanceAttributes' copy of user_attributes.
  if (!stageInstanceAttributes) {
    ESP_DEBUG() << "Stage built from StageInstanceAttributes";
    // TODO merge instance attributes into staticStageObject_'s existing
    // attributes
  }
  // TODO process any stage transformations here from stageInstanceAttributes

  return sceneSuccess;
}  // PhysicsManager::addStage

bool PhysicsManager::addStageFinalize(
    const metadata::attributes::StageAttributes::ptr& initAttributes) {
  //! Initialize stage
  bool stageSuccess = staticStageObject_->initialize(initAttributes);
  return stageSuccess;
}  // PhysicsManager::addStageFinalize

int PhysicsManager::addObjectInstance(
    const esp::metadata::attributes::SceneObjectInstanceAttributes::cptr&
        objInstAttributes,
    const std::string& attributesHandle,
    bool defaultCOMCorrection,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  // Get ObjectAttributes
  auto objAttributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByHandle(
          attributesHandle);
  // check if an object is being set to be not visible for a particular
  // instance.
  int visSet = objInstAttributes->getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // specfied in scene instance
    objAttributes->setIsVisible(visSet == 1);
  }

  if (!objAttributes) {
    ESP_ERROR() << "Missing/improperly configured objectAttributes"
                << attributesHandle << ", whose handle contains"
                << objInstAttributes->getHandle()
                << "as specified in object instance attributes.";
    return 0;
  }
  // set shader type to use for stage
  const auto objShaderType = objInstAttributes->getShaderType();
  if (objShaderType !=
      metadata::attributes::ObjectInstanceShaderType::Unknown) {
    objAttributes->setShaderType(getShaderTypeName(objShaderType));
  }
  int objID = 0;
  if (simulator_ != nullptr) {
    auto& drawables = simulator_->getDrawableGroup();
    objID = addObject(objAttributes, &drawables, attachmentNode, lightSetup);
  } else {
    // support creation when simulator DNE
    objID = addObject(objAttributes, nullptr, attachmentNode, lightSetup);
  }

  if (objID == ID_UNDEFINED) {
    // instancing failed for some reason.
    ESP_ERROR() << "Object create failed for objectAttributes"
                << attributesHandle << ", whose handle contains"
                << objInstAttributes->getHandle()
                << "as specified in object instance attributes.";
    return ID_UNDEFINED;
  }
  auto objPtr = this->existingObjects_.at(objID);

  // save the scene init attributes used to configure object's initial state
  objPtr->setSceneInstanceAttr(objInstAttributes);
  // merge scene instance user-defined configurations with the new object's, if
  // scene instance specifies any set articulated object's user-defined
  // attributes, if any exist in scene
  // instance.
  objPtr->mergeUserAttributes(objInstAttributes->getUserConfiguration());

  // set object's location, rotation and other pertinent state values based on
  // scene object instance attributes set in the object above.
  objPtr->resetStateFromSceneInstanceAttr(defaultCOMCorrection);

  return objID;
}  // PhysicsManager::addObjectInstance

int PhysicsManager::addObject(const std::string& attributesHandle,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  esp::metadata::attributes::ObjectAttributes::ptr attributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByHandle(
          attributesHandle);
  if (!attributes) {
    ESP_ERROR() << "Object creation failed due to unknown attributes"
                << attributesHandle;
    return ID_UNDEFINED;
  } else {
    // attributes exist, get drawables if valid simulator accessible
    if (simulator_ != nullptr) {
      auto& drawables = simulator_->getDrawableGroup();
      return addObject(attributes, &drawables, attachmentNode, lightSetup);
    } else {
      // support creation when simulator DNE
      return addObject(attributes, nullptr, attachmentNode, lightSetup);
    }
  }
}  // PhysicsManager::addObject

int PhysicsManager::addObject(const int attributesID,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  const esp::metadata::attributes::ObjectAttributes::ptr attributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByID(
          attributesID);
  if (!attributes) {
    ESP_ERROR() << "Object creation failed due to unknown attributes ID"
                << attributesID;
    return ID_UNDEFINED;
  } else {
    // attributes exist, get drawables if valid simulator accessible
    if (simulator_ != nullptr) {
      auto& drawables = simulator_->getDrawableGroup();
      return addObject(attributes, &drawables, attachmentNode, lightSetup);
    } else {
      // support creation when simulator DNE
      return addObject(attributes, nullptr, attachmentNode, lightSetup);
    }
  }
}  // PhysicsManager::addObject

int PhysicsManager::addObject(
    const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
    DrawableGroup* drawables,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  //! Make rigid object and add it to existingObjects
  if (!objectAttributes) {
    // should never run, but just in case
    ESP_ERROR() << "Object creation failed due to nonexistant "
                   "objectAttributes";
    return ID_UNDEFINED;
  }
  // verify whether necessary assets exist, and if not, instantiate them
  // only make object if asset instantiation succeeds (short circuit)
  bool objectSuccess =
      resourceManager_.instantiateAssetsOnDemand(objectAttributes);
  if (!objectSuccess) {
    ESP_ERROR() << "ResourceManager::instantiateAssetsOnDemand "
                   "unsuccessful. Aborting.";
    return ID_UNDEFINED;
  }

  // derive valid object ID and create new node if necessary
  int nextObjectID_ = allocateObjectID();
  scene::SceneNode* objectNode = attachmentNode;
  if (attachmentNode == nullptr) {
    objectNode = &staticStageObject_->node().createChild();
  }

  objectSuccess =
      makeAndAddRigidObject(nextObjectID_, objectAttributes, objectNode);

  if (!objectSuccess) {
    deallocateObjectID(nextObjectID_);
    if (attachmentNode == nullptr) {
      delete objectNode;
    }
    ESP_ERROR() << "PhysicsManager::makeRigidObject unsuccessful. "
                   " Aborting.";
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
    // if failed for some reason, remove and return
    removeObject(nextObjectID_, true, true);
    ESP_ERROR() << "PhysicsManager::finalizeObject unsuccessful.  Aborting.";
    return ID_UNDEFINED;
  }
  // Valid object exists by here.
  // Now we need to create wrapper, wrap around object,
  // and register wrapper with wrapper manager
  // 1.0 Get unique name for object using simplified attributes name.
  std::string simpleObjectHandle = objectAttributes->getSimplifiedHandle();
  std::string newObjectHandle =
      rigidObjectManager_->getUniqueHandleFromCandidate(simpleObjectHandle);
  ESP_WARNING() << "Simplified template handle :" << simpleObjectHandle
                << " | newObjectHandle :" << newObjectHandle;

  existingObjects_.at(nextObjectID_)->setObjectName(newObjectHandle);

  // 2.0 Get wrapper - name is irrelevant, do not register.
  ManagedRigidObject::ptr objWrapper = getRigidObjectWrapper();

  // 3.0 Put object in wrapper
  objWrapper->setObjectRef(existingObjects_.at(nextObjectID_));

  // 4.0 register wrapper in manager
  rigidObjectManager_->registerObject(objWrapper, newObjectHandle);

  return nextObjectID_;
}  // PhysicsManager::addObject

int PhysicsManager::addArticulatedObjectInstance(
    const std::string& filepath,
    const std::shared_ptr<
        const esp::metadata::attributes::SceneAOInstanceAttributes>&
        aObjInstAttributes,
    const std::string& lightSetup) {
  // Get drawables from simulator. TODO: Support non-existent simulator?
  auto& drawables = simulator_->getDrawableGroup();

  // check if an object is being set to be not visible for a particular
  // instance.
  int visSet = aObjInstAttributes->getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // specfied in scene instance
    // objAttributes->setIsVisible(visSet == 1);
    // TODO: manage articulated object visibility.
  }

  // call object creation (resides only in physics library-based derived physics
  // managers)
  int aObjID = this->addArticulatedObjectFromURDF(
      filepath, &drawables, aObjInstAttributes->getFixedBase(),
      aObjInstAttributes->getUniformScale(), aObjInstAttributes->getMassScale(),
      false, false, lightSetup);
  if (aObjID == ID_UNDEFINED) {
    // instancing failed for some reason.
    ESP_ERROR() << "Articulated Object create failed for model filepath"
                << filepath << ", whose handle is"
                << aObjInstAttributes->getHandle()
                << "as specified in articulated object instance attributes.";
    return ID_UNDEFINED;
  }

  // set articulated object up using scene instance
  auto aObjPtr = existingArticulatedObjects_.at(aObjID);
  // set articulated object's scene instancing attributes
  aObjPtr->setSceneInstanceAttr(aObjInstAttributes);

  // merge articulated object's user-defined attributes, if any exist in scene
  // instance.
  aObjPtr->mergeUserAttributes(aObjInstAttributes->getUserConfiguration());

  // set articulated object's location, rotation and other pertinent state
  // values based on
  // scene object instance attributes set in the object above.
  aObjPtr->resetStateFromSceneInstanceAttr();

  return aObjID;
}  // PhysicsManager::addArticulatedObjectInstance

esp::physics::ManagedRigidObject::ptr PhysicsManager::getRigidObjectWrapper() {
  return rigidObjectManager_->createObject("ManagedRigidObject");
}

esp::physics::ManagedArticulatedObject::ptr
PhysicsManager::getArticulatedObjectWrapper() {
  // should never be called unless we support non-dynamic AOs - would only be
  // called from AO creation occurring from within PM
  return articulatedObjectManager_->createObject("ManagedArticulatedObject");
}

void PhysicsManager::removeObject(const int objectId,
                                  bool deleteObjectNode,
                                  bool deleteVisualNode) {
  assertRigidIdValidity(objectId);
  scene::SceneNode* objectNode = &existingObjects_.at(objectId)->node();
  scene::SceneNode* visualNode = existingObjects_.at(objectId)->visualNode_;
  std::string objName = existingObjects_.at(objectId)->getObjectName();
  existingObjects_.erase(objectId);
  deallocateObjectID(objectId);
  if (deleteObjectNode) {
    delete objectNode;
  } else if (deleteVisualNode && visualNode) {
    delete visualNode;
  }
  // remove wrapper if one is present
  if (rigidObjectManager_->getObjectLibHasHandle(objName)) {
    rigidObjectManager_->removeObjectByID(objectId);
  }
}  // PhysicsManager::removeObject

void PhysicsManager::removeArticulatedObject(int objectId) {
  CORRADE_INTERNAL_ASSERT(existingArticulatedObjects_.count(objectId));
  scene::SceneNode* objectNode =
      &existingArticulatedObjects_.at(objectId)->node();
  for (auto linkObjId :
       existingArticulatedObjects_.at(objectId)->objectIdToLinkId_) {
    deallocateObjectID(linkObjId.first);
  }
  std::string artObjName =
      existingArticulatedObjects_.at(objectId)->getObjectName();
  existingArticulatedObjects_.erase(objectId);
  deallocateObjectID(objectId);
  delete objectNode;
  // remove wrapper if one is present
  if (articulatedObjectManager_->getObjectLibHasHandle(artObjName)) {
    articulatedObjectManager_->removeObjectByID(objectId);
  }
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
    const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
    scene::SceneNode* objectNode) {
  auto ptr =
      physics::RigidObject::create(objectNode, newObjectID, resourceManager_);
  bool objSuccess = ptr->initialize(objectAttributes);
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

    // kinematic velocity control integration
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
  for (auto& ao : existingArticulatedObjects_)
    ao.second->deferUpdate();
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

#ifdef ESP_BUILD_WITH_VHACD
void PhysicsManager::generateVoxelization(const int physObjectID,
                                          const int resolution) {
  assertRigidIdValidity(physObjectID);
  existingObjects_.at(physObjectID)
      ->generateVoxelization(resourceManager_, resolution);
}

void PhysicsManager::generateStageVoxelization(const int resolution) {
  staticStageObject_->generateVoxelization(resourceManager_, resolution);
}
#endif

std::shared_ptr<esp::geo::VoxelWrapper> PhysicsManager::getObjectVoxelization(
    const int physObjectID) const {
  assertRigidIdValidity(physObjectID);
  return existingObjects_.at(physObjectID)->getVoxelization();
}

std::shared_ptr<esp::geo::VoxelWrapper> PhysicsManager::getStageVoxelization()
    const {
  return staticStageObject_->getVoxelization();
}

void PhysicsManager::setObjectBBDraw(int physObjectID,
                                     DrawableGroup* drawables,
                                     bool drawBB) {
  assertRigidIdValidity(physObjectID);
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

void PhysicsManager::setObjectVoxelizationDraw(int physObjectID,
                                               const std::string& gridName,
                                               DrawableGroup* drawables,
                                               bool drawVoxelization) {
  assertRigidIdValidity(physObjectID);
  setVoxelizationDraw(gridName,
                      static_cast<esp::physics::RigidBase*>(
                          existingObjects_.at(physObjectID).get()),
                      drawables, drawVoxelization);
}

void PhysicsManager::setStageVoxelizationDraw(const std::string& gridName,
                                              DrawableGroup* drawables,
                                              bool drawVoxelization) {
  setVoxelizationDraw(
      gridName, static_cast<esp::physics::RigidBase*>(staticStageObject_.get()),
      drawables, drawVoxelization);
}

void PhysicsManager::setVoxelizationDraw(const std::string& gridName,
                                         esp::physics::RigidBase* rigidBase,
                                         DrawableGroup* drawables,
                                         bool drawVoxelization) {
  if (rigidBase->VoxelNode_ && !drawVoxelization) {
    // destroy the node
    delete rigidBase->VoxelNode_;
    rigidBase->VoxelNode_ = nullptr;

  } else if (drawVoxelization && rigidBase->visualNode_) {
    // if the VoxelNode is already rendering something, destroy it.
    delete rigidBase->VoxelNode_;

    // re-create the voxel node
    rigidBase->VoxelNode_ = &rigidBase->visualNode_->createChild();

    esp::geo::VoxelWrapper* voxelWrapper_ = rigidBase->voxelWrapper.get();
    gfx::Drawable::Flags meshAttributeFlags{};
    resourceManager_.createDrawable(
        &voxelWrapper_->getVoxelGrid()->getMeshGL(gridName), meshAttributeFlags,
        *rigidBase->VoxelNode_, DEFAULT_LIGHTING_KEY,
        PER_VERTEX_OBJECT_ID_MATERIAL_KEY, drawables);

    // If the RigidBase is a stage, need to set the BB to make culling work.
    if (dynamic_cast<esp::physics::RigidStage*>(rigidBase) != nullptr) {
      // set bounding box for the node to be the bb computed by vhacd
      Mn::Range3D bb{rigidBase->voxelWrapper->getVoxelGrid()->getOffset(),
                     rigidBase->voxelWrapper->getVoxelGrid()->getMaxOffset()};
      rigidBase->VoxelNode_->setMeshBB(bb);
      //
      rigidBase->node().computeCumulativeBB();
    }
  }
}
}  // namespace physics
}  // namespace esp
