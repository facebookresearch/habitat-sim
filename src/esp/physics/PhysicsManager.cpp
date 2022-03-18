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
  if (sceneSuccess) {
    // save instance attributes used to create stage
    staticStageObject_->setSceneInstanceAttr(stageInstanceAttributes);
    // add/merge stageInstanceAttributes' copy of user_attributes.
    staticStageObject_->mergeUserAttributes(
        stageInstanceAttributes->getUserConfiguration());
  }
  // TODO process any stage transformations here from
  // stageInstanceAttributes

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
  // set shader type to use for object instance, which may override shadertype
  // specified in object attributes.
  const auto objShaderType = objInstAttributes->getShaderType();
  if (objShaderType !=
      metadata::attributes::ObjectInstanceShaderType::Unspecified) {
    objAttributes->setShaderType(getShaderTypeName(objShaderType));
  }
  int objID =
      addObjectQueryDrawables(objAttributes, attachmentNode, lightSetup);

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
  // determine and set if this object should be COM Corrected or not
  metadata::attributes::SceneInstanceTranslationOrigin instanceCOMOrigin =
      objInstAttributes->getTranslationOrigin();
  objPtr->setIsCOMCorrected(
      ((defaultCOMCorrection &&
        (instanceCOMOrigin !=
         metadata::attributes::SceneInstanceTranslationOrigin::COM)) ||
       (instanceCOMOrigin ==
        metadata::attributes::SceneInstanceTranslationOrigin::AssetLocal)));

  // set object's location, rotation and other pertinent state values based on
  // scene object instance attributes set in the object above.
  objPtr->resetStateFromSceneInstanceAttr();

  return objID;
}  // PhysicsManager::addObjectInstance

int PhysicsManager::addObjectQueryDrawables(
    const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  // attributes exist, get drawables if valid simulator accessible
  if (simulator_ != nullptr) {
    // aquire context if available
    simulator_->getRenderGLContext();
    auto& drawables = simulator_->getDrawableGroup();
    return addObject(objectAttributes, &drawables, attachmentNode, lightSetup);
  }
  // support creation when simulator DNE
  return addObject(objectAttributes, nullptr, attachmentNode, lightSetup);
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
  if (simulator_ == nullptr) {
    return ID_UNDEFINED;
  }

  // aquire context if available
  simulator_->getRenderGLContext();
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

void PhysicsManager::buildCurrentStateSceneAttributes(
    const metadata::attributes::SceneInstanceAttributes::ptr&
        sceneInstanceAttrs) const {
  // 1. set stage instance
  sceneInstanceAttrs->setStageInstance(
      staticStageObject_->getCurrentStateInstanceAttr());
  // 2. Clear existing object instances, and set new ones reflecting current
  // state
  sceneInstanceAttrs->clearObjectInstances();
  // get each object's current state as a SceneObjectInstanceAttributes
  for (const auto& item : existingObjects_) {
    sceneInstanceAttrs->addObjectInstance(
        item.second->getCurrentStateInstanceAttr());
  }
  // 3. Clear existing Articulated object instances, and set new ones reflecting
  // current state
  sceneInstanceAttrs->clearArticulatedObjectInstances();
  // get each articulated object's current state as a SceneAOInstanceAttributes
  for (const auto& item : existingArticulatedObjects_) {
    sceneInstanceAttrs->addArticulatedObjectInstance(
        item.second->getCurrentStateInstanceAttr());
  }

}  // PhysicsManager::buildCurrentStateSceneAttributes

int PhysicsManager::addTrajectoryObject(const std::string& trajVisName,
                                        const std::vector<Mn::Vector3>& pts,
                                        const std::vector<Mn::Color3>& colorVec,
                                        int numSegments,
                                        float radius,
                                        bool smooth,
                                        int numInterp) {
  if (simulator_ != nullptr) {
    // aquire context if available
    simulator_->getRenderGLContext();
  }
  // 0. Deduplicate sequential points
  std::vector<Magnum::Vector3> uniquePts;
  uniquePts.push_back(pts[0]);
  for (const auto& loc : pts) {
    if (loc != uniquePts.back()) {
      uniquePts.push_back(loc);
    }
  }

  // 1. create trajectory tube asset from points and save it
  bool success = resourceManager_.buildTrajectoryVisualization(
      trajVisName, uniquePts, colorVec, numSegments, radius, smooth, numInterp);
  if (!success) {
    ESP_ERROR() << "Failed to create Trajectory visualization mesh for"
                << trajVisName;
    return ID_UNDEFINED;
  }
  // 2. create object attributes for the trajectory
  auto objAttrMgr = resourceManager_.getObjectAttributesManager();
  auto trajObjAttr = objAttrMgr->createObject(trajVisName, false);
  // turn off collisions
  trajObjAttr->setIsCollidable(false);
  trajObjAttr->setComputeCOMFromShape(false);
  objAttrMgr->registerObject(trajObjAttr, trajVisName, true);

  // 3. add trajectory object to manager
  auto trajVisID = addObjectQueryDrawables(trajObjAttr);
  if (trajVisID == ID_UNDEFINED) {
    // failed to add object - need to delete asset from resourceManager.
    ESP_ERROR() << "Failed to create Trajectory visualization object for"
                << trajVisName;
    // TODO : support removing asset by removing from resourceDict_ properly
    // using trajVisName
    return ID_UNDEFINED;
  }
  auto trajObj = getRigidObjectManager()->getObjectCopyByID(trajVisID);
  ESP_DEBUG() << "Trajectory visualization object created with ID" << trajVisID;
  trajObj->setMotionType(esp::physics::MotionType::KINEMATIC);
  // add to internal references of object ID and resourceDict name
  // this is for eventual asset deletion/resource freeing.
  trajVisIDByName[trajVisName] = trajVisID;
  trajVisNameByID[trajVisID] = trajVisName;

  return trajVisID;

}  // PhysicsManager::addTrajectoryObject (vector of colors)

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
  if (simulator_ != nullptr) {
    // aquire context if available
    simulator_->getRenderGLContext();
  }
  auto existingObjIter = getRigidObjIteratorOrAssert(objectId);
  scene::SceneNode* objectNode = &existingObjIter->second->node();
  scene::SceneNode* visualNode = existingObjIter->second->visualNode_;
  std::string objName = existingObjIter->second->getObjectName();
  existingObjects_.erase(existingObjIter);
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
  // remove trajvis
  auto trajVisIter = trajVisNameByID.find(objectId);
  if (trajVisIter != trajVisNameByID.end()) {
    std::string trajVisAssetName = trajVisNameByID[objectId];
    trajVisNameByID.erase(trajVisIter);
    trajVisIDByName.erase(trajVisAssetName);
    // TODO : if object is trajectory visualization, remove its assets as
    // well once this is supported.
    // resourceManager_->removeResourceByName(trajVisAssetName);
  }
}  // PhysicsManager::removeObject

void PhysicsManager::removeArticulatedObject(int objectId) {
  if (simulator_ != nullptr) {
    // aquire context if available
    simulator_->getRenderGLContext();
  }
  auto existingAOIter = getArticulatedObjIteratorOrAssert(objectId);
  scene::SceneNode* objectNode = &existingAOIter->second->node();
  for (auto linkObjId : existingAOIter->second->objectIdToLinkId_) {
    deallocateObjectID(linkObjId.first);
  }
  std::string artObjName = existingAOIter->second->getObjectName();
  existingArticulatedObjects_.erase(existingAOIter);
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
  auto objIter = getRigidObjIteratorOrAssert(physObjectID);
  objIter->second->generateVoxelization(resourceManager_, resolution);
}

void PhysicsManager::generateStageVoxelization(const int resolution) {
  staticStageObject_->generateVoxelization(resourceManager_, resolution);
}
#endif

std::shared_ptr<esp::geo::VoxelWrapper> PhysicsManager::getObjectVoxelization(
    const int physObjectID) const {
  auto objIter = getConstRigidObjIteratorOrAssert(physObjectID);
  return objIter->second->getVoxelization();
}

std::shared_ptr<esp::geo::VoxelWrapper> PhysicsManager::getStageVoxelization()
    const {
  return staticStageObject_->getVoxelization();
}

void PhysicsManager::setObjectBBDraw(int physObjectID,
                                     DrawableGroup* drawables,
                                     bool drawBB) {
  auto objIter = getRigidObjIteratorOrAssert(physObjectID);
  if (objIter->second->BBNode_ && !drawBB) {
    // destroy the node
    delete objIter->second->BBNode_;
    objIter->second->BBNode_ = nullptr;
  } else if (drawBB && objIter->second->visualNode_) {
    // add a new BBNode
    Magnum::Vector3 scale =
        objIter->second->visualNode_->getCumulativeBB().size() / 2.0;
    objIter->second->BBNode_ = &objIter->second->visualNode_->createChild();
    objIter->second->BBNode_->MagnumObject::setScaling(scale);
    objIter->second->BBNode_->MagnumObject::setTranslation(
        existingObjects_[physObjectID]
            ->visualNode_->getCumulativeBB()
            .center());
    resourceManager_.addPrimitiveToDrawables(0, *objIter->second->BBNode_,
                                             drawables);
  }
}

void PhysicsManager::setObjectVoxelizationDraw(int physObjectID,
                                               const std::string& gridName,
                                               DrawableGroup* drawables,
                                               bool drawVoxelization) {
  auto objIter = getRigidObjIteratorOrAssert(physObjectID);
  setVoxelizationDraw(
      gridName, static_cast<esp::physics::RigidBase*>(objIter->second.get()),
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
