// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManager.h"
#include <Magnum/Math/Range.h>

#include <utility>
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/ResourceManager.h"
#include "esp/metadata/managers/AOAttributesManager.h"
#include "esp/metadata/managers/ObjectAttributesManager.h"
#include "esp/metadata/managers/PhysicsAttributesManager.h"
#include "esp/metadata/managers/SceneInstanceAttributesManager.h"
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

/////////////////////////////////
// Stage Creation

bool PhysicsManager::addStageInstance(
    const metadata::attributes::StageAttributes::ptr& initAttributes,
    const metadata::attributes::SceneObjectInstanceAttributes::cptr&
        stageInstanceAttributes) {
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

/////////////////////////////////
// Object Creation

int PhysicsManager::addObject(const std::string& attributesHandle,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  esp::metadata::attributes::ObjectAttributes::ptr attributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByHandle(
          attributesHandle);
  if (!attributes) {
    ESP_ERROR() << "Object creation failed due to unknown attributes"
                << attributesHandle;
    return ID_UNDEFINED;
  }
  // create object. drawables may be nullptr.
  return addObjectAndSaveAttributes(attributes, drawables, attachmentNode,
                                    lightSetup);
}  // PhysicsManager::addObject

int PhysicsManager::addObject(int attributesID,
                              DrawableGroup* drawables,
                              scene::SceneNode* attachmentNode,
                              const std::string& lightSetup) {
  const esp::metadata::attributes::ObjectAttributes::ptr attributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByID(
          attributesID);
  if (!attributes) {
    ESP_ERROR() << "Object creation failed due to unknown attributes ID"
                << attributesID;
    return ID_UNDEFINED;
  }
  // create object. drawables may be nullptr.
  return addObjectAndSaveAttributes(attributes, drawables, attachmentNode,
                                    lightSetup);
}  // PhysicsManager::addObject

int PhysicsManager::addObjectInstance(
    const esp::metadata::attributes::SceneObjectInstanceAttributes::cptr&
        objInstAttributes,
    DrawableGroup* drawables,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  // Template attributes handle from instance
  const std::string objAttrHandle = objInstAttributes->getHandle();
  // Get full object template attributes handle
  const std::string attributesHandle =
      resourceManager_.getObjectAttributesManager()->getFullAttrNameFromStr(
          objAttrHandle);

  // make sure full handle is not empty, meaning template is not found in
  // manager
  ESP_CHECK(!attributesHandle.empty(),
            Cr::Utility::formatString(
                "PhysicsManager::addObjectInstance() : Attempt to "
                "load object instance specified in current scene instance "
                ":{} failed due to object instance configuration handle '{}' "
                "being empty or unknown. Aborting",
                simulator_->getActiveSceneDatasetName(), objAttrHandle));

  // Get ObjectAttributes
  auto objAttributes =
      resourceManager_.getObjectAttributesManager()->getObjectCopyByHandle(
          attributesHandle);

  if (!objAttributes) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Missing/improperly configured ObjectAttributes '"
        << attributesHandle << "', whose handle contains '" << objAttrHandle
        << "' as specified in object instance attributes, so addObjectInstance "
           "aborted.";
    return ID_UNDEFINED;
  }
  // check if an object is being set to be not visible for a particular
  // instance.
  int visSet = objInstAttributes->getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // specfied in scene instance
    objAttributes->setIsVisible(visSet == 1);
  }

  // set shader type to use for object instance, which may override shadertype
  // specified in object attributes.
  const auto objShaderType = objInstAttributes->getShaderType();
  if (objShaderType !=
      metadata::attributes::ObjectInstanceShaderType::Unspecified) {
    objAttributes->setShaderType(getShaderTypeName(objShaderType));
  }

  // set scaling values for this instance of object attributes - first uniform
  // scaling
  objAttributes->setScale(objAttributes->getScale() *
                          objInstAttributes->getUniformScale());
  // set scaling values for this instance of object attributes - next
  // non-uniform scaling
  objAttributes->setScale(objAttributes->getScale() *
                          objInstAttributes->getNonUniformScale());

  // If boolean specifies to do so, apply geometric scaling to mass (product of
  // scale values)
  if (objInstAttributes->getApplyScaleToMass()) {
    objAttributes->setMass(
        objAttributes->getMass() *
        static_cast<double>(objAttributes->getScale().product()));
  }
  // set scaled mass
  objAttributes->setMass(objAttributes->getMass() *
                         objInstAttributes->getMassScale());

  return addObjectAndSaveAttributes(objAttributes, drawables, attachmentNode,
                                    lightSetup, objInstAttributes);

}  // PhysicsManager::addObjectInstance

int PhysicsManager::cloneExistingObject(int objectID) {
  // Retrieve object by ID
  const auto existingObjIter = existingObjects_.find(objectID);
  // Verify object id exists and is a rigid object
  if (existingObjIter == existingObjects_.end()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Object cloning failed due to unknown existing object ID `"
        << objectID << "`. Aborting";
    return ID_UNDEFINED;
  }
  auto objPtr = existingObjIter->second;
  // Get object instance attributes copy with current state of object instance
  esp::metadata::attributes::SceneObjectInstanceAttributes::ptr
      newObjInstAttrs = objPtr->getCurrentStateInstanceAttr();

  // Create object instance
  int newObjID =
      addObjectInstance(newObjInstAttrs, &simulator_->getDrawableGroup(),
                        nullptr, simulator_->getCurrentLightSetupKey());

  return newObjID;

}  // PhysicsManager::cloneExistingObject

int PhysicsManager::addObjectAndSaveAttributes(
    const esp::metadata::attributes::ObjectAttributes::ptr& objAttributes,
    DrawableGroup* drawables,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup,
    esp::metadata::attributes::SceneObjectInstanceAttributes::cptr
        objInstAttributes) {
  bool defaultCOMCorrection = false;
  if (simulator_ != nullptr) {
    // get defaultCOMCorrection from simulator
    defaultCOMCorrection = simulator_->getCurSceneDefaultCOMHandling();
    // If no drawables were passed, and a simulator exists
    // retrieve a drawable group to use
    if (drawables == nullptr) {
      // acquire context if available
      simulator_->getRenderGLContext();
      // acquire an appropriate drawable group
      drawables = &simulator_->getDrawableGroup();
    }
  }

  if (objInstAttributes == nullptr) {
    // Create objInstAttributes and populate with start values from config
    // attributes.
    // Use simplified handle to reference attributes
    // TODO : probably need something more specific eventually
    objInstAttributes =
        resourceManager_.getSceneInstanceAttributesManager()
            ->createEmptyInstanceAttributes(
                objAttributes->getSimplifiedHandle(), objAttributes);
  }

  // create and add object using provided object attributes
  int objID =
      addObjectInternal(objAttributes, drawables, attachmentNode, lightSetup);

  if (objID == ID_UNDEFINED) {
    // instancing failed for some reason.
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Object create failed for ObjectAttributes '"
        << objAttributes->getHandle() << "', whose handle contains '"
        << objInstAttributes->getHandle()
        << "' as specified in object instance attributes, so addObjectInstance "
           "aborted.";
    return ID_UNDEFINED;
  }
  auto objPtr = this->existingObjects_.at(objID);

  // save the scene init attributes used to configure object's initial state
  objPtr->setSceneInstanceAttr(objInstAttributes);
  // merge scene instance user-defined configurations with the new object's, if
  // scene instance specifies any set articulated object's user-defined
  // attributes, if any exist in scene instance.
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

}  // PhysicsManager::addObjectAndSaveAttributes

int PhysicsManager::addObjectInternal(
    const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
    DrawableGroup* drawables,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  //! Make rigid object and add it to existingObjects
  if (!objectAttributes) {
    // should never run, but just in case
    ESP_ERROR() << "Object creation failed due to nonexistant "
                   "objectAttributes, so addObject aborted.";
    return ID_UNDEFINED;
  }
  // verify whether necessary assets exist, and if not, instantiate them
  // only make object if asset instantiation succeeds (short circuit)
  bool objectSuccess =
      resourceManager_.instantiateAssetsOnDemand(objectAttributes);
  if (!objectSuccess) {
    ESP_ERROR() << "ResourceManager::instantiateAssetsOnDemand "
                   "unsuccessful, so addObject `"
                << objectAttributes->getHandle() << "` aborted.";
    return ID_UNDEFINED;
  }

  // derive valid object ID and create new node if necessary
  int newObjectID = allocateObjectID();
  scene::SceneNode* objectNode = attachmentNode;
  if (attachmentNode == nullptr) {
    objectNode = &staticStageObject_->node().createChild();
  }
  // Attempt to create a new object, initialize it and add to existingObjects_
  objectSuccess =
      makeAndAddRigidObject(newObjectID, objectAttributes, objectNode);

  if (!objectSuccess) {
    deallocateObjectID(newObjectID);
    if (attachmentNode == nullptr) {
      delete objectNode;
    }
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "PhysicsManager::makeAndAddRigidObject unsuccessful, so addObject `"
        << objectAttributes->getHandle() << "` aborted.";
    return ID_UNDEFINED;
  }

  // temp non-owning pointer to object
  esp::physics::RigidObject* const obj =
      (existingObjects_.at(newObjectID).get());

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
    removeObject(newObjectID, true, true);
    ESP_ERROR() << "PhysicsManager::finalizeObject unsuccessful, so addObject `"
                << objectAttributes->getHandle() << "` aborted.";
    return ID_UNDEFINED;
  }
  // Valid object exists by here.
  // Now we need to create wrapper, wrap around object,
  // and register wrapper with wrapper manager
  // 1.0 Get unique name for object using simplified attributes name.
  std::string simpleObjectHandle = objectAttributes->getSimplifiedHandle();
  std::string newObjectHandle =
      rigidObjectManager_->getUniqueHandleFromCandidate(simpleObjectHandle);
  ESP_DEBUG() << "Simplified template handle :" << simpleObjectHandle
              << " | newObjectHandle :" << newObjectHandle;

  obj->setObjectName(newObjectHandle);

  // 2.0 Get wrapper - name is irrelevant, do not register.
  ManagedRigidObject::ptr objWrapper = getRigidObjectWrapper();

  // 3.0 Put object in wrapper
  objWrapper->setObjectRef(existingObjects_.at(newObjectID));

  // 4.0 register wrapper in manager
  rigidObjectManager_->registerObject(objWrapper, newObjectHandle);

  // 4.5 register wrapper with object it contains
  obj->setManagedObjectPtr(objWrapper);

  return newObjectID;
}  // PhysicsManager::addObject

/////////////////////////////////
// Articulated Object Creation

int PhysicsManager::addArticulatedObject(const std::string& attributesHandle,
                                         DrawableGroup* drawables,
                                         bool forceReload,
                                         const std::string& lightSetup) {
  esp::metadata::attributes::ArticulatedObjectAttributes::ptr attributes =
      resourceManager_.getAOAttributesManager()->getObjectCopyByHandle(
          attributesHandle);
  if (!attributes) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Articulated Object creation failed due to unknown attributes '"
        << attributesHandle << "'";
    return ID_UNDEFINED;
  }
  return addArticulatedObjectAndSaveAttributes(attributes, drawables,
                                               forceReload, lightSetup);
}  // PhysicsManager::addArticulatedObject

int PhysicsManager::addArticulatedObject(int attributesID,
                                         DrawableGroup* drawables,
                                         bool forceReload,
                                         const std::string& lightSetup) {
  const esp::metadata::attributes::ArticulatedObjectAttributes::ptr attributes =
      resourceManager_.getAOAttributesManager()->getObjectCopyByID(
          attributesID);
  if (!attributes) {
    ESP_ERROR()
        << "Articulated Object creation failed due to unknown attributes ID"
        << attributesID;
    return ID_UNDEFINED;
  }

  return addArticulatedObjectAndSaveAttributes(attributes, drawables,
                                               forceReload, lightSetup);
}  // PhysicsManager::addArticulatedObject

int PhysicsManager::addArticulatedObjectFromURDF(
    const std::string& filepath,
    DrawableGroup* drawables,
    bool fixedBase,
    float globalScale,
    float massScale,
    bool forceReload,
    bool maintainLinkOrder,
    bool intertiaFromURDF,
    const std::string& lightSetup) {
  // Retrieve or create the appropriate ArticulatedObjectAttributes to create
  // this AO.

  bool attribsFound =
      resourceManager_.getAOAttributesManager()->getObjectLibHasHandle(
          filepath);

  esp::metadata::attributes::ArticulatedObjectAttributes::ptr artObjAttributes =
      attribsFound
          ? resourceManager_.getAOAttributesManager()->getObjectCopyByHandle(
                filepath)
          : resourceManager_.getAOAttributesManager()->createObject(filepath,
                                                                    true);

  // Set pertinent values
  artObjAttributes->setUniformScale(globalScale);
  artObjAttributes->setMassScale(static_cast<double>(massScale));

  artObjAttributes->setBaseType(metadata::attributes::getAOBaseTypeName(
      fixedBase ? metadata::attributes::ArticulatedObjectBaseType::Fixed
                : metadata::attributes::ArticulatedObjectBaseType::Free));

  artObjAttributes->setInertiaSource(
      metadata::attributes::getAOInertiaSourceName(
          intertiaFromURDF
              ? metadata::attributes::ArticulatedObjectInertiaSource::URDF
              : metadata::attributes::ArticulatedObjectInertiaSource::
                    Computed));

  artObjAttributes->setLinkOrder(metadata::attributes::getAOLinkOrderName(
      maintainLinkOrder
          ? metadata::attributes::ArticulatedObjectLinkOrder::URDFOrder
          : metadata::attributes::ArticulatedObjectLinkOrder::TreeTraversal));

  return addArticulatedObjectAndSaveAttributes(artObjAttributes, drawables,
                                               forceReload, lightSetup);

}  // PhysicsManager::addArticulatedObjectFromURDF

int PhysicsManager::addArticulatedObjectInstance(
    const std::shared_ptr<
        const esp::metadata::attributes::SceneAOInstanceAttributes>&
        aObjInstAttributes,
    DrawableGroup* drawables,
    const std::string& lightSetup) {
  // AO template attributes handle from articulated object instance
  const std::string artObjHandle = aObjInstAttributes->getHandle();

  // Get model file name from handle
  const std::string artObjAttrHandle =
      resourceManager_.getAOAttributesManager()->getFullAttrNameFromStr(
          artObjHandle);

  // make sure full handle is not empty
  ESP_CHECK(
      !artObjAttrHandle.empty(),
      Cr::Utility::formatString(
          "PhysicsManager::addArticulatedObjectInstance() : "
          "Attempt to load articulated object instance specified in "
          "current scene instance :{} failed due to AO instance "
          "configuration file handle '{}' being empty or unknown. Aborting",
          simulator_->getActiveSceneDatasetName(), artObjHandle));

  // Get ArticulatedObjectAttributes
  auto artObjAttributes =
      resourceManager_.getAOAttributesManager()->getObjectCopyByHandle(
          artObjAttrHandle);

  if (!artObjAttributes) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Missing/improperly configured ArticulatedObjectAttributes '"
        << artObjAttrHandle << "', whose handle contains '"
        << aObjInstAttributes->getHandle()
        << "' as specified in articulated object instance attributes, so "
           "addArticulatedObjectInstance aborted.";
    return ID_UNDEFINED;
  }

  // check if an object is being set to be not visible for a particular
  // instance.
  int visSet = aObjInstAttributes->getIsInstanceVisible();
  if (visSet != ID_UNDEFINED) {
    // specfied in scene instance
    // artObjAttributes->setIsVisible(visSet == 1);
    // TODO: manage articulated object visibility.
  }
  // set shader type to use for articulated object instance, which may override
  // shadertype specified in articulated object attributes.
  const auto artObjShaderType = aObjInstAttributes->getShaderType();
  if (artObjShaderType !=
      metadata::attributes::ObjectInstanceShaderType::Unspecified) {
    artObjAttributes->setShaderType(getShaderTypeName(artObjShaderType));
  }

  // set uniform scale
  artObjAttributes->setUniformScale(artObjAttributes->getUniformScale() *
                                    aObjInstAttributes->getUniformScale());

  // If boolean specifies to do so, apply geometric scaling to mass (product of
  // scale values)
  if (aObjInstAttributes->getApplyScaleToMass()) {
    artObjAttributes->setMassScale(
        artObjAttributes->getMassScale() *
        static_cast<double>(artObjAttributes->getUniformScale()));
  }

  // set scaled mass
  artObjAttributes->setMassScale(artObjAttributes->getMassScale() *
                                 aObjInstAttributes->getMassScale());

  const auto baseType = aObjInstAttributes->getBaseType();
  if (baseType !=
      metadata::attributes::ArticulatedObjectBaseType::Unspecified) {
    artObjAttributes->setBaseType(
        metadata::attributes::getAOBaseTypeName(baseType));
  }

  const auto inertiaSrc = aObjInstAttributes->getInertiaSource();
  if (inertiaSrc !=
      metadata::attributes::ArticulatedObjectInertiaSource::Unspecified) {
    artObjAttributes->setInertiaSource(
        metadata::attributes::getAOInertiaSourceName(inertiaSrc));
  }

  const auto linkOrder = aObjInstAttributes->getLinkOrder();
  if (linkOrder !=
      metadata::attributes::ArticulatedObjectLinkOrder::Unspecified) {
    artObjAttributes->setLinkOrder(
        metadata::attributes::getAOLinkOrderName(linkOrder));
  }

  return addArticulatedObjectAndSaveAttributes(
      artObjAttributes, drawables, false, lightSetup, aObjInstAttributes);

}  // PhysicsManager::addArticulatedObjectInstance

int PhysicsManager::cloneExistingArticulatedObject(int aObjectID) {
  // Retrieve object by ID
  const auto existingAOIter = existingArticulatedObjects_.find(aObjectID);
  // Verify object id exists and is a rigid object
  if (existingAOIter == existingArticulatedObjects_.end()) {
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Articulated Object cloning failed due to unknown existing "
           "articulated object ID `"
        << aObjectID << "`. Aborting";
    return ID_UNDEFINED;
  }
  auto aObjPtr = existingAOIter->second;
  // Get articulated object instance attributes copy with current state of AO
  esp::metadata::attributes::SceneAOInstanceAttributes::ptr artObjInstAttrs =
      aObjPtr->getCurrentStateInstanceAttr();

  // Create object instance
  int newArtObjID = addArticulatedObjectInstance(
      artObjInstAttrs, &simulator_->getDrawableGroup(),
      simulator_->getCurrentLightSetupKey());

  return newArtObjID;

}  // PhysicsManager::cloneExistingArticulatedObject

int PhysicsManager::addArticulatedObjectAndSaveAttributes(
    const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
        artObjAttributes,
    DrawableGroup* drawables,
    bool forceReload,
    const std::string& lightSetup,
    esp::metadata::attributes::SceneAOInstanceAttributes::cptr
        aObjInstAttributes) {
  // If no drawables were passed, and a simulator exists
  // retrieve a drawable group to use
  if ((drawables == nullptr) && (simulator_ != nullptr)) {
    // acquire context if available
    simulator_->getRenderGLContext();
    // acquire an appropriate drawable group
    drawables = &simulator_->getDrawableGroup();
  }
  // If no drawables, do not create articulated object
  if (drawables == nullptr) {
    // TODO Support non-existent simulator?
    return ID_UNDEFINED;
  }
  if (aObjInstAttributes == nullptr) {
    // Create aObjInstAttributes and populate with start values from config
    // attributes.
    // Use simplified handle to reference attributes
    // TODO : probably need something more specific eventually
    aObjInstAttributes =
        resourceManager_.getSceneInstanceAttributesManager()
            ->createEmptyAOInstanceAttributes(
                artObjAttributes->getSimplifiedHandle(), artObjAttributes);
    // TODO do we need to save this to curSceneInstanceAttributes responsible
    // for this scene?
  }

  int aObjID = addArticulatedObjectInternal(artObjAttributes, drawables,
                                            forceReload, lightSetup);

  if (aObjID == ID_UNDEFINED) {
    // instancing failed for some reason.
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Articulated Object create failed for ArticulatedObjectAttributes '"
        << artObjAttributes->getHandle() << "', whose handle contains '"
        << aObjInstAttributes->getHandle()
        << "' as specified in articulated object instance attributes, so "
           "addArticulatedObjectInstance aborted.";
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

}  // PhysicsManager::addArticulatedObjectAndSaveAttributes

void PhysicsManager::buildCurrentStateSceneAttributes(
    const metadata::attributes::SceneInstanceAttributes::ptr&
        sceneInstanceAttrs) const {
  // 1. set stage instance
  sceneInstanceAttrs->setStageInstanceAttrs(
      staticStageObject_->getCurrentStateInstanceAttr());
  // 2. Clear existing object instances, and set new ones reflecting current
  // state
  sceneInstanceAttrs->clearObjectInstances();

  // TODO : drive by diagnostics when implemented
  bool validateUnique = false;

  // get each object's current state as a SceneObjectInstanceAttributes
  for (const auto& item : existingObjects_) {
    sceneInstanceAttrs->addObjectInstanceAttrs(
        item.second->getCurrentStateInstanceAttr(), validateUnique);
  }
  // 3. Clear existing Articulated object instances, and set new ones reflecting
  // current state
  sceneInstanceAttrs->clearArticulatedObjectInstances();
  // get each articulated object's current state as a SceneAOInstanceAttributes
  for (const auto& item : existingArticulatedObjects_) {
    sceneInstanceAttrs->addArticulatedObjectInstanceAttrs(
        item.second->getCurrentStateInstanceAttr(), validateUnique);
  }

}  // PhysicsManager::buildCurrentStateSceneAttributes

int PhysicsManager::addTrajectoryObject(const std::string& trajVisName,
                                        const std::vector<Mn::Vector3>& pts,
                                        const std::vector<Mn::Color3>& colorVec,
                                        int numSegments,
                                        float radius,
                                        bool smooth,
                                        int numInterp,
                                        DrawableGroup* drawables) {
  if (simulator_ != nullptr) {
    // acquire context if available
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
                << trajVisName << "so addTrajectoryObject aborted.";
    return ID_UNDEFINED;
  }
  // 2. create object attributes for the trajectory
  auto objAttrMgr = resourceManager_.getObjectAttributesManager();
  auto trajObjAttr = objAttrMgr->createObject(trajVisName, false);
  // turn off collisions
  trajObjAttr->setIsCollidable(false);
  trajObjAttr->setComputeCOMFromShape(false);
  objAttrMgr->registerObject(trajObjAttr, trajVisName, true);

  // 3. query for drawables
  if ((drawables == nullptr) && (simulator_ != nullptr)) {
    // acquire context if available
    simulator_->getRenderGLContext();
    // acquire an appropriate drawable group
    drawables = &simulator_->getDrawableGroup();
  }

  // 4. add trajectory object to manager
  auto trajVisID = addObjectInternal(trajObjAttr, drawables);
  if (trajVisID == ID_UNDEFINED) {
    // failed to add object - need to delete asset from resourceManager.
    ESP_ERROR() << "Failed to create Trajectory visualization object for"
                << trajVisName << "so addTrajectoryObject aborted.";
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
    // acquire context if available
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
  } else if (visualNode) {
    if (deleteVisualNode) {
      delete visualNode;
    } else {
      // Clear out the object ID that was set for the owning visual node,
      // setting to stage ID
      visualNode->setBaseObjectId(RIGID_STAGE_ID);
    }
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
    // resourceManager_.removeResourceByName(trajVisAssetName);
  }
}  // PhysicsManager::removeObject

void PhysicsManager::removeArticulatedObject(int objectId) {
  if (simulator_ != nullptr) {
    // acquire context if available
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

metadata::attributes::PhysicsManagerAttributes::ptr
PhysicsManager::getInitializationAttributes() const {
  return metadata::attributes::PhysicsManagerAttributes::create(
      *physicsManagerAttributes_);
}

}  // namespace physics
}  // namespace esp
