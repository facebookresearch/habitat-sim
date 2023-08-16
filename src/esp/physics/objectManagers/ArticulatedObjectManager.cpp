// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ArticulatedObjectManager.h"

namespace esp {
namespace physics {

ArticulatedObjectManager::ArticulatedObjectManager()
    : esp::physics::PhysicsObjectBaseManager<ManagedArticulatedObject>::
          PhysicsObjectBaseManager("ArticulatedObject") {
  // build this manager's copy constructor map
  this->copyConstructorMap_["ManagedArticulatedObject"] =
      &ArticulatedObjectManager::createObjectCopy<ManagedArticulatedObject>;

  // build the function pointers to proper wrapper construction methods, keyed
  // by the wrapper names
  managedObjTypeConstructorMap_["ManagedArticulatedObject"] =
      &ArticulatedObjectManager::createPhysicsObjectWrapper<
          ManagedArticulatedObject>;

  this->copyConstructorMap_["ManagedBulletArticulatedObject"] =
      &ArticulatedObjectManager::createObjectCopy<
          ManagedBulletArticulatedObject>;
  managedObjTypeConstructorMap_["ManagedBulletArticulatedObject"] =
      &ArticulatedObjectManager::createPhysicsObjectWrapper<
          ManagedBulletArticulatedObject>;

}  // ctor

std::shared_ptr<ManagedArticulatedObject>
ArticulatedObjectManager::addArticulatedObjectFromURDF(
    const std::string& filepath,
    bool fixedBase,
    float globalScale,
    float massScale,
    bool forceReload,
    bool maintainLinkOrder,
    bool intertiaFromURDF,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newAObjID = physMgr->addArticulatedObjectFromURDF(
        filepath, fixedBase, globalScale, massScale, forceReload,
        maintainLinkOrder, intertiaFromURDF, lightSetup);
    return this->getObjectCopyByID(newAObjID);
  }
  return nullptr;
}

std::shared_ptr<ManagedArticulatedObject>
ArticulatedObjectManager::addArticulatedObjectByHandle(
    const std::string& attributesHandle,
    bool forceReload,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newAObjID = physMgr->addArticulatedObject(attributesHandle, forceReload,
                                                  lightSetup);
    return this->getObjectCopyByID(newAObjID);
  }
  return nullptr;
}

std::shared_ptr<ManagedArticulatedObject>
ArticulatedObjectManager::addArticulatedObjectByID(
    int attributesID,
    bool forceReload,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newAObjID =
        physMgr->addArticulatedObject(attributesID, forceReload, lightSetup);
    return this->getObjectCopyByID(newAObjID);
  }
  return nullptr;
}

std::shared_ptr<ManagedArticulatedObject>
ArticulatedObjectManager::addArticulatedObjectFromURDFWithDrawables(
    const std::string& filepath,
    gfx::DrawableGroup* drawables,
    bool fixedBase,
    float globalScale,
    float massScale,
    bool forceReload,
    bool maintainLinkOrder,
    bool intertiaFromURDF,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newAObjID = physMgr->addArticulatedObjectFromURDF(
        filepath, drawables, fixedBase, globalScale, massScale, forceReload,
        maintainLinkOrder, intertiaFromURDF, lightSetup);
    return this->getObjectCopyByID(newAObjID);
  }
  return nullptr;
}

}  // namespace physics
}  // namespace esp
