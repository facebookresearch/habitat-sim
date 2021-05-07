// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObjectManager.h"
namespace esp {
namespace physics {

std::shared_ptr<ManagedRigidObject> RigidObjectManager::addObjectByHandle(
    const std::string& attributesHandle,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newObjID =
        physMgr->addObject(attributesHandle, attachmentNode, lightSetup);
    return this->getObjectCopyByID(newObjID);
  } else {
    return nullptr;
  }
}  // RigidObjectManager::addObject */

std::shared_ptr<ManagedRigidObject> RigidObjectManager::addObjectByID(
    const int attributesID,
    scene::SceneNode* attachmentNode,
    const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    int newObjID = physMgr->addObject(attributesID, attachmentNode, lightSetup);
    return this->getObjectCopyByID(newObjID);
  } else {
    return nullptr;
  }
}  // RigidObjectManager::addObject

std::shared_ptr<ManagedRigidObject> RigidObjectManager::removePhysObjectByID(
    int objectID,
    bool deleteObjectNode,
    bool deleteVisualNode) {
  if (auto physMgr = this->getPhysicsManager()) {
    physMgr->removeObject(objectID, deleteObjectNode, deleteVisualNode);
  }
  return nullptr;
}  // RigidObjectManager::removeObjectByID

std::shared_ptr<ManagedRigidObject>
RigidObjectManager::removePhysObjectByHandle(const std::string& objectHandle,
                                             bool deleteObjectNode,
                                             bool deleteVisualNode) {
  if (auto physMgr = this->getPhysicsManager()) {
    int objectID = this->getObjectIDByHandle(objectHandle);
    if (objectID == ID_UNDEFINED) {
      return nullptr;
    }
    physMgr->removeObject(objectID, deleteObjectNode, deleteVisualNode);
  }
  return nullptr;
}  // RigidObjectManager::removeObjectByHandle

}  // namespace physics
}  // namespace esp
