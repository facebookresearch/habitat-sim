// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObjectManager.h"
namespace esp {
namespace physics {

int RigidObjectManager::addObjectByHandle(const std::string& attributesHandle,
                                          scene::SceneNode* attachmentNode,
                                          const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    return physMgr->addObject(attributesHandle, attachmentNode, lightSetup);
  } else {
    return ID_UNDEFINED;
  }
}  // RigidObjectManager::addObject */

int RigidObjectManager::addObjectByID(const int attributesID,
                                      scene::SceneNode* attachmentNode,
                                      const std::string& lightSetup) {
  if (auto physMgr = this->getPhysicsManager()) {
    return physMgr->addObject(attributesID, attachmentNode, lightSetup);
  } else {
    return ID_UNDEFINED;
  }
}  // RigidObjectManager::addObject

}  // namespace physics
}  // namespace esp
