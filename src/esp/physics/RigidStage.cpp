// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidStage.h"

namespace esp {
namespace physics {

RigidStage::RigidStage(scene::SceneNode* rigidBodyNode,
                       const assets::ResourceManager& resMgr)
    : RigidBase(rigidBodyNode, resMgr) {}

bool RigidStage::initialize(const std::string& handle) {
  if (initializationAttributes_ != nullptr) {
    LOG(ERROR) << "Cannot initialize a RigidStage more than once";
    return false;
  }
  objectMotionType_ = MotionType::STATIC;
  initializationAttributes_ =
      resMgr_.getStageAttributesManager()->getObjectCopyByHandle(handle);

  return initialization_LibSpecific();
}

}  // namespace physics
}  // namespace esp
