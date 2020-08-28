// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidStage.h"

namespace esp {
namespace physics {

RigidStage::RigidStage(scene::SceneNode* rigidBodyNode)
    : RigidBase(rigidBodyNode) {}

bool RigidStage::initialize(const assets::ResourceManager& resMgr,
                            const std::string& handle) {
  if (initializationAttributes_ != nullptr) {
    LOG(ERROR) << "Cannot initialize a RigidStage more than once";
    return false;
  }
  objectMotionType_ = MotionType::STATIC;
  initializationAttributes_ =
      resMgr.getStageAttributesManager()->getTemplateCopyByHandle(handle);

  return initialization_LibSpecific(resMgr);
}

}  // namespace physics
}  // namespace esp
