// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidStage.h"

namespace esp {
namespace physics {

RigidStage::RigidStage(scene::SceneNode* rigidBodyNode,
                       const assets::ResourceManager& resMgr)
    : RigidBase(rigidBodyNode, ID_UNDEFINED, resMgr) {}

bool RigidStage::initialize(
    metadata::attributes::AbstractObjectAttributes::ptr initAttributes) {
  if (initializationAttributes_ != nullptr) {
    ESP_ERROR() << "Cannot initialize a RigidStage more than once";
    return false;
  }
  objectMotionType_ = MotionType::STATIC;
  objectName_ = Cr::Utility::formatString(
      "Stage from {}", initAttributes->getSimplifiedHandle());
  // save the copy of the template used to create the object at initialization
  // time
  setUserAttributes(initAttributes->getUserConfiguration());
  initializationAttributes_ = std::move(initAttributes);

  return initialization_LibSpecific();
}

}  // namespace physics
}  // namespace esp
