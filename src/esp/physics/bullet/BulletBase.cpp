// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/physics/bullet/BulletBase.h"

namespace esp {
namespace physics {

BulletBase::BulletBase(scene::SceneNode* rigidBodyNode,
                       std::shared_ptr<btMultiBodyDynamicsWorld> bWorld)
    : MotionState(*rigidBodyNode), bWorld_(bWorld) {}

BulletBase::~BulletBase() {
  bWorld_.reset();
}

}  // namespace physics
}  // namespace esp