// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObject.h"

namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* rigidBodyNode)
    : RigidBase(rigidBodyNode), velControl_(VelocityControl::create()) {}

bool RigidObject::initialize(
    const assets::ResourceManager& resMgr,
    const assets::AbstractPhysicsAttributes::ptr physicsAttributes) {
  if (initializationAttributes_ != nullptr) {
    LOG(ERROR) << "Cannot initialize a RigidObject more than once";
    return false;
  }

  // save a copy of the template at initialization time
  initializationAttributes_ = esp::assets::PhysicsObjectAttributes::create(
      *(static_cast<esp::assets::PhysicsObjectAttributes*>(
          physicsAttributes.get())));

  return initializationFinalize(resMgr);
}

bool RigidObject::initializationFinalize(const assets::ResourceManager&) {
  // default kineamtic unless a simulator is initialized...
  objectMotionType_ = MotionType::KINEMATIC;
  return true;
}

bool RigidObject::setMotionType(MotionType mt) {
  if (mt != MotionType::DYNAMIC) {
    objectMotionType_ = mt;
    return true;
  } else {
    return false;  // can't set DYNAMIC without a dynamics engine.
  }
}

//////////////////
// VelocityControl

core::RigidState VelocityControl::integrateTransform(
    const float dt,
    const core::RigidState& rigidState) {
  core::RigidState newRigidState(rigidState);
  // linear first
  if (controllingLinVel) {
    if (linVelIsLocal) {
      newRigidState.translation =
          rigidState.translation +
          rigidState.rotation.transformVector(linVel * dt);
    } else {
      newRigidState.translation = rigidState.translation + (linVel * dt);
    }
  }

  // then angular
  if (controllingAngVel) {
    Magnum::Vector3 globalAngVel{angVel};
    if (angVelIsLocal) {
      globalAngVel = rigidState.rotation.transformVector(angVel);
    }
    Magnum::Quaternion q = Magnum::Quaternion::rotation(
        Magnum::Rad{(globalAngVel * dt).length()}, globalAngVel.normalized());
    newRigidState.rotation = (q * rigidState.rotation).normalized();
  }
  return newRigidState;
}

}  // namespace physics
}  // namespace esp
