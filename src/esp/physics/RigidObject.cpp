// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObject.h"

namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* rigidBodyNode,
                         int objectId,
                         const assets::ResourceManager& resMgr)
    : RigidBase(rigidBodyNode, resMgr), velControl_(VelocityControl::create()) {
  objectId_ = objectId;
}

bool RigidObject::initialize(const std::string& handle) {
  if (initializationAttributes_ != nullptr) {
    LOG(ERROR) << "Cannot initialize a RigidObject more than once";
    return false;
  }

  // save a copy of the template at initialization time
  initializationAttributes_ =
      resMgr_.getObjectAttributesManager()->getObjectCopyByHandle(handle);

  return initialization_LibSpecific();
}  // RigidObject::initialize

bool RigidObject::finalizeObject() {
  node().computeCumulativeBB();

  // cast initialization attributes
  metadata::attributes::ObjectAttributes::cptr ObjectAttributes =
      std::dynamic_pointer_cast<const metadata::attributes::ObjectAttributes>(
          initializationAttributes_);

  if (!ObjectAttributes->getComputeCOMFromShape()) {
    // will be false if the COM is provided; shift by that COM
    Magnum::Vector3 comShift = -ObjectAttributes->getCOM();
    // first apply scale
    comShift = ObjectAttributes->getScale() * comShift;
    shiftOrigin(comShift);
  } else {
    // otherwise use the bounding box center
    shiftOriginToBBCenter();
  }

  // set the visualization semantic id
  setSemanticId(ObjectAttributes->getSemanticId());

  // finish object by instancing any dynamics library-specific code required
  return finalizeObject_LibSpecific();
}  // RigidObject::finalizeObject

bool RigidObject::initialization_LibSpecific() {
  // default kineamtic unless a simulator is initialized...
  objectMotionType_ = MotionType::KINEMATIC;
  return true;
}  // RigidObject::initialization_LibSpecific

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
  if (controllingAngVel && angVel != Magnum::Vector3{0.0}) {
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
