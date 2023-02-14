// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "RigidObject.h"

namespace esp {
namespace physics {

RigidObject::RigidObject(scene::SceneNode* rigidBodyNode,
                         int objectId,
                         const assets::ResourceManager& resMgr)
    : RigidBase(rigidBodyNode, objectId, resMgr),
      velControl_(VelocityControl::create()) {}

bool RigidObject::initialize(
    metadata::attributes::AbstractObjectAttributes::ptr initAttributes) {
  if (initializationAttributes_ != nullptr) {
    ESP_ERROR() << "Cannot initialize a RigidObject more than once";
    return false;
  }

  // save the copy of the template used to create the object at initialization
  // time
  setUserAttributes(initAttributes->getUserConfiguration());
  initializationAttributes_ = std::move(initAttributes);

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

void RigidObject::setMotionType(MotionType mt) {
  if (mt != MotionType::DYNAMIC) {
    // can't set DYNAMIC without a dynamics engine.
    objectMotionType_ = mt;
  }
}

void RigidObject::resetStateFromSceneInstanceAttr() {
  auto sceneInstanceAttr = getInitObjectInstanceAttr();
  if (!sceneInstanceAttr) {
    return;
  }
  // set object's location and rotation based on translation and rotation
  // params specified in instance attributes
  auto translate = sceneInstanceAttr->getTranslation();
  auto rotation = sceneInstanceAttr->getRotation();
  // This was set when object was created, based on whether or not the object
  // should be centered at COM or via Asset Local origin.
  if (isCOMCorrected_) {
    // if default COM correction is set and no object-based override, or if
    // Object set to correct for COM.
    translate -= rotation.transformVector(visualNode_->translation());
  }
  setTranslation(translate);
  setRotation(rotation);

  // set object's motion type if different than set value
  const physics::MotionType attrObjMotionType =
      static_cast<physics::MotionType>(sceneInstanceAttr->getMotionType());
  if (attrObjMotionType != physics::MotionType::UNDEFINED) {
    this->setMotionType(attrObjMotionType);
  }
}  // RigidObject::resetStateFromSceneInstanceAttr

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
