// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
#define ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_

#include "ManagedPhysicsObjectBase.h"
#include "esp/physics/ArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for ArticulatedObject constructions.
 * Provides bindings for all ArticulatedObject-specific functionality.
 */
class ManagedArticulatedObject
    : public esp::physics::AbstractManagedPhysicsObject<
          esp::physics::ArticulatedObject> {
 public:
  explicit ManagedArticulatedObject(
      const std::string& classKey = "ManagedArticulatedObject")
      : AbstractManagedPhysicsObject<esp::physics::ArticulatedObject>(
            classKey) {}

  int createJointMotor(const int dof,
                       const JointMotorSettings& settings) const {
    if (auto sp = getObjectReference()) {
      return sp->createJointMotor(dof, settings);
    }
    return ID_UNDEFINED;
  }

  void removeJointMotor(const int motorId) {
    if (auto sp = getObjectReference()) {
      sp->removeJointMotor(motorId);
    }
  }

  JointMotorSettings getJointMotorSettings(const int motorId) const {
    if (auto sp = getObjectReference()) {
      return sp->getJointMotorSettings(motorId);
    }
    return {};
  }

  void updateJointMotor(const int motorId, const JointMotorSettings& settings) {
    if (auto sp = getObjectReference()) {
      sp->updateJointMotor(motorId, settings);
    }
  }

  scene::SceneNode* getLinkSceneNode(int linkId = -1) const {
    if (auto sp = getObjectReference()) {
      return &const_cast<scene::SceneNode&>(sp->getLinkSceneNode(linkId));
    }
    return nullptr;
  }

  std::vector<scene::SceneNode*> getLinkVisualSceneNodes(
      int linkId = -1) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkVisualSceneNodes(linkId);
    }
    return {};
  }

  ArticulatedLink* getLink(int linkId) const {
    if (auto sp = getObjectReference()) {
      return &sp->getLink(linkId);
    }
    return nullptr;
  }
  int getNumLinks() const {
    if (auto sp = getObjectReference()) {
      return sp->getNumLinks();
    }
    return -1;
  }

  std::vector<int> getLinkIds() const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkIds();
    }
    return {};
  }

  void setForces(const std::vector<float>& forces) {
    if (auto sp = getObjectReference()) {
      sp->setForces(forces);
    }
  }

  std::vector<float> getForces() {
    if (auto sp = getObjectReference()) {
      return sp->getForces();
    }
    return {};
  }

  void setVelocities(const std::vector<float>& vels) {
    if (auto sp = getObjectReference()) {
      sp->setVelocities(vels);
    }
  }

  std::vector<float> getVelocities() {
    if (auto sp = getObjectReference()) {
      return sp->getVelocities();
    }
    return {};
  }

  void setPositions(const std::vector<float>& positions) {
    if (auto sp = getObjectReference()) {
      sp->setPositions(positions);
    }
  }

  std::vector<float> getPositions() {
    if (auto sp = getObjectReference()) {
      return sp->getPositions();
    }
    return {};
  }

  std::vector<float> getPositionLimits(bool upperLimits = false) {
    if (auto sp = getObjectReference()) {
      return sp->getPositionLimits(upperLimits);
    }
    return {};
  }

  void addArticulatedLinkForce(int linkId, Magnum::Vector3 force) {
    if (auto sp = getObjectReference()) {
      sp->addArticulatedLinkForce(linkId, force);
    }
  }

  float getArticulatedLinkFriction(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getArticulatedLinkFriction(linkId);
    }
    return 0.0;
  }

  void setArticulatedLinkFriction(int linkId, float friction) {
    if (auto sp = getObjectReference()) {
      sp->setArticulatedLinkFriction(linkId, friction);
    }
  }

  void reset() {
    if (auto sp = getObjectReference()) {
      sp->reset();
    }
  }

  bool getCanSleep() const {
    if (auto sp = getObjectReference()) {
      return sp->getCanSleep();
    }
    return false;
  }

  std::map<int, int> getExistingJointMotors() const {
    if (auto sp = getObjectReference()) {
      return sp->getExistingJointMotors();
    }
    return {};
  }

  std::map<int, int> createMotorsForAllDofs(JointMotorSettings settings) {
    if (auto sp = getObjectReference()) {
      return sp->createMotorsForAllDofs(settings);
    }
    return {};
  }

  void setAutoClampJointLimits(bool autoClamp) {
    if (auto sp = getObjectReference()) {
      sp->setAutoClampJointLimits(autoClamp);
    }
  }

  bool getAutoClampJointLimits() const {
    if (auto sp = getObjectReference()) {
      return sp->getAutoClampJointLimits();
    }
    return false;
  }

  void clampJointLimits() {
    if (auto sp = getObjectReference()) {
      sp->clampJointLimits();
    }
  }

 public:
  ESP_SMART_POINTERS(ManagedArticulatedObject)
};  // namespace physics

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
