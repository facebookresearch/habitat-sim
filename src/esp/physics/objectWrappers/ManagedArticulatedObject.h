// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

  float getGlobalScale() const {
    if (auto sp = getObjectReference()) {
      return sp->getGlobalScale();
    }
    return 1.0;
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

  std::vector<int> getLinkIdsWithBase() const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkIdsWithBase();
    }
    return {};
  }

  std::unordered_map<int, int> getLinkObjectIds() const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkObjectIds();
    }
    return {};
  }

  void setRootLinearVelocity(const Mn::Vector3& linVel) {
    if (auto sp = getObjectReference()) {
      sp->setRootLinearVelocity(linVel);
    }
  }
  Mn::Vector3 getRootLinearVelocity() const {
    if (auto sp = getObjectReference()) {
      return sp->getRootLinearVelocity();
    }
    return Mn::Vector3(0);
  }
  void setRootAngularVelocity(const Mn::Vector3& angVel) {
    if (auto sp = getObjectReference()) {
      sp->setRootAngularVelocity(angVel);
    }
  }
  Mn::Vector3 getRootAngularVelocity() const {
    if (auto sp = getObjectReference()) {
      return sp->getRootAngularVelocity();
    }
    return Mn::Vector3(0);
  }

  void setJointForces(const std::vector<float>& forces) {
    if (auto sp = getObjectReference()) {
      sp->setJointForces(forces);
    }
  }

  void addJointForces(const std::vector<float>& forces) {
    if (auto sp = getObjectReference()) {
      sp->addJointForces(forces);
    }
  }

  std::vector<float> getJointForces() {
    if (auto sp = getObjectReference()) {
      return sp->getJointForces();
    }
    return {};
  }

  void setJointVelocities(const std::vector<float>& vels) {
    if (auto sp = getObjectReference()) {
      sp->setJointVelocities(vels);
    }
  }

  std::vector<float> getJointVelocities() {
    if (auto sp = getObjectReference()) {
      return sp->getJointVelocities();
    }
    return {};
  }

  void setJointPositions(const std::vector<float>& positions) {
    if (auto sp = getObjectReference()) {
      sp->setJointPositions(positions);
    }
  }

  std::vector<float> getJointPositions() {
    if (auto sp = getObjectReference()) {
      return sp->getJointPositions();
    }
    return {};
  }

  std::vector<float> getJointMotorTorques(double fixedTimeStep) {
    if (auto sp = getObjectReference()) {
      return sp->getJointMotorTorques(fixedTimeStep);
    }
    return {};
  }

  std::pair<std::vector<float>, std::vector<float>> getJointPositionLimits() {
    if (auto sp = getObjectReference()) {
      return sp->getJointPositionLimits();
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

  JointType getLinkJointType(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkJointType(linkId);
    }
    return JointType::Invalid;
  }

  std::string getLinkJointName(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkJointName(linkId);
    }
    return "";
  }

  std::string getLinkName(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkName(linkId);
    }
    return "";
  }

  int getLinkDoFOffset(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkDoFOffset(linkId);
    }
    return -1;
  }

  int getLinkNumDoFs(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkNumDoFs(linkId);
    }
    return 0;
  }

  int getLinkJointPosOffset(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkJointPosOffset(linkId);
    }
    return -1;
  }

  int getLinkNumJointPos(int linkId) const {
    if (auto sp = getObjectReference()) {
      return sp->getLinkNumJointPos(linkId);
    }
    return 0;
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

  std::unordered_map<int, int> getExistingJointMotors() const {
    if (auto sp = getObjectReference()) {
      return sp->getExistingJointMotors();
    }
    return {};
  }

  std::unordered_map<int, int> createMotorsForAllDofs(
      const JointMotorSettings& settings) {
    if (auto sp = getObjectReference()) {
      return sp->createMotorsForAllDofs(settings);
    }
    return {};
  }

  void updateAllMotorTargets(const std::vector<float>& stateTargets,
                             bool velocities) {
    if (auto sp = getObjectReference()) {
      sp->updateAllMotorTargets(stateTargets, velocities);
    }
  }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   * TODO : once Magnum supports retrieving key-values of configurations, use
   * that to build this data.
   */

  std::string getPhyObjInfoHeaderInternal() const override {
    // TODO fill out appropriate reporting values
    return "# links";
  }

  /**
   * @brief Specialization-specific extension of getObjectInfo, comma separated
   * info ideal for saving to csv
   */
  std::string getPhysObjInfoInternal(
      std::shared_ptr<esp::physics::ArticulatedObject>& sp) const override {
    // TODO fill out appropriate reporting values
    return std::to_string(sp->getNumLinks());
  }

 public:
  ESP_SMART_POINTERS(ManagedArticulatedObject)
};  // namespace physics

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDARTICULATEDOBJECT_H_
