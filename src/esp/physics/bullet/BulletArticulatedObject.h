// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::BulletArticulatedObject
 */

#include "../ArticulatedObject.h"
#include "BulletBase.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"

namespace esp {

namespace physics {

////////////////////////////////////
// Link
////////////////////////////////////

class BulletArticulatedLink : public ArticulatedLink, public BulletBase {
 public:
  BulletArticulatedLink(scene::SceneNode* bodyNode,
                        std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                        int index)
      : ArticulatedLink(bodyNode, index), BulletBase(bWorld){};

  int getIndex() { return mbIndex_; };

  bool isMe(const btCollisionObject* collisionObject);

  virtual const Magnum::Range3D getCollisionShapeAabb() const override {
    // TODO: collision object should be linked here
    return Magnum::Range3D();
  };

  // RigidBase overrides
  virtual bool initialize(
      const assets::ResourceManager& resMgr,
      const assets::AbstractPhysicsAttributes::ptr physicsAttributes) override {
    // TODO: this should initialize the link visual shape from a template
    return false;
  };

  //! link can't do this.
  virtual bool setMotionType(MotionType mt) override { return false; };

 protected:
  int mbIndex_;

 private:
  // RigidBase overrides
  virtual bool initializationFinalize(
      CORRADE_UNUSED const assets::ResourceManager& resMgr) override {
    return false;
  };

  ESP_SMART_POINTERS(BulletArticulatedLink)
};

////////////////////////////////////
// Articulated Object
////////////////////////////////////

class BulletArticulatedObject : public ArticulatedObject {
 public:
  BulletArticulatedObject(scene::SceneNode* rootNode,
                          std::shared_ptr<btMultiBodyDynamicsWorld> bWorld)
      : bWorld_(bWorld), ArticulatedObject(rootNode) {
    motionType_ = MotionType::DYNAMIC;
  };

  virtual ~BulletArticulatedObject();

  virtual bool initializeFromURDF(URDFImporter& u2b,
                                  const Magnum::Matrix4& worldTransform,
                                  assets::ResourceManager& resourceManager,
                                  gfx::DrawableGroup* drawables,
                                  scene::SceneNode* physicsNode,
                                  bool fixedBase = false) override;

  virtual Magnum::Matrix4 getRootState() override;

  // update the SceneNode state to match the simulation state
  virtual void updateNodes() override;

  virtual void setRootState(const Magnum::Matrix4& state) override;

  virtual void setForces(std::vector<float> forces) override;

  virtual std::vector<float> getForces() override;

  virtual void setVelocities(std::vector<float> vels) override;

  virtual std::vector<float> getVelocities() override;

  virtual void setPositions(std::vector<float> positions) override;

  virtual std::vector<float> getPositions() override;

  // TODO: This is tough in Bullet. We'll need to construct a map from link
  // indices to MultiBody constraint pointers in the world to track these...
  // virtual std::vector<float> getPositionLowerLimits() override;
  // virtual std::vector<float> getPositionUpperLimits() override;

  virtual void addArticulatedLinkForce(int linkId,
                                       Magnum::Vector3 force) override;

  /**
   * @brief reset the articulated rigid body to 0 velocities and positions.
   */
  virtual void reset() override;

  virtual void setSleep(bool sleep) override;

  virtual bool getSleep() override;

  virtual bool getCanSleep() override;

  virtual void setMotionType(MotionType mt) override;

  //! Bullet supports vel/pos control joint motors for revolute and prismatic
  //! joints (1 Dof) This is the suggested way to implement friction/damping at
  //! dof level
  bool supportsJointMotor(int linkIx);

  // TODO: should be stored in the link
  std::map<int, btCollisionShape*> linkCollisionShapes_;

  // std::unique_ptr<btMultiBody> btMultiBody_; //TODO:
  // TODO: also protected? not due to p2p constraint system
  btMultiBody* btMultiBody_;

  //============ Joint Motor Constraints =============

  //! create a new set of motors with 0 gains for all valid dofs and return a
  //! map of dof to motor id
  std::map<int, int> createMotorsForAllDofs();

  int createJointMotor(int linkId,
                       int linkDof,
                       float velocityTarget = 0.0,
                       float velGain = 1.0,
                       float positionTarget = 0.0,
                       float posGain = 0.0,
                       float maxImpulse = 1000.0);

  void updateJointMotorParams(int motorId,
                              float velocityTarget = 0.0,
                              float velGain = 1.0,
                              float positionTarget = 0.0,
                              float posGain = 0.0,
                              float maxImpulse = 1000.0);

  float getJointMotorMaxImpulse(int motorId);

  void removeJointMotor(int jointMotorId);

  int nextJointMotorId_ = 0;

  std::map<int, btMultiBodyJointMotor*> articulatedJointMotors;

 protected:
  virtual bool attachGeometry(
      scene::SceneNode& node,
      std::shared_ptr<io::UrdfLink> link,
      const std::map<std::string, std::shared_ptr<io::UrdfMaterial> >&
          materials,
      assets::ResourceManager& resourceManager,
      gfx::DrawableGroup* drawables) override;

  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  ESP_SMART_POINTERS(BulletArticulatedObject)
};

}  // namespace physics
}  // namespace esp
