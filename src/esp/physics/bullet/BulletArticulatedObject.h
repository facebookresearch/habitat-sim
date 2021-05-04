// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::BulletArticulatedObject
 */

#include <utility>

#include "../ArticulatedObject.h"
#include "BulletBase.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"

namespace esp {

namespace physics {

// forward delcaration from BulletURDFImporter
struct JointLimitConstraintInfo;

////////////////////////////////////
// Link
////////////////////////////////////

class BulletArticulatedLink : public ArticulatedLink, public BulletBase {
 public:
  BulletArticulatedLink(scene::SceneNode* bodyNode,
                        const assets::ResourceManager& resMgr,
                        std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
                        int index,
                        std::shared_ptr<std::map<const btCollisionObject*, int>>
                            collisionObjToObjIds)
      : ArticulatedLink(bodyNode, index, resMgr),
        BulletBase(std::move(bWorld), std::move(collisionObjToObjIds)){};

  virtual const Magnum::Range3D getCollisionShapeAabb() const override {
    // TODO: collision object should be linked here
    return Magnum::Range3D();
  };

  //! link can't do this.
  virtual void setMotionType(CORRADE_UNUSED MotionType mt) override{};

 protected:
  int mbIndex_;

 private:
  ESP_SMART_POINTERS(BulletArticulatedLink)
};

////////////////////////////////////
// Articulated Object
////////////////////////////////////

class BulletArticulatedObject : public ArticulatedObject {
 public:
  BulletArticulatedObject(
      scene::SceneNode* rootNode,
      assets::ResourceManager& resMgr,
      int objectId,
      std::shared_ptr<btMultiBodyDynamicsWorld> bWorld,
      std::shared_ptr<std::map<const btCollisionObject*, int>>
          collisionObjToObjIds)
      : bWorld_(std::move(bWorld)),
        ArticulatedObject(rootNode, resMgr, objectId) {
    motionType_ = MotionType::DYNAMIC;
    collisionObjToObjIds_ = std::move(collisionObjToObjIds);
  };

  virtual ~BulletArticulatedObject();

  virtual bool initializeFromURDF(URDFImporter& u2b,
                                  const Magnum::Matrix4& worldTransform,
                                  gfx::DrawableGroup* drawables,
                                  scene::SceneNode* physicsNode,
                                  bool fixedBase = false) override;

  virtual Magnum::Matrix4 getRootState() override;

  // update the SceneNode state to match the simulation state
  virtual void updateNodes(bool force = false) override;

  virtual void setRootState(const Magnum::Matrix4& state) override;

  virtual void setForces(const std::vector<float>& forces) override;

  virtual std::vector<float> getForces() override;

  virtual void setVelocities(const std::vector<float>& vels) override;

  virtual std::vector<float> getVelocities() override;

  virtual void setPositions(const std::vector<float>& positions) override;

  virtual std::vector<float> getPositions() override;

  virtual std::vector<float> getPositionLimits(
      bool upperLimits = false) override;

  virtual void addArticulatedLinkForce(int linkId,
                                       Magnum::Vector3 force) override;

  //! get the coefficient of friction for a link's collision objects
  virtual float getArticulatedLinkFriction(int linkId) override;

  //! set the coefficient of friction for a link's collision objects
  virtual void setArticulatedLinkFriction(int linkId, float friction) override;

  /**
   * @brief reset the articulated rigid body to 0 velocities and positions.
   */
  virtual void reset() override;

  virtual void setSleep(bool sleep) override;

  virtual bool getSleep() override;

  virtual bool getCanSleep() override;

  virtual void setMotionType(MotionType mt) override;

  /**
   * @brief Return result of a discrete contact test between the object and
   * collision world.
   *
   * See @ref SimulationContactResultCallback
   * @param staticAsStage When false, override configured collision groups|masks
   * for articulated fixed base such that contact with other STATICs such as the
   * stage are considered.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest(bool staticAsStage = true);

  //! Bullet supports vel/pos control joint motors for revolute and prismatic
  //! joints (1 Dof) This is the suggested way to implement friction/damping at
  //! dof level
  bool supportsJointMotor(int linkIx);

  // TODO: should be stored in the link
  std::map<int, std::unique_ptr<btCollisionShape>> linkCollisionShapes_;

  // used to update raycast objectId checks (maps to link ids)
  std::shared_ptr<std::map<const btCollisionObject*, int>>
      collisionObjToObjIds_;

  // std::unique_ptr<btMultiBody> btMultiBody_; //TODO:
  // TODO: also protected? not due to p2p constraint system
  std::unique_ptr<btMultiBody> btMultiBody_;

  //============ Joint Motor Constraints =============

  virtual int createJointMotor(const int dof,
                               const JointMotorSettings& settings) override;

  //! internal version specific to Bullet setup to simplify the creation
  //! process.
  int createJointMotor(const int linkIx,
                       const int linkDof,
                       const int globalDof,
                       const JointMotorSettings& settings);

  virtual void removeJointMotor(const int motorId) override;
  virtual void updateJointMotor(const int motorId,
                                const JointMotorSettings& settings) override;

  virtual std::map<int, int> createMotorsForAllDofs(
      JointMotorSettings settings = JointMotorSettings()) override;

  float getJointMotorMaxImpulse(int motorId);

  int nextJointMotorId_ = 0;

  std::map<int, std::unique_ptr<btMultiBodyJointMotor>> articulatedJointMotors;

  //! maps local link id to parent joint's limit constraint
  std::map<int, JointLimitConstraintInfo> jointLimitConstraints;

  //! clamp current pose to joint limits
  void clampJointLimits() override;

 protected:
  virtual bool attachGeometry(
      ArticulatedLink* linkObject,
      const std::shared_ptr<io::URDF::Link>& link,
      const std::map<std::string, std::shared_ptr<io::URDF::Material>>&
          materials,
      gfx::DrawableGroup* drawables) override;

  //! Performs forward kinematics, updates collision object states and
  //! broadphase aabbs for the object. Do this with manual state setters.
  void updateKinematicState();

  // scratch datastrcutures for updateKinematicState
  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  std::unique_ptr<btCompoundShape> bFixedObjectShape_;
  std::unique_ptr<btRigidBody> bFixedObjectRigidBody_;

  ESP_SMART_POINTERS(BulletArticulatedObject)
};

}  // namespace physics
}  // namespace esp
