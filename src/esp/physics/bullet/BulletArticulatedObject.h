// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_BULLET_BULLETARTICULATEDOBJECT_H_
#define ESP_PHYSICS_BULLET_BULLETARTICULATEDOBJECT_H_

/** @file
 * @brief Class @ref esp::physics::BulletArticulatedObject
 */

#include <utility>

#include "../ArticulatedObject.h"
#include "BulletBase.h"
#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodySphericalJointMotor.h"
#include "BulletURDFImporter.h"

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
        BulletBase(std::move(bWorld), std::move(collisionObjToObjIds)) {}

  Magnum::Range3D getCollisionShapeAabb() const override {
    // TODO: collision object should be linked here
    return Magnum::Range3D();
  }

  //! link can't do this.
  void setMotionType(CORRADE_UNUSED MotionType mt) override {}

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
      : ArticulatedObject(rootNode, resMgr, objectId),
        bWorld_(std::move(bWorld)) {
    objectMotionType_ = MotionType::DYNAMIC;
    collisionObjToObjIds_ = std::move(collisionObjToObjIds);
  }

  ~BulletArticulatedObject() override;

  bool initializeFromURDF(URDFImporter& u2b,
                          const Magnum::Matrix4& worldTransform,
                          gfx::DrawableGroup* drawables,
                          scene::SceneNode* physicsNode,
                          bool fixedBase = false) override;

  // update the SceneNode state to match the simulation state
  void updateNodes(bool force = false) override;

  Mn::Vector3 getRootLinearVelocity() const override;
  void setRootLinearVelocity(const Mn::Vector3& linVel) override;

  Mn::Vector3 getRootAngularVelocity() const override;
  void setRootAngularVelocity(const Mn::Vector3& angVel) override;

  void setForces(const std::vector<float>& forces) override;
  void addForces(const std::vector<float>& forces) override;

  std::vector<float> getForces() override;

  void setVelocities(const std::vector<float>& vels) override;

  std::vector<float> getVelocities() override;

  void setPositions(const std::vector<float>& positions) override;

  std::vector<float> getPositions() override;

  std::vector<float> getPositionLimits(bool upperLimits = false) override;

  void addArticulatedLinkForce(int linkId, Magnum::Vector3 force) override;

  //! get the coefficient of friction for a link's collision objects
  float getArticulatedLinkFriction(int linkId) override;

  //! set the coefficient of friction for a link's collision objects
  void setArticulatedLinkFriction(int linkId, float friction) override;

  JointType getLinkJointType(CORRADE_UNUSED int linkId) const override;

  int getLinkDoFOffset(CORRADE_UNUSED int linkId) const override;

  int getLinkNumDoFs(CORRADE_UNUSED int linkId) const override;

  int getLinkJointPosOffset(CORRADE_UNUSED int linkId) const override;

  int getLinkNumJointPos(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief reset the articulated object state by clearing forces and zeroing
   * positions and velocities.
   */
  void reset() override;

  void setActive(bool active) override;

  bool isActive() const override;

  bool getCanSleep() override;

  void setMotionType(MotionType mt) override;

  /**
   * @brief Return result of a discrete contact test between the object and
   * collision world.
   *
   * See @ref SimulationContactResultCallback
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  bool contactTest() override;

  //! Bullet supports vel/pos control joint motors for revolute and prismatic
  //! joints (1 Dof) This is the suggested way to implement friction/damping at
  //! dof level
  bool supportsJointMotor(int linkIx) const;

  //============ Joint Motor Constraints =============

  /**
   * @brief Create a new JointMotor from a JointMotorSettings.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   * @param index DoF (for revolute or prismatic joints) or Link (spherical
   * joints)
   * @param settings The settings for the joint motor. Must have JointMotorType
   * correctly configured.
   * @return The motorId for the new joint motor or ID_UNDEFINED (-1) if failed.
   */
  int createJointMotor(const int dof,
                       const JointMotorSettings& settings) override;

  void removeJointMotor(const int motorId) override;
  void updateJointMotor(const int motorId,
                        const JointMotorSettings& settings) override;

  std::map<int, int> createMotorsForAllDofs(
      JointMotorSettings settings = JointMotorSettings()) override;

  float getJointMotorMaxImpulse(int motorId);

  //! clamp current pose to joint limits
  void clampJointLimits() override;

  /**
   * @brief Set or reset the articulated object's state using the object's
   * specified @p sceneAOInstanceAttributes_ (down cast in method).
   * @param defaultCOMCorrection Not used in AO currently. The default value of
   * whether COM-based translation correction needs to occur.
   */
  void resetStateFromSceneInstanceAttr(
      CORRADE_UNUSED bool defaultCOMCorrection = false) override;

  // std::unique_ptr<btMultiBody> btMultiBody_; //TODO:
  // TODO: also protected? not due to p2p constraint system
  std::unique_ptr<btMultiBody> btMultiBody_;

 protected:
  /**
   * @brief Called internally from syncPose()  Used to update physics
   * constructs when kinematic transformations are performed manually.  See @ref
   * esp::physics::PhysicsObjectBase for the transformations.
   */
  void setRootState(const Magnum::Matrix4& state) override;

  bool attachGeometry(ArticulatedLink* linkObject,
                      const std::shared_ptr<io::URDF::Link>& link,
                      gfx::DrawableGroup* drawables) override;

  /**
   * @brief Called internally.  Version specific to Bullet setup to simplify the
   * creation process.
   * @param linkIx link index to use for link and link's parent bodies, between
   * which to put joint.
   * @param linkDof link DOF index corresponding to the current DOF within the
   * current link being attached to a motor.
   * @param globalDof index in DOF-based array of motor IDs corresponding to
   * this motor.
   * @param settings the @ref esp::physics::JointMotorSettings values that
   * describe the desired motor's various parameters.
   * @return index of created joint motor in @p jointMotors_ map.
   */
  int createJointMotorInternal(const int linkIx,
                               const int linkDof,
                               const int globalDof,
                               const JointMotorSettings& settings);

  //! Performs forward kinematics, updates collision object states and
  //! broadphase aabbs for the object. Do this with manual state setters.
  void updateKinematicState();

  int nextJointMotorId_ = 0;

  std::map<int, std::unique_ptr<btMultiBodyJointMotor>> articulatedJointMotors;
  std::map<int, std::unique_ptr<btMultiBodySphericalJointMotor>>
      articulatedSphericalJointMotors;

  //! maps local link id to parent joint's limit constraint
  std::map<int, JointLimitConstraintInfo> jointLimitConstraints;

  // scratch datastrcutures for updateKinematicState
  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  std::unique_ptr<btCompoundShape> bFixedObjectShape_;
  std::unique_ptr<btRigidBody> bFixedObjectRigidBody_;

  // TODO: should be stored in the link
  // compound parent collision shapes for the links
  std::map<int, std::unique_ptr<btCompoundShape>> linkCompoundShapes_;

  // child mesh convex and primitive shapes for the link compound shapes
  std::map<int, std::vector<std::unique_ptr<btCollisionShape>>>
      linkChildShapes_;

  // used to update raycast objectId checks (maps to link ids)
  std::shared_ptr<std::map<const btCollisionObject*, int>>
      collisionObjToObjIds_;

  ESP_SMART_POINTERS(BulletArticulatedObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_BULLET_BULLETARTICULATEDOBJECT_H_
