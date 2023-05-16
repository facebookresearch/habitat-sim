// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

// forward declaration from BulletURDFImporter
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
    ESP_WARNING() << "Not implemented.";
    return Magnum::Range3D();
  }

  //! link can't do this.
  void setMotionType(CORRADE_UNUSED MotionType mt) override {
    ESP_WARNING() << "Cannot set MotionType individually for links.";
  }

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

  /**
   * @brief Initialize this ArticulatedObject from a parsed URDF stored in a
   * URDFImporter. Creates a btMultiBody.
   *
   * @param u2b The BulletURDFImporter which will initialize this object from a
   * parsed URDF file.
   * @param worldTransform Desired global root state of the ArticulatedObject.
   * @param drawables DrawableGroup to which this object's visual shapes will be
   * added.
   * @param physicsNode The parent node of this object.
   * @param fixedBase Whether or not the root link should be fixed or free.
   */
  void initializeFromURDF(URDFImporter& u2b,
                          const Magnum::Matrix4& worldTransform,
                          scene::SceneNode* physicsNode) override;

  /**
   * @brief Construct a Static btRigidObject to act as a proxy collision object
   * for the fixed base.
   *
   * This optimization reduces the collision island size for articulated objects
   * with heavy branching (e.g. a counter with many drawers) resulting in better
   * sleeping behavior (e.g. contact with the countertop should not activate all
   * drawers and contained objects).
   *
   * To utilize this feature, the object URDF should mark fixed link collision
   * shapes as part of the Static collision group (e.g. `<collision group="2">`)
   */
  void constructStaticRigidBaseObject();

  //! update the SceneNode state to match the simulation state
  void updateNodes(bool force = false) override;

  /**
   * @brief Return a @ref
   * metadata::attributes::SceneAOInstanceAttributes reflecting the current
   * state of this Articulated Object.
   * @return a @ref SceneAOInstanceAttributes reflecting this Articulated
   * Object's current state
   */
  std::shared_ptr<metadata::attributes::SceneAOInstanceAttributes>
  getCurrentStateInstanceAttr() override;

  /**
   * @brief Get the linear velocity of the articulated object's root in the
   * global frame.
   *
   * @return The root linear velocity.
   */
  Mn::Vector3 getRootLinearVelocity() const override;

  /**
   * @brief Set the linear velocity of the articulated object's root in the
   * global frame.
   *
   * @param linVel The root linear velocity.
   */
  void setRootLinearVelocity(const Mn::Vector3& linVel) override;

  /**
   * @brief Get the angular velocity (omega) of the articulated object's root in
   * the global frame.
   *
   * @return The root angular velocity (omega).
   */
  Mn::Vector3 getRootAngularVelocity() const override;

  /**
   * @brief Set the angular velocity (omega) of the articulated object's root in
   * the global frame.
   *
   * @param angVel The root angular velocity (omega).
   */
  void setRootAngularVelocity(const Mn::Vector3& angVel) override;

  /**
   * @brief Set forces/torques for all joints indexed by degrees of freedom.
   *
   * Bullet clears joint forces/torques with each simulation step.
   *
   * @param forces The desired joint forces/torques.
   */
  void setJointForces(const std::vector<float>& forces) override;

  /**
   * @brief Add forces/torques to all joints indexed by degrees of freedom.
   *
   * Bullet clears joint forces/torques with each simulation step.
   *
   * @param forces The desired joint forces/torques to add.
   */
  void addJointForces(const std::vector<float>& forces) override;

  /**
   * @brief Get current forces/torques for all joints indexed by degrees of
   * freedom.
   *
   * Bullet clears joint forces/torques with each simulation step.
   *
   * @return The current joint forces/torques.
   */
  std::vector<float> getJointForces() override;

  /**
   * @brief Set velocities for all joints indexed by degrees of freedom.
   *
   * @param vels The desired joint velocities.
   */
  void setJointVelocities(const std::vector<float>& vels) override;

  /**
   * @brief Get current velocities for all joints indexed by degrees of freedom.
   *
   * @return The current joint velocities.
   */
  std::vector<float> getJointVelocities() override;

  /**
   * @brief Set positions for all joints.
   *
   * Note that positions are not indexed by DoF because some joints (spherical)
   * have a different number of DoFs from positions. For spherical joints, a
   * block of 4 position values should specify the state as a unit quaternion (x
   * y z w).
   *
   * @param positions The desired joint positions.
   */
  void setJointPositions(const std::vector<float>& positions) override;

  /**
   * @brief Get positions for all joints.
   *
   * Note that positions are not indexed by DoF because some joints (spherical)
   * have a different number of DoFs from positions. For spherical joints, a
   * block of 4 position values should specify the state as a unit quaternion (x
   * y z w).
   *
   * @return The current joint positions.
   */
  std::vector<float> getJointPositions() override;

  /**
   * @brief Get the torques on each joint
   *
   * @param fixedTimeStep The physics timestep used by the simulator.  Necessary
   * to convert impulse into torque.
   *
   * @return Array of torques on each joint
   */
  std::vector<float> getJointMotorTorques(double fixedTimeStep) override;

  /**
   * @brief Get position limits for all joints. (lower, upper)
   *
   * @return The active joint position limits as a pair of vectors (lower,
   * upper). When no limit is set, entry is inf or -inf.
   */
  std::pair<std::vector<float>, std::vector<float>> getJointPositionLimits()
      override;

  /**
   * @brief Add linear force to a link's COM specified in the global frame.
   *
   * @param linkId The link's index.
   * @param force The desired force to add.
   */
  void addArticulatedLinkForce(int linkId, Magnum::Vector3 force) override;

  /**
   * @brief Get the friction coefficient for a link.
   *
   * @param linkId The link's index.
   * @return The link's friction coefficient.
   */
  float getArticulatedLinkFriction(int linkId) override;

  /**
   * @brief Set the friction coefficient for a link.
   *
   * @param linkId The link's index.
   * @param friction The link's friction coefficient.
   */
  void setArticulatedLinkFriction(int linkId, float friction) override;

  /**
   * @brief Get the type of the link's parent joint.
   *
   * @param linkId The link's index.
   * @return The link's parent joint's type.
   */
  JointType getLinkJointType(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief Get the starting position for this link's parent joint in the global
   * DoFs array.
   *
   * @param linkId The link's index.
   * @return The link's starting DoF index.
   */
  int getLinkDoFOffset(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief Get the number of DoFs for this link's parent joint.
   *
   * @param linkId The link's index.
   * @return The number of DoFs for this link's parent joint.
   */
  int getLinkNumDoFs(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief Get the starting position for this link's parent joint in the global
   * positions array.
   *
   * @param linkId The link's index.
   * @return The link's starting position index.
   */
  int getLinkJointPosOffset(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief Get the number of positions for this link's parent joint.
   *
   * @param linkId The link's index.
   * @return The number of positions for this link's parent joint.
   */
  int getLinkNumJointPos(CORRADE_UNUSED int linkId) const override;

  /**
   * @brief reset the articulated object state by clearing forces and zeroing
   * positions and velocities.
   */
  void reset() override;

  /**
   * @brief Set an object as being actively simulated or sleeping.
   *
   * @param active Whether to activate or sleep the object
   */
  void setActive(bool active) override;

  /**
   * @brief Check whether object is being actively simulated, or sleeping.
   *
   * @return true if active, false otherwise.
   */
  bool isActive() const override;

  /**
   * @brief Check if this object can be de-activated (i.e. sleep).
   *
   * @return Whether or not the object is able to deactivate.
   */
  bool getCanSleep() override;

  /**
   * @brief Set the @ref MotionType of the object.
   *
   * Dynamic state is integrated by physics simulator.
   * Kinematic state is manually set by user.
   * Static state should not be changed. Object can be added to NavMesh.
   *
   * @param mt The desired @ref MotionType.
   */
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

  //! clamp current pose to joint limits
  void clampJointLimits() override;

  /**
   * @brief Manually set the collision group for all links of the object.
   * @param group The desired CollisionGroup for the object.
   */
  void overrideCollisionGroup(CollisionGroup group) override;

  //============ Joint Motor Constraints =============

  //! Bullet supports vel/pos control joint motors for revolute and prismatic
  //! joints (1 Dof) This is the suggested way to implement friction/damping at
  //! dof level
  bool supportsSingleDofJointMotor(int linkIx) const;

  /**
   * @brief Create a new JointMotor from a JointMotorSettings.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   * @param linkIndex the index of the link for which the parent joint will have
   * a motor attached
   * @param settings The settings for the joint motor. Must have JointMotorType
   * correctly configured for the target joint.
   * @return The motorId for the new joint motor
   */
  int createJointMotor(int linkIndex,
                       const JointMotorSettings& settings) override;

  /**
   * @brief Remove and destroy a joint motor.
   */
  void removeJointMotor(int motorId) override;

  /**
   * @brief Update a JointMotor with new settings.
   */
  void updateJointMotor(int motorId,
                        const JointMotorSettings& settings) override;

  /**
   * @brief Create a new set of default JointMotors for all valid dofs in an
   * ArticulatedObject.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @return A map of motorIds to link/joint indices for the new motors.
   */
  std::unordered_map<int, int> createMotorsForAllDofs(
      const JointMotorSettings& settings = JointMotorSettings()) override;

  /**
   * @brief Update all motors targets for this object's joints which support
   * motors (Revolute, Prismatic, Spherical) from a state array.
   *
   * By default, state is interpreted as position targets unless `velocities` is
   * specified. Expected input is the full length position or velocity array for
   * this object. This function will safely skip states for joints which don't
   * support JointMotors.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @param stateTargets Full length joint position or velocity array for this
   * object.
   * @param velocities Whether to interpret stateTargets as velocities or
   * positions.
   */
  void updateAllMotorTargets(const std::vector<float>& stateTargets,
                             bool velocities = false) override;

  //============ END - Joint Motor Constraints =============

  /**
   * @brief Set or reset the articulated object's state using the object's
   * specified @p sceneAOInstanceAttributes_ (down cast in method).
   * @param defaultCOMCorrection Not used in AO currently. The default value of
   * whether COM-based translation correction needs to occur.
   */
  void resetStateFromSceneInstanceAttr() override;

  //! The Bullet multibody structure
  std::unique_ptr<btMultiBody> btMultiBody_;

 protected:
  /**
   * @brief Called internally from syncPose()  Used to update physics
   * constructs when kinematic transformations are performed manually.  See @ref
   * esp::physics::PhysicsObjectBase for the transformations.
   */
  void setRootState(const Magnum::Matrix4& state) override;

  //! Performs forward kinematics, updates collision object states and
  //! broadphase aabbs for the object. Do this with manual state setters.
  void updateKinematicState();

  int nextJointMotorId_ = 0;

  std::unordered_map<int, std::unique_ptr<btMultiBodyJointMotor>>
      articulatedJointMotors;
  std::unordered_map<int, std::unique_ptr<btMultiBodySphericalJointMotor>>
      articulatedSphericalJointMotors;

  //! maps local link id to parent joint's limit constraint
  std::unordered_map<int, JointLimitConstraintInfo> jointLimitConstraints;

  // scratch data structures for updateKinematicState
  btAlignedObjectArray<btQuaternion> scratch_q_;
  btAlignedObjectArray<btVector3> scratch_m_;

  //! The containing Bullet world
  std::shared_ptr<btMultiBodyDynamicsWorld> bWorld_;

  //! The collision shape for the fixed base rigid object
  std::unique_ptr<btCompoundShape> bFixedObjectShape_;

  //! A proxy rigid object to replace fixed base links as a perf optimization.
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
