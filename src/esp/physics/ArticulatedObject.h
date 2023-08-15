// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDOBJECT_H_
#define ESP_PHYSICS_ARTICULATEDOBJECT_H_

/** @file
 * @brief Class @ref esp::physics::ArticulatedLink, class @ref
 * esp::physics::ArticulatedObject, enum class @ref JointType, enum class @ref
 * JointMotorType, struct @ref JointMotorSettings
 */

#include "RigidBase.h"
#include "esp/core/Esp.h"
#include "esp/metadata/URDFParser.h"
#include "esp/metadata/attributes/ArticulatedObjectAttributes.h"
#include "esp/scene/SceneNode.h"

namespace esp {

namespace gfx {
class DrawableGroup;
}

namespace assets {
class ResourceManager;
}

namespace physics {

class URDFImporter;

//! copy of eFeatherstoneJointType from
//! bullet3/src/BulletDynamics/Featherstone/btMultiBodyLink.h for access
//! convenience w/o Bullet install.
enum class JointType : int {
  Revolute = 0,
  Prismatic = 1,
  Spherical = 2,
  Planar = 3,
  Fixed = 4,
  Invalid
};

////////////////////////////////////
// Joint Motor Interface
////////////////////////////////////

//! describes the type of a motor for generality.
enum class JointMotorType { SingleDof, Spherical };

/**
 * @brief Stores JointMotor (i.e. per-DoF PD control) parameters for creation
 * and updates.
 */
struct JointMotorSettings {
 public:
  JointMotorSettings() = default;

  //! constructor for single dof motor settings
  JointMotorSettings(double _positionTarget,
                     double _positionGain,
                     double _velocityTarget,
                     double _velocityGain,
                     double _maxImpulse)
      : positionTarget(_positionTarget),
        positionGain(_positionGain),
        velocityTarget(_velocityTarget),
        velocityGain(_velocityGain),
        maxImpulse(_maxImpulse) {}

  //! constructor for spherical motor settings
  JointMotorSettings(const Mn::Quaternion& _sphericalPositionTarget,
                     double _positionGain,
                     const Mn::Vector3& _sphericalVelocityTarget,
                     double _velocityGain,
                     double _maxImpulse)
      : motorType(JointMotorType::Spherical),
        sphericalPositionTarget(_sphericalPositionTarget),
        positionGain(_positionGain),
        sphericalVelocityTarget(_sphericalVelocityTarget),
        velocityGain(_velocityGain),
        maxImpulse(_maxImpulse) {}

  //! The type of motor parameterized by these settings. Determines which
  //! parameters to use.
  JointMotorType motorType = JointMotorType::SingleDof;

  //! Single DoF joint position target.
  double positionTarget = 0.0;

  //! Spherical joint position target.
  Mn::Quaternion sphericalPositionTarget = {};

  //! Position (proportional) gain Kp
  double positionGain = 0.0;

  //! Single DoF joint velocity target. Zero acts like joint damping/friction.
  double velocityTarget = 0.0;

  //! Spherical joint velocity target.
  Mn::Vector3 sphericalVelocityTarget = {};

  //! Velocity (derivative) gain Kd
  double velocityGain = 1.0;

  //! The maximum impulse applied by this motor. Should be tuned relative to
  //! physics timestep.
  double maxImpulse = 1000.0;

  ESP_SMART_POINTERS(JointMotorSettings)
};

/**
 * @brief A general wrapper class for JointMotor (e.g. PD control)
 * implementation.
 */
struct JointMotor {
  //! settings parameterizing the motor
  JointMotorSettings settings;

  //! link/joint index controlled by the motor
  int index;

  //! identifies the motor in internal datastructures
  int motorId;

  ESP_SMART_POINTERS(JointMotor)
};

////////////////////////////////////
// Link
////////////////////////////////////

/**
 * @brief A single rigid link in a kinematic chain. Abstract class. Feature
 * attaches to a SceneNode.
 */
class ArticulatedLink : public RigidBase {
 public:
  ArticulatedLink(scene::SceneNode* bodyNode,
                  int index,
                  const assets::ResourceManager& resMgr)
      : RigidBase(bodyNode,
                  0,  // TODO: pass an actual object ID. This is currently
                      // assigned AFTER creation.
                  resMgr),
        mbIndex_(index) {}

  ~ArticulatedLink() override = default;

  //! Get the link's index within its multibody
  int getIndex() const { return mbIndex_; }

  //! List of visual components attached to this link. Used for NavMesh
  //! recomputation. Each entry is a child node of this link's node and a string
  //! key to reference the asset in ResourceManager.
  std::vector<std::pair<esp::scene::SceneNode*, std::string>>
      visualAttachments_;

  // RigidBase overrides

  /**
   * @brief Initializes the link.
   * @param resMgr a reference to ResourceManager object
   * @param handle The handle for the template structure defining relevant
   * physical parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialize(
      CORRADE_UNUSED metadata::attributes::AbstractObjectAttributes::ptr
          initAttributes) override {
    return true;
  }

  /**
   * @brief Finalize the creation of the link.
   * @return whether successful finalization.
   */
  bool finalizeObject() override { return true; }

  void setTransformation(
      CORRADE_UNUSED const Magnum::Matrix4& transformation) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setTranslation(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setRotation(
      CORRADE_UNUSED const Magnum::Quaternion& quaternion) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void setRigidState(
      CORRADE_UNUSED const core::RigidState& rigidState) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void resetTransformation() override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void translate(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void translateLocal(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotate(CORRADE_UNUSED const Magnum::Rad angleInRad,
              CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad,
      CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateX(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateY(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateZ(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateXLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateYLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  void rotateZLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  /**
   * @brief Not used for articulated links.  Set or reset the object's state
   * using the object's specified @p sceneInstanceAttributes_.
   */
  void resetStateFromSceneInstanceAttr() override {
    ESP_DEBUG() << "ArticulatedLink can't do this.";
  }

  std::string linkName = "";
  std::string linkJointName = "";

 private:
  /**
   * @brief Finalize the initialization of this link.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override { return true; }
  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after creation.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override { return true; }

  // end RigidBase overrides

 protected:
  int mbIndex_;

 public:
  ESP_SMART_POINTERS(ArticulatedLink)
};

////////////////////////////////////
// Articulated Object
////////////////////////////////////

/**
 * @brief An articulated rigid object (i.e. kinematic chain). Abstract class to
 * be derived by physics simulator specific implementations.
 */
class ArticulatedObject : public esp::physics::PhysicsObjectBase {
 public:
  ArticulatedObject(scene::SceneNode* rootNode,
                    assets::ResourceManager& resMgr,
                    int objectId)
      : PhysicsObjectBase(rootNode, objectId, resMgr){};

  ~ArticulatedObject() override {
    // clear links and delete their SceneNodes
    std::vector<scene::SceneNode*> linkNodes;
    linkNodes.reserve(links_.size());
    for (auto& link : links_) {
      linkNodes.push_back(&link.second->node());
    }
    links_.clear();
    for (auto* node : linkNodes) {
      delete node;
    }
  };

  /**
   * @brief Get the uniform global scaling applied to this object during import.
   * @return The global scaling applied to the object.
   */
  float getGlobalScale() const { return globalScale_; }

  /**
   * @brief Get a const reference to an ArticulatedLink SceneNode for
   * info query purposes.
   * @param linkId The ArticulatedLink ID or -1 for the baseLink.
   * @return Const reference to the SceneNode.
   */
  const scene::SceneNode& getLinkSceneNode(int linkId = -1) const {
    if (linkId == ID_UNDEFINED) {
      // base link
      return baseLink_->node();
    }
    auto linkIter = links_.find(linkId);
    CORRADE_INTERNAL_ASSERT(linkIter != links_.end());
    return linkIter->second->node();
  }

  /**
   * @brief Get pointers to a link's visual SceneNodes.
   * @param linkId The ArticulatedLink ID or -1 for the baseLink.
   * @return vector of pointers to the link's visual scene nodes.
   */
  std::vector<scene::SceneNode*> getLinkVisualSceneNodes(
      int linkId = -1) const {
    if (linkId == ID_UNDEFINED) {
      // base link
      return baseLink_->visualNodes_;
    }
    auto linkIter = links_.find(linkId);
    CORRADE_INTERNAL_ASSERT(linkIter != links_.end());
    return linkIter->second->visualNodes_;
  }

  /**
   * @brief Get pointers to all visual SceneNodes associated to this
   * ArticulatedObject.
   * @return vector of pointers to base and all links' visual scene nodes.
   */
  std::vector<scene::SceneNode*> getVisualSceneNodes() const override {
    std::vector<scene::SceneNode*> allVisualNodes;
    // base link
    allVisualNodes.insert(allVisualNodes.end(), baseLink_->visualNodes_.begin(),
                          baseLink_->visualNodes_.end());
    // other links
    for (const auto& link : links_) {
      allVisualNodes.insert(allVisualNodes.end(),
                            link.second->visualNodes_.begin(),
                            link.second->visualNodes_.end());
    }
    return allVisualNodes;
  }

  /**
   * @brief Initialize this ArticulatedObject from a parsed URDF stored in a
   * URDFImporter.
   *
   * @param urdfImporter The URDFImporter which will initialize this object from
   * a parsed URDF file.
   * @param worldTransform Desired global root state of the ArticulatedObject.
   * @param drawables DrawableGroup to which this object's visual shapes will be
   * added.
   * @param physicsNode The parent node of this object.
   * @param fixedBase Whether or not the root link should be fixed or free.
   */
  virtual void initializeFromURDF(
      CORRADE_UNUSED URDFImporter& urdfImporter,
      CORRADE_UNUSED const Magnum::Matrix4& worldTransform,
      CORRADE_UNUSED scene::SceneNode* physicsNode) {
    CORRADE_INTERNAL_ASSERT_UNREACHABLE();
  };

  /**
   * @brief Get a link by index.
   *
   * @param id The id of the desired link. -1 for base link.
   * @return The desired link.
   */
  ArticulatedLink& getLink(int id) {
    // option to get the baseLink_ with id=-1
    if (id == -1) {
      return *baseLink_.get();
    }

    auto linkIter = links_.find(id);
    CORRADE_INTERNAL_ASSERT(linkIter != links_.end());
    return *linkIter->second;
  }

  /**
   * @brief Get the number of links for this object (not including the base).
   *
   * @return The number of non-base links.
   */
  int getNumLinks() const { return links_.size(); }

  /**
   * @brief Get a list of link ids, not including the base (-1).
   *
   * @return A list of link ids for this object.
   */
  std::vector<int> getLinkIds() const {
    std::vector<int> ids;
    ids.reserve(links_.size());
    for (auto it = links_.begin(); it != links_.end(); ++it) {
      ids.push_back(it->first);
    }
    return ids;
  }

  /**
   * @brief Get a list of link ids including the base (-1).
   *
   * @return A list of link ids for this object.
   */
  std::vector<int> getLinkIdsWithBase() const {
    std::vector<int> ids;
    ids.reserve(links_.size() + 1);
    ids.push_back(-1);
    for (auto it = links_.begin(); it != links_.end(); ++it) {
      ids.push_back(it->first);
    }
    return ids;
  }

  /**
   * @brief Get a map of object ids to link ids.
   *
   * @return A a map of Habitat object ids to link ids for this AO's links.
   */
  std::unordered_map<int, int> getLinkObjectIds() const {
    return objectIdToLinkId_;
  }

  /**
   * @brief Set forces/torques for all joints indexed by degrees of freedom.
   *
   * @param forces The desired joint forces/torques.
   */
  virtual void setJointForces(CORRADE_UNUSED const std::vector<float>& forces) {
  }

  /**
   * @brief Add forces/torques to all joints indexed by degrees of freedom.
   *
   * @param forces The desired joint forces/torques to add.
   */
  virtual void addJointForces(CORRADE_UNUSED const std::vector<float>& forces) {
  }

  /**
   * @brief Get current forces/torques for all joints indexed by degrees of
   * freedom.
   *
   * @return The current joint forces/torques.
   */
  virtual std::vector<float> getJointForces() { return {}; }

  /**
   * @brief Set velocities for all joints indexed by degrees of freedom.
   *
   * @param vels The desired joint velocities.
   */
  virtual void setJointVelocities(
      CORRADE_UNUSED const std::vector<float>& vels) {}

  /**
   * @brief Get current velocities for all joints indexed by degrees of freedom.
   *
   * @return The current joint velocities.
   */
  virtual std::vector<float> getJointVelocities() { return {}; }

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
  virtual void setJointPositions(
      CORRADE_UNUSED const std::vector<float>& positions) {}

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
  virtual std::vector<float> getJointPositions() { return {}; }

  /**
   * @brief Get the torques on each joint
   *
   * @param fixedTimeStep The physics timestep used by the simulator.  Necessary
   * to convert impulse into torque.
   *
   * @return Vector of torques on each joint
   */
  virtual std::vector<float> getJointMotorTorques(
      CORRADE_UNUSED double fixedTimeStep) {
    return {};
  }

  /**
   * @brief Get position limits for all joints.
   *
   * @return The active joint position limits. Default implementation returns
   * empty list.
   */
  virtual std::pair<std::vector<float>, std::vector<float>>
  getJointPositionLimits() {
    return {};
  }

  /**
   * @brief Get the linear velocity of the articulated object's root in the
   * global frame.
   *
   * @return The root linear velocity.
   */
  virtual Mn::Vector3 getRootLinearVelocity() const { return Mn::Vector3(0); }

  /**
   * @brief Set the linear velocity of the articulated object's root in the
   * global frame.
   *
   * @param linVel The root linear velocity.
   */
  virtual void setRootLinearVelocity(CORRADE_UNUSED const Mn::Vector3& linVel) {
  }

  /**
   * @brief Get the angular velocity (omega) of the articulated object's root in
   * the global frame.
   *
   * @return The root angular velocity (omega).
   */
  virtual Mn::Vector3 getRootAngularVelocity() const { return Mn::Vector3(0); }

  /**
   * @brief Set the angular velocity (omega) of the articulated object's root in
   * the global frame.
   *
   * @param angVel The root angular velocity (omega).
   */
  virtual void setRootAngularVelocity(
      CORRADE_UNUSED const Mn::Vector3& angVel) {}

  /**
   * @brief Add linear force to a link's COM specified in the global frame.
   *
   * @param linkId The link's index.
   * @param force The desired force to add.
   */
  virtual void addArticulatedLinkForce(CORRADE_UNUSED int linkId,
                                       CORRADE_UNUSED Magnum::Vector3 force) {}

  /**
   * @brief Get the friction coefficient for a link.
   *
   * @param linkId The link's index.
   * @return The link's friction coefficient.
   */
  virtual float getArticulatedLinkFriction(CORRADE_UNUSED int linkId) {
    return 0;
  }

  /**
   * @brief Set the friction coefficient for a link.
   *
   * @param linkId The link's index.
   * @param friction The link's friction coefficient.
   */
  virtual void setArticulatedLinkFriction(CORRADE_UNUSED int linkId,
                                          CORRADE_UNUSED float friction) {}

  /**
   * @brief Get the type of the link's parent joint.
   *
   * @param linkId The link's index.
   * @return The link's parent joint's type.
   */
  virtual JointType getLinkJointType(CORRADE_UNUSED int linkId) const {
    return JointType::Invalid;
  }

  /**
   * @brief Get the name of the link's parent joint.
   *
   * @param linkId The link's index.
   * @return The link's parent joint's name.
   */
  virtual std::string getLinkJointName(CORRADE_UNUSED int linkId) const {
    auto linkIter = links_.find(linkId);
    ESP_CHECK(linkIter != links_.end(),
              "ArticulatedObject::getLinkJointName - no link with linkId ="
                  << linkId);
    return linkIter->second->linkJointName;
  }

  /**
   * @brief Get the name of the link.
   *
   * @param linkId The link's index. -1 for base link.
   * @return The link's name.
   */
  virtual std::string getLinkName(CORRADE_UNUSED int linkId) const {
    if (linkId == -1) {
      return baseLink_->linkName;
    }

    auto linkIter = links_.find(linkId);
    ESP_CHECK(
        linkIter != links_.end(),
        "ArticulatedObject::getLinkName - no link with linkId =" << linkId);
    return linkIter->second->linkName;
  }

  /**
   * @brief Get the starting position for this link's parent joint in the global
   * DoFs array.
   *
   * @param linkId The link's index.
   * @return The link's starting DoF index.
   */
  virtual int getLinkDoFOffset(CORRADE_UNUSED int linkId) const { return -1; }

  /**
   * @brief Get the number of DoFs for this link's parent joint.
   *
   * @param linkId The link's index.
   * @return The number of DoFs for this link's parent joint.
   */
  virtual int getLinkNumDoFs(CORRADE_UNUSED int linkId) const { return 0; }

  /**
   * @brief Get the starting position for this link's parent joint in the global
   * positions array.
   *
   * @param linkId The link's index.
   * @return The link's starting position index.
   */
  virtual int getLinkJointPosOffset(CORRADE_UNUSED int linkId) const {
    return -1;
  }

  /**
   * @brief Get the number of positions for this link's parent joint.
   *
   * @param linkId The link's index.
   * @return The number of positions for this link's parent joint.
   */
  virtual int getLinkNumJointPos(CORRADE_UNUSED int linkId) const { return 0; }

  /**
   * @brief reset the articulated object state by clearing forces and zeroing
   * positions and velocities. Does not change root state.
   */
  virtual void reset() {}

  /**
   * @brief Check if this object can be de-activated (i.e. sleep).
   *
   * @return Whether or not the object is able to deactivate.
   */
  virtual bool getCanSleep() { return false; }

  /**
   * @brief Set whether articulated object state is automatically clamped to
   * configured joint limits before physics simulation.
   */
  void setAutoClampJointLimits(bool autoClamp) {
    autoClampJointLimits_ = autoClamp;
  }

  /**
   * @brief Query whether articulated object state is automatically clamped to
   * configured joint limits before physics simulation.
   */
  bool getAutoClampJointLimits() const { return autoClampJointLimits_; }

  /**
   * @brief Clamp current pose to joint limits.
   * See derived implementations.
   */
  virtual void clampJointLimits() {
    ESP_DEBUG() << "No base implementation of \"clampJointLimits\".";
  }

  //=========== Joint Motor API ===========

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
  virtual int createJointMotor(
      CORRADE_UNUSED const int index,
      CORRADE_UNUSED const JointMotorSettings& settings) {
    ESP_DEBUG() << "No base implementation of \"createJointMotor\". "
                   "Requires a physics simulator implementation.";
    return ID_UNDEFINED;
  }

  /**
   * @brief Remove and destroy a joint motor.
   */
  virtual void removeJointMotor(const int motorId) {
    auto jointMotorIter = jointMotors_.find(motorId);
    ESP_CHECK(
        jointMotorIter != jointMotors_.end(),
        "ArticulatedObject::removeJointMotor - No motor exists with motorId ="
            << motorId);
    jointMotors_.erase(jointMotorIter);
  }

  /**
   * @brief Get a copy of the JointMotorSettings for an existing motor.
   */
  virtual JointMotorSettings getJointMotorSettings(const int motorId) {
    auto jointMotorIter = jointMotors_.find(motorId);
    ESP_CHECK(jointMotorIter != jointMotors_.end(),
              "ArticulatedObject::getJointMotorSettings - No motor exists with "
              "motorId ="
                  << motorId);
    return jointMotorIter->second->settings;
  }

  /**
   * @brief Update a JointMotor with new settings.
   */
  virtual void updateJointMotor(const int motorId,
                                const JointMotorSettings& settings) {
    auto jointMotorIter = jointMotors_.find(motorId);
    ESP_CHECK(
        jointMotorIter != jointMotors_.end(),
        "ArticulatedObject::updateJointMotor - No motor exists with motorId ="
            << motorId);
    ESP_CHECK(
        jointMotorIter->second->settings.motorType == settings.motorType,
        "ArticulatedObject::updateJointMotor - JointMotorSettings.motorType "
        "does not match joint type.");
    jointMotorIter->second->settings = settings;
  }

  /**
   * @brief Query a map of motorIds -> links/joints for all active JointMotors.
   */
  virtual std::unordered_map<int, int> getExistingJointMotors() {
    std::unordered_map<int, int> motorIdsToLinkIds;
    motorIdsToLinkIds.reserve(jointMotors_.size());
    for (auto& motor : jointMotors_) {
      motorIdsToLinkIds[motor.first] = motor.second->index;
    }
    return motorIdsToLinkIds;
  }

  /**
   * @brief Create a new set of default JointMotors for all valid dofs in an
   * ArticulatedObject.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @return A map motorIds to link/joint indices for the new motors.
   */
  virtual std::unordered_map<int, int> createMotorsForAllDofs(
      CORRADE_UNUSED const JointMotorSettings& settings =
          JointMotorSettings()) {
    ESP_ERROR() << "ERROR, SHOULD NOT BE CALLED WITHOUT BULLET";
    return std::unordered_map<int, int>();
  }

  /**
   * @brief Update all motors targets for this object's joints which support
   * motors (Revolute, Prismatic, Spherical) from a state array.
   *
   * By default, state is interpreted as position targets unless `velocities` is
   * specified. Expected input is the full length position or velocity array for
   * this object. This function will safely skip states for jointa which don't
   * support JointMotors.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @param stateTargets Full length joint position or velocity array for this
   * object.
   * @param velocities Whether to interpret stateTargets as velocities or
   * positions.
   */
  virtual void updateAllMotorTargets(
      CORRADE_UNUSED const std::vector<float>& stateTargets,
      CORRADE_UNUSED bool velocities = false) {
    ESP_ERROR() << "ERROR,SHOULD NOT BE CALLED WITHOUT BULLET";
  }

  //=========== END - Joint Motor API ===========

  //! map PhysicsManager objectId to local multibody linkId
  std::unordered_map<int, int> objectIdToLinkId_;

  /**
   * @brief Returns the @ref
   * metadata::attributes::SceneAOInstanceAttributes used to place this
   * Articulated Object initially in the scene.
   * @return a read-only copy of the scene instance attributes used to place
   * this object in the scene.
   */
  std::shared_ptr<const metadata::attributes::SceneAOInstanceAttributes>
  getInitObjectInstanceAttr() const {
    return PhysicsObjectBase::getInitObjectInstanceAttrInternal<
        const metadata::attributes::SceneAOInstanceAttributes>();
  }

  /**
   * @brief Return a @ref
   * metadata::attributes::SceneAOInstanceAttributes reflecting the current
   * state of this Articulated Object.
   * Note : base PhysicsManager implementation does not support state changes on
   * ArticulatedObjects, so no change will occur from initialization
   * InstanceAttributes.
   * @return a @ref SceneAOInstanceAttributes reflecting this Articulated
   * Object's current state
   */
  virtual std::shared_ptr<metadata::attributes::SceneAOInstanceAttributes>
  getCurrentStateInstanceAttr() {
    return PhysicsObjectBase::getCurrentObjectInstanceAttrInternal<
        metadata::attributes::SceneAOInstanceAttributes>();
  }

 protected:
  /**
   * @brief Used to synchronize simulator's notion of the object state
   * after it was changed kinematically. Must be called automatically on
   * kinematic updates.  For ArticulatedObjects, the transformation of the root
   * node is used to transform the root physical constructs.
   */
  void syncPose() override { this->setRootState(node().transformation()); }

  /**
   * @brief Called internally from syncPose()  Used to update physics
   * constructs when kinematic transformations are performed manually.  See @ref
   * esp::physics::PhysicsObjectBase for the transformations.
   */
  virtual void setRootState(CORRADE_UNUSED const Magnum::Matrix4& state) {}

  //! map linkId to ArticulatedLink
  std::map<int, ArticulatedLink::uptr> links_;

  //! link object for the AO base
  ArticulatedLink::uptr baseLink_;

  //! map motorId to JointMotor
  std::unordered_map<int, JointMotor::uptr> jointMotors_;

  //! if true, automatically clamp dofs to joint limits before physics
  //! simulation steps
  bool autoClampJointLimits_ = false;

  //! Cache the global scaling from the source model. Set during import.
  float globalScale_ = 1.0;

 public:
  ESP_SMART_POINTERS(ArticulatedObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDOBJECT_H_
