// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDOBJECT_H_
#define ESP_PHYSICS_ARTICULATEDOBJECT_H_

/** @file
 * @brief Class @ref esp::physics::ArticulatedLink, Class @ref
 * esp::physics::ArticulatedObject
 */

#include "RigidBase.h"
#include "esp/io/URDFParser.h"
#include "esp/scene/SceneNode.h"

namespace esp {

namespace gfx {
class DrawableGroup;
}

namespace assets {
class ResourceManager;
}

namespace io {
struct URDFLinkContactInfo;
struct UrdfLink;
}  // namespace io

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

enum class JointMotorType { SingleDof, Spherical };

struct JointMotorSettings {
 public:
  JointMotorSettings() = default;

  JointMotorSettings(double _positionTarget,
                     double _positionGain,
                     double _velocityTarget,
                     double _velocityGain,
                     double _maxImpulse) {
    positionTarget = _positionTarget;
    positionGain = _positionGain;
    velocityTarget = _velocityTarget;
    velocityGain = _velocityGain;
    maxImpulse = _maxImpulse;
  }

  JointMotorType motorType = JointMotorType::SingleDof;
  double positionTarget = 0.0;
  Mn::Quaternion sphericalPositionTarget = {};
  double positionGain = 0.0;
  double velocityTarget = 0.0;
  Mn::Vector3 sphericalVelocityTarget = {};
  double velocityGain = 1.0;
  double maxImpulse = 1000.0;

  ESP_SMART_POINTERS(JointMotorSettings)
};

struct JointMotor {
  JointMotorSettings settings;
  int index;
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
   * phyiscal parameters for this object
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
    Corrade::Utility::Debug()
        << "(setTransformation) - ArticulatedLink can't do this.";
  }

  void setTranslation(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug()
        << "(setTranslation) - ArticulatedLink can't do this.";
  }

  void setRotation(
      CORRADE_UNUSED const Magnum::Quaternion& quaternion) override {
    Corrade::Utility::Debug()
        << "(setRotation) - ArticulatedLink can't do this.";
  }

  void setRigidState(
      CORRADE_UNUSED const core::RigidState& rigidState) override {
    Corrade::Utility::Debug()
        << "(setRigidState) - ArticulatedLink can't do this.";
  }

  void resetTransformation() override {
    Corrade::Utility::Debug()
        << "(resetTransformation) - ArticulatedLink can't do this.";
  }

  void translate(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug() << "(translate) - ArticulatedLink can't do this.";
  }

  void translateLocal(CORRADE_UNUSED const Magnum::Vector3& vector) override {
    Corrade::Utility::Debug()
        << "(translateLocal) - ArticulatedLink can't do this.";
  }

  void rotate(CORRADE_UNUSED const Magnum::Rad angleInRad,
              CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    Corrade::Utility::Debug() << "(rotate) - ArticulatedLink can't do this.";
  }

  void rotateLocal(
      CORRADE_UNUSED const Magnum::Rad angleInRad,
      CORRADE_UNUSED const Magnum::Vector3& normalizedAxis) override {
    Corrade::Utility::Debug()
        << "(rotateLocal) - ArticulatedLink can't do this.";
  }

  void rotateX(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateX) - ArticulatedLink can't do this.";
  }
  void rotateY(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateY) - ArticulatedLink can't do this.";
  }
  void rotateZ(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug() << "(rotateZ) - ArticulatedLink can't do this.";
  }
  void rotateXLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateXLocal) - ArticulatedLink can't do this.";
  }
  void rotateYLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateYLocal) - ArticulatedLink can't do this.";
  }
  void rotateZLocal(CORRADE_UNUSED const Magnum::Rad angleInRad) override {
    Corrade::Utility::Debug()
        << "(rotateZLocal) - ArticulatedLink can't do this.";
  }

  /**
   * @brief Not used for articulated links.  Set or reset the object's state
   * using the object's specified @p sceneInstanceAttributes_.
   * @param defaultCOMCorrection The default value of whether COM-based
   * translation correction needs to occur.
   */
  void resetStateFromSceneInstanceAttr(
      CORRADE_UNUSED bool defaultCOMCorrection = false) override {
    Corrade::Utility::Debug()
        << "(resetStateFromSceneInstanceAttr) - ArticulatedLink can't do this.";
  }

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
    for (auto& link : links_) {
      linkNodes.push_back(&link.second->node());
    }
    links_.clear();
    for (auto* node : linkNodes) {
      delete node;
    }
  };

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
    CHECK(links_.count(linkId));
    return links_.at(linkId)->node();
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
    CHECK(links_.count(linkId));
    return links_.at(linkId)->visualNodes_;
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

  virtual bool initializeFromURDF(
      CORRADE_UNUSED URDFImporter& urdfImporter,
      CORRADE_UNUSED const Magnum::Matrix4& worldTransform,
      CORRADE_UNUSED gfx::DrawableGroup* drawables,
      CORRADE_UNUSED scene::SceneNode* physicsNode,
      CORRADE_UNUSED bool fixedBase = false) {
    return false;
  };

  ArticulatedLink& getLink(int id) {
    // option to get the baseLink_ with id=-1
    if (id == -1) {
      return *baseLink_.get();
    }
    CHECK(links_.count(id));
    return *links_.at(id).get();
  }

  int getNumLinks() const { return links_.size(); }

  std::vector<int> getLinkIds() const {
    std::vector<int> ids;
    for (auto it = links_.begin(); it != links_.end(); ++it) {
      ids.push_back(it->first);
    }
    return ids;
  }

  virtual void setForces(CORRADE_UNUSED const std::vector<float>& forces) {}
  virtual void addForces(CORRADE_UNUSED const std::vector<float>& forces) {}

  virtual std::vector<float> getForces() { return {}; }

  virtual void setVelocities(CORRADE_UNUSED const std::vector<float>& vels) {}

  virtual std::vector<float> getVelocities() { return {}; }

  virtual void setPositions(
      CORRADE_UNUSED const std::vector<float>& positions) {}

  virtual std::vector<float> getPositions() { return {}; }

  virtual std::vector<float> getPositionLimits(
      CORRADE_UNUSED bool upperLimits = false) {
    return {};
  }

  virtual Mn::Vector3 getRootLinearVelocity() const { return Mn::Vector3(0); }
  virtual void setRootLinearVelocity(CORRADE_UNUSED const Mn::Vector3& linVel) {
  }

  virtual Mn::Vector3 getRootAngularVelocity() const { return Mn::Vector3(0); }
  virtual void setRootAngularVelocity(
      CORRADE_UNUSED const Mn::Vector3& angVel) {}

  virtual void addArticulatedLinkForce(CORRADE_UNUSED int linkId,
                                       CORRADE_UNUSED Magnum::Vector3 force) {}

  virtual float getArticulatedLinkFriction(CORRADE_UNUSED int linkId) {
    return 0;
  }

  virtual void setArticulatedLinkFriction(CORRADE_UNUSED int linkId,
                                          CORRADE_UNUSED float friction) {}

  virtual JointType getLinkJointType(CORRADE_UNUSED int linkId) const {
    return JointType::Invalid;
  }

  virtual int getLinkDoFOffset(CORRADE_UNUSED int linkId) const { return -1; }

  virtual int getLinkNumDoFs(CORRADE_UNUSED int linkId) const { return 0; }

  virtual int getLinkJointPosOffset(CORRADE_UNUSED int linkId) const {
    return -1;
  }

  virtual int getLinkNumJointPos(CORRADE_UNUSED int linkId) const { return 0; }

  /**
   * @brief reset the articulated object state by clearing forces and zeroing
   * positions and velocities. Does not change root state.
   */
  virtual void reset() {}

  /**
   * @brief Check if this object can be de-activated (i.e. sleep).
   */
  virtual bool getCanSleep() { return false; }

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
    Magnum::Debug{} << "No base implementation of \"createJointMotor\". "
                       "Requires a physics simulator implementation.";
    return ID_UNDEFINED;
  }

  /**
   * @brief Remove and destroy a joint motor.
   */
  virtual void removeJointMotor(const int motorId) {
    CHECK(jointMotors_.count(motorId) > 0);
    jointMotors_.erase(motorId);
  }

  /**
   * @brief Get a copy of the JointMotorSettings for an existing motor.
   */
  virtual JointMotorSettings getJointMotorSettings(const int motorId) {
    CHECK(jointMotors_.count(motorId) > 0);
    return jointMotors_.at(motorId)->settings;
  }

  /**
   * @brief Update a JointMotor with new settings.
   */
  virtual void updateJointMotor(const int motorId,
                                const JointMotorSettings& settings) {
    CHECK(jointMotors_.count(motorId) > 0);
    CHECK(jointMotors_.at(motorId)->settings.motorType == settings.motorType);
    jointMotors_.at(motorId)->settings = settings;
  }

  /**
   * @brief Query a map of motorIds -> dofs (or links for spherical motors) for
   * all active JointMotors.
   */
  virtual std::map<int, int> getExistingJointMotors() {
    std::map<int, int> motorIdsToDofIds;
    for (auto& motor : jointMotors_) {
      motorIdsToDofIds[motor.first] = motor.second->index;
    }
    return motorIdsToDofIds;
  }

  /**
   * @brief Create a new set of default JointMotors for all valid dofs in an
   * ArticulatedObject.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @return A map of dofs -> motorIds for the new motors.
   */
  virtual std::map<int, int> createMotorsForAllDofs(
      CORRADE_UNUSED JointMotorSettings settings = JointMotorSettings()) {
    Magnum::Debug{} << "ArticulatedObject::createMotorsForAllDofs(): - ERROR, "
                       "SHOULD NOT BE CALLED WITHOUT BULLET ";
    return std::map<int, int>();
  }

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
    Magnum::Debug{} << "No base implementation of \"clampJointLimits\". ";
  }

  //=========== END - Joint Motor API ===========

  //! map PhysicsManager objectId to local multibody linkId
  std::map<int, int> objectIdToLinkId_;

  /**
   * @brief Returns the @ref
   * metadata::attributes::SceneAOInstanceAttributes used to place this
   * Articulated object in the scene.
   * @return a copy of the scene instance attributes used to place this object
   * in the scene.
   */
  std::shared_ptr<metadata::attributes::SceneAOInstanceAttributes>
  getSceneInstanceAttributes() const {
    return PhysicsObjectBase::getSceneInstanceAttrInternal<
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
  virtual bool attachGeometry(
      CORRADE_UNUSED ArticulatedLink* linkObject,
      CORRADE_UNUSED const std::shared_ptr<io::URDF::Link>& link,
      CORRADE_UNUSED gfx::DrawableGroup* drawables) {
    return false;
  }

  //! map linkId to ArticulatedLink
  std::map<int, ArticulatedLink::uptr> links_;

  //! link object for the AO base
  ArticulatedLink::uptr baseLink_;

  //! map motorId to JointMotor
  std::map<int, JointMotor::uptr> jointMotors_;

  //! if true, automatically clamp dofs to joint limits before physics
  //! simulation steps
  bool autoClampJointLimits_ = false;

 public:
  ESP_SMART_POINTERS(ArticulatedObject)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDOBJECT_H_
