// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSMANAGER_H_
#define ESP_PHYSICS_PHYSICSMANAGER_H_

/** @file
 * @brief Class @ref esp::physics::PhysicsManager, enum @ref
 * esp::physics::PhysicsManager::PhysicsSimulationLibrary
 */

#include <map>
#include <memory>
#include <string>
#include <vector>

/* Bullet Physics Integration */

#include "ArticulatedObject.h"
#include "CollisionGroupHelper.h"
#include "RigidObject.h"
#include "RigidStage.h"
#include "URDFImporter.h"
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/CollisionMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/MeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/assets/ResourceManager.h"
#include "esp/gfx/DrawableGroup.h"
#include "esp/scene/SceneNode.h"

namespace esp {
//! core physics simulation namespace
namespace physics {

//! Holds information about one ray hit instance.
struct RayHitInfo {
  //! The id of the object hit by this ray. Stage hits are -1.
  int objectId{};
  //! The first impact point of the ray in world space.
  Magnum::Vector3 point;
  //! The collision object normal at the point of impact.
  Magnum::Vector3 normal;
  //! Distance along the ray direction from the ray origin (in units of ray
  //! length).
  double rayDistance{};

  ESP_SMART_POINTERS(RayHitInfo)
};

//! Holds information about all ray hit instances from a ray cast.
struct RaycastResults {
  std::vector<RayHitInfo> hits;
  esp::geo::Ray ray;

  bool hasHits() { return hits.size() > 0; }

  void sortByDistance() {
    std::sort(hits.begin(), hits.end(),
              [](const RayHitInfo& A, const RayHitInfo& B) {
                return A.rayDistance < B.rayDistance;
              });
  }

  ESP_SMART_POINTERS(RaycastResults)
};

// based on Bullet b3ContactPointData
struct ContactPointData {
  int objectIdA = -2;  // stage is -1
  int objectIdB = -2;
  int linkIndexA = -1;  // -1 if not a multibody
  int linkIndexB = -1;

  Magnum::Vector3 positionOnAInWS;  // contact point location on object A, in
                                    // world space coordinates
  Magnum::Vector3 positionOnBInWS;  // contact point location on object B, in
                                    // world space coordinates
  Magnum::Vector3
      contactNormalOnBInWS;  // the separating contact normal, pointing from
                             // object B towards object A
  double contactDistance =
      0.0;  // negative number is penetration, positive is distance.

  double normalForce = 0.0;

  double linearFrictionForce1 = 0.0;
  double linearFrictionForce2 = 0.0;
  Magnum::Vector3 linearFrictionDirection1;
  Magnum::Vector3 linearFrictionDirection2;

  // the contact is considered active if at least one object is active (not
  // asleep)
  bool isActive = false;

  ESP_SMART_POINTERS(ContactPointData)
};

// TODO: repurpose to manage multiple physical worlds. Currently represents
// exactly one world.

/**
@brief Kinematic and dynamic scene and object manager.

Responsible for tracking, updating, and synchronizing the state of the physical
world and all non-static geometry in the scene as well as interfacing with
specific physical simulation implementations.

The physical world in this case consists of any objects which can be manipulated
(kinematically or dynamically) or simulated and anything such objects must be
aware of (e.g. static scene collision geometry).

Will later manager multiple physical scenes, but currently assumes only one
unique physical world can exist.
*/
class PhysicsManager {
 public:
  //! ==== physics engines ====

  /**
  @brief The specific physics implementation used by the current @ref
  PhysicsManager. Each entry suggests a derived class of @ref PhysicsManager and
  @ref RigidObject implementing the specific interface to a simulation library.
  */
  enum PhysicsSimulationLibrary {

    /**
     * The default implemenation of kineamtics through the base @ref
     * PhysicsManager class. Supports @ref MotionType::STATIC and @ref
     * MotionType::KINEMATIC objects of base class @ref RigidObject. If the
     * derived @ref PhysicsManager class for a desired @ref
     * PhysicsSimulationLibrary fails to initialize, it will default to @ref
     * PhysicsSimulationLibrary::NONE.
     */
    NONE,

    /**
     * An implemenation of dynamics through the Bullet Physics library.
     * Supports @ref MotionType::STATIC, @ref MotionType::KINEMATIC, and @ref
     * MotionType::DYNAMIC objects of @ref RigidObject derived class @ref
     * BulletRigidObject. Suggests the use of @ref PhysicsManager derived class
     * @ref BulletPhysicsManager
     */
    BULLET
  };

  /**
   * @brief Construct a #ref PhysicsManager with access to specific resource
   * assets.
   *
   * @param _resourceManager The @ref esp::assets::ResourceManager which
   * tracks the assets this
   * @ref PhysicsManager will have access to.
   */
  explicit PhysicsManager(
      assets::ResourceManager& _resourceManager,
      const metadata::attributes::PhysicsManagerAttributes::cptr&
          _physicsManagerAttributes)
      : resourceManager_(_resourceManager),
        physicsManagerAttributes_(_physicsManagerAttributes){};

  /** @brief Destructor*/
  virtual ~PhysicsManager();

  /**
   * @brief Initialization: load physical properties and setup the world.
   *
   *
   * @param node    The scene graph node which will act as the parent of all
   * physical scene and object nodes.
   */
  bool initPhysics(scene::SceneNode* node);

  /**
   * @brief Reset the simulation and physical world.
   * Sets the @ref worldTime_ to 0.0, does not change physical state.
   */
  virtual void reset() {
    /* TODO: reset object states or clear them? Other? */
    worldTime_ = 0.0;
  }

  /** @brief Stores references to a set of drawable elements. */
  using DrawableGroup = gfx::DrawableGroup;

  /**
   * @brief Initialize static scene collision geometry from loaded mesh data.
   * Only one 'scene' may be initialized per simulated world, but this scene may
   * contain several components (e.g. GLB heirarchy).
   *
   * @param initAttributes The attributes structure defining physical
   * properties of the scene.  Must be a copy of the attributes stored in the
   * Attributes Manager.
   * @param meshGroup collision meshs for the scene.
   * @return true if successful and false otherwise
   */
  bool addStage(
      const metadata::attributes::StageAttributes::ptr& initAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager.
   * @anchor addObject_string
   * @param attributesHandle The handle of the object attributes used as the key
   * to query @ref esp::metadata::managers::ObjectAttributesManager.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(const std::string& attributesHandle,
                DrawableGroup* drawables,
                scene::SceneNode* attachmentNode = nullptr,
                const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    esp::metadata::attributes::ObjectAttributes::ptr attributes =
        resourceManager_.getObjectAttributesManager()->getObjectCopyByHandle(
            attributesHandle);
    if (!attributes) {
      LOG(ERROR) << "PhysicsManager::addObject : "
                    "Object creation failed due to unknown attributes "
                 << attributesHandle;
      return ID_UNDEFINED;
    }

    return addObject(attributes, drawables, attachmentNode, lightSetup);
  }

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager by template
   * handle.
   * @param attributesID The ID of the object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(const int attributesID,
                DrawableGroup* drawables,
                scene::SceneNode* attachmentNode = nullptr,
                const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    const esp::metadata::attributes::ObjectAttributes::ptr attributes =
        resourceManager_.getObjectAttributesManager()->getObjectCopyByID(
            attributesID);
    if (!attributes) {
      LOG(ERROR) << "PhysicsManager::addObject : "
                    "Object creation failed due to unknown attributes ID "
                 << attributesID;
      return ID_UNDEFINED;
    }
    return addObject(attributes, drawables, attachmentNode, lightSetup);
  }

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager by template
   * handle.
   * @param objectAttributes The object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      DrawableGroup* drawables,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /** @brief Remove an object instance from the pysical scene by ID, destroying
   * its scene graph node and removing it from @ref
   * PhysicsManager::existingObjects_.
   *  @param physObjectID The ID (key) of the object instance in @ref
   * PhysicsManager::existingObjects_.
   * @param deleteObjectNode If true, deletes the object's scene node. Otherwise
   * detaches the object from simulation.
   * @param deleteVisualNode If true, deletes the object's visual node.
   * Otherwise detaches the object from simulation. Is not considered if
   * deleteObjectNode==true.
   */
  virtual void removeObject(const int physObjectID,
                            bool deleteObjectNode = true,
                            bool deleteVisualNode = true);

  /** @brief Get the number of objects mapped in @ref
   * PhysicsManager::existingObjects_.
   *  @return The size of @ref PhysicsManager::existingObjects_.
   */
  int getNumRigidObjects() const { return existingObjects_.size(); };

  /** @brief Get a list of existing object IDs (i.e., existing keys in @ref
   * PhysicsManager::existingObjects_.)
   *  @return List of object ID keys from @ref PhysicsManager::existingObjects_.
   */
  std::vector<int> getExistingObjectIDs() const {
    std::vector<int> v;
    for (auto& bro : existingObjects_) {
      v.push_back(bro.first);
    }
    return v;
  };

  /** @brief Get a list of existing object IDs for articulated objects (i.e.,
   * existing keys in @ref PhysicsManager::existingArticulatedObjects_.)
   *  @return List of object ID keys from @ref
   * PhysicsManager::existingArticulatedObjects_.
   */
  std::vector<int> getExistingArticulatedObjectIDs() const {
    std::vector<int> v;
    for (auto& bro : existingArticulatedObjects_) {
      v.push_back(bro.first);
    }
    return v;
  };

  /** @brief Set the @ref MotionType of an object, allowing or disallowing its
   * manipulation by dynamic processes or kinematic control.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param  mt The desired @ref MotionType of the object to set.
   */
  void setObjectMotionType(const int physObjectID, MotionType mt);

  /** @brief Get the @ref MotionType of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The object's @ref MotionType
   */
  MotionType getObjectMotionType(const int physObjectID) const;

  //============= ArticulatedObject functions =============
  // TODO: think more about how these should be incorporated into the existing
  // framework
  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * ArticulatedObject in the world.
   *
   * Not implemented in base PhysicsManager.
   *
   * @return A unique id for the @ref ArticulatedObject, allocated from the same
   * id set as rigid objects.
   */
  virtual int addArticulatedObjectFromURDF(
      CORRADE_UNUSED const std::string& filepath,
      CORRADE_UNUSED DrawableGroup* drawables,
      CORRADE_UNUSED bool fixedBase = false,
      CORRADE_UNUSED float globalScale = 1.0,
      CORRADE_UNUSED float massScale = 1.0,
      CORRADE_UNUSED bool forceReload = false) {
    Magnum::Debug{} << "addArticulatedObjectFromURDF not implemented in base "
                       "PhysicsManager.";
    return ID_UNDEFINED;
  };

  //! remove an @ref ArticulatedObject from the world by unique id.
  virtual void removeArticulatedObject(int id);

  int getNumArticulatedObjects() { return existingArticulatedObjects_.size(); };

  //! return a list of ids for all existing @ref ArticulatedObjects in the world
  std::vector<int> getArticulatedObjectIds() {
    std::vector<int> ids;
    for (auto it = existingArticulatedObjects_.begin();
         it != existingArticulatedObjects_.end(); ++it) {
      ids.push_back(it->first);
    }
    return ids;
  };

  ArticulatedObject& getArticulatedObject(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return *existingArticulatedObjects_.at(objectId).get();
  };

  //============= ArticulatedObject JointMotor API =============

  /**
   * @brief Create a new JointMotor for a dof in an ArticulatedObject from a
   * JointMotorSettings.
   *
   * @return The motorId for the new joint motor or ID_UNDEFINED (-1) if failed.
   */
  int createJointMotor(const int objectId,
                       const int dof,
                       const JointMotorSettings& settings) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->createJointMotor(dof,
                                                                      settings);
  }

  /**
   * @brief Remove and destroy a JointMotor for an ArticulatedObject.
   */
  void removeJointMotor(const int objectId, const int motorId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->removeJointMotor(motorId);
  }

  /**
   * @brief Get a copy of the JointMotorSettings for an ArticulatedObject's
   * existing JointMotor .
   */
  JointMotorSettings getJointMotorSettings(const int objectId,
                                           const int motorId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getJointMotorSettings(
        motorId);
  };

  /**
   * @brief Update an ArticulatedObject's JointMotor with new settings.
   */
  void updateJointMotor(const int objectId,
                        const int motorId,
                        const JointMotorSettings& settings) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->updateJointMotor(motorId,
                                                               settings);
  };

  /**
   * @brief Query a map of motorIds -> dofs for all active JointMotors attached
   * to an ArticulatedObject.
   */
  std::map<int, int> getExistingJointMotors(const int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getExistingJointMotors();
  };

  /**
   * @brief Create a new set of default JointMotors for all valid dofs in an
   * ArticulatedObject.
   *
   * Note: No base implementation. See @ref bullet::BulletArticulatedObject.
   *
   * @return A map of dofs -> motorIds for the new motors.
   */
  virtual std::map<int, int> createMotorsForAllDofs(
      const int objectId,
      JointMotorSettings settings = JointMotorSettings()) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->createMotorsForAllDofs(
        settings);
  };

  //============= Object Point to Point Constraint API =============

  /**
   * @brief Create a ball&socket joint to constrain a DYNAMIC RigidObject
   * provided a position in local or global coordinates. Note: Method not
   * implemented for base PhysicsManager.
   * @param objectId The id of the RigidObject to constrain.
   * @param position The position of the ball and socket joint pivot.
   * @param positionLocal Indicates whether the position is provided in global
   * or object local coordinates.
   * @return The unique id of the new constraint.
   */
  virtual int createRigidP2PConstraint(
      CORRADE_UNUSED int objectId,
      CORRADE_UNUSED const Magnum::Vector3& position,
      CORRADE_UNUSED bool positionLocal = true) {
    Magnum::Debug{}
        << "createRigidP2PConstraint not implemented in base PhysicsManager.";
    return ID_UNDEFINED;
  };

  // TODO: document AO->rigid
  virtual int createArticulatedP2PConstraint(
      CORRADE_UNUSED int articulatedObjectId,
      CORRADE_UNUSED int linkId,
      CORRADE_UNUSED int objectId,
      CORRADE_UNUSED float maxImpulse,
      CORRADE_UNUSED const Corrade::Containers::Optional<Magnum::Vector3>&
          pivotA,
      CORRADE_UNUSED const Corrade::Containers::Optional<Magnum::Vector3>&
          pivotB) {
    return -1;
  }

  // TODO: document AO->rigid
  virtual int createArticulatedFixedConstraint(
      CORRADE_UNUSED int articulatedObjectId,
      CORRADE_UNUSED int linkId,
      CORRADE_UNUSED int objectId,
      CORRADE_UNUSED float maxImpulse,
      CORRADE_UNUSED const Corrade::Containers::Optional<Magnum::Vector3>&
          pivotA,
      CORRADE_UNUSED const Corrade::Containers::Optional<Magnum::Vector3>&
          pivotB) {
    return -1;
  }

  /**
   * @brief Create a ball&socket joint to constrain two links of two
   * ArticulatedObjects to one another with local offsets for each.
   * Note: Method not implemented for base PhysicsManager.
   * @param articulatedObjectIdA The id of the first ArticulatedObject to
   * constrain.
   * @param linkIdA The local id of the first ArticulatedLink to constrain.
   * @param linkOffsetA The position of the first ball and socket joint pivot in
   * link A local space.
   * @param articulatedObjectIdB The id of the second ArticulatedObject to
   * constrain.
   * @param linkIdB The local id of the second ArticulatedLink to constrain.
   * @param linkOffsetB The position of the ball and socket joint pivot in link
   * B local space.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(
      CORRADE_UNUSED int articulatedObjectIdA,
      CORRADE_UNUSED int linkIdA,
      CORRADE_UNUSED const Magnum::Vector3& linkOffsetA,
      CORRADE_UNUSED int articulatedObjectIdB,
      CORRADE_UNUSED int linkIdB,
      CORRADE_UNUSED const Magnum::Vector3& linkOffsetB,
      CORRADE_UNUSED float maxImpulse = 2.0) {
    Magnum::Debug{} << "createArticulatedP2PConstraint not implemented in base "
                       "PhysicsManager.";
    return ID_UNDEFINED;
  };

  /**
   * @brief Create a ball&socket joint to constrain two links of two
   * ArticulatedObjects to one another at some global point.
   * Note: Method not implemented for base PhysicsManager.
   * @param articulatedObjectIdA The id of the first ArticulatedObject to
   * constrain.
   * @param linkIdA The local id of the first ArticulatedLink to constrain.
   * @param articulatedObjectIdB The id of the second ArticulatedObject to
   * constrain.
   * @param linkIdB The local id of the second ArticulatedLink to constrain.
   * @param globalConstraintPoint The position of the ball and socket joint
   * pivot in global space.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(
      CORRADE_UNUSED int articulatedObjectIdA,
      CORRADE_UNUSED int linkIdA,
      CORRADE_UNUSED int articulatedObjectIdB,
      CORRADE_UNUSED int linkIdB,
      CORRADE_UNUSED const Magnum::Vector3& globalConstraintPoint,
      CORRADE_UNUSED float maxImpulse = 2.0) {
    Magnum::Debug{} << "createArticulatedP2PConstraint not implemented in base "
                       "PhysicsManager.";
    return ID_UNDEFINED;
  };

  /**
   * @brief Create a ball&socket joint to constrain a single link of an
   * ArticulatedObject provided a position in global coordinates and a local
   * offset. Note: Method not implemented for base PhysicsManager.
   * @param articulatedObjectId The id of the ArticulatedObject to constrain.
   * @param linkId The local id of the ArticulatedLink to constrain.
   * @param linkOffset The position of the ball and socket joint pivot in link
   * local coordinates.
   * @param pickPos The global position of the ball and socket joint pivot.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(
      CORRADE_UNUSED int articulatedObjectId,
      CORRADE_UNUSED int linkId,
      CORRADE_UNUSED const Magnum::Vector3& linkOffset,
      CORRADE_UNUSED const Magnum::Vector3& pickPos,
      CORRADE_UNUSED float maxImpulse = 2.0) {
    Magnum::Debug{} << "createArticulatedP2PConstraint not implemented in base "
                       "PhysicsManager.";
    return ID_UNDEFINED;
  };

  /**
   * @brief Create a ball&socket joint to constrain a single link of an
   * ArticulatedObject provided a position in global coordinates. Note: Method
   * not implemented for base PhysicsManager.
   * @param articulatedObjectId The id of the ArticulatedObject to constrain.
   * @param linkId The local id of the ArticulatedLink to constrain.
   * @param pickPos The global position of the ball and socket joint pivot.
   * @return The unique id of the new constraint.
   */
  virtual int createArticulatedP2PConstraint(
      CORRADE_UNUSED int articulatedObjectId,
      CORRADE_UNUSED int linkId,
      CORRADE_UNUSED const Magnum::Vector3& pickPos,
      CORRADE_UNUSED float maxImpulse = 2.0) {
    Magnum::Debug{} << "createArticulatedP2PConstraint not implemented in base "
                       "PhysicsManager.";
    return ID_UNDEFINED;
  };

  /**
   * @brief Update the position target (pivot) of a constraint.
   * Note: Method not implemented for base PhysicsManager.
   * @param p2pId The id of the constraint to update.
   * @param pivot The new position target of the constraint.
   */
  virtual void updateP2PConstraintPivot(
      CORRADE_UNUSED int p2pId,
      CORRADE_UNUSED const Magnum::Vector3& pivot) {
    Magnum::Debug{}
        << "updateP2PConstraintPivot not implemented in base PhysicsManager.";
  };

  /**
   * @brief Remove a constraint by id.
   * Note: Method not implemented for base PhysicsManager.
   * @param constraintId The id of the constraint to remove.
   */
  virtual void removeConstraint(CORRADE_UNUSED int constraintId) {
    Magnum::Debug{}
        << "removeConstraint not implemented in base PhysicsManager.";
  };

  //============ Simulator functions =============

  /** @brief Step the physical world forward in time. Time may only advance in
   * increments of @ref fixedTimeStep_.
   * @param dt The desired amount of time to advance the physical world.
   */
  virtual void stepPhysics(double dt = 0.0);

  // Defers the update of the scene graph nodes until updateNodes is called
  // This is needed to do ownership transfer of the scene graph to a
  // background thread
  virtual void deferNodesUpdate();

  // Syncs the state of the bullet scene graph to the rendering scene graph
  virtual void updateNodes();

  // =========== Global Setter functions ===========

  /** @brief Set the @ref fixedTimeStep_ of the physical world. See @ref
   * stepPhysics.
   * @param dt The increment of time by which the physical world will advance.
   */
  virtual void setTimestep(double dt);

  /** @brief Set the gravity of the physical world if the world is dyanmic and
   * therefore has a notion of force. By default does nothing since the world is
   * kinematic. Exact implementations of gravity will depend on the specific
   * dynamics of the derived physical simulator class.
   * @param gravity The desired gravity force of the physical world.
   */
  virtual void setGravity(const Magnum::Vector3& gravity);

  // =========== Global Getter functions ===========

  /** @brief Get the @ref fixedTimeStep_ of the physical world. See @ref
   * stepPhysics.
   * @return The increment of time, @ref fixedTimeStep_, by which the physical
   * world will advance.
   */
  virtual double getTimestep() const { return fixedTimeStep_; };

  /** @brief Get the current @ref worldTime_ of the physical world. See @ref
   * stepPhysics.
   * @return The amount of time, @ref worldTime_, by which the physical world
   * has advanced.
   */
  virtual double getWorldTime() const { return worldTime_; };

  /** @brief Get the current gravity in the physical world. By default returns
   * [0,0,0] since their is no notion of force in a kinematic world.
   * @return The current gravity vector in the physical world.
   */
  virtual Magnum::Vector3 getGravity() const;

  // =========== Stage Getter/Setter functions ===========

  /** @brief Get the current friction coefficient of the scene collision
   * geometry. See @ref staticStageObject_.
   * @return The scalar friction coefficient of the scene geometry.
   */
  virtual double getStageFrictionCoefficient() const { return 0.0; };

  /** @brief Set the friction coefficient of the scene collision geometry. See
   * @ref staticStageObject_.
   * @param frictionCoefficient The scalar friction coefficient of the scene
   * geometry.
   */
  virtual void setStageFrictionCoefficient(
      CORRADE_UNUSED const double frictionCoefficient){};

  /** @brief Get the current coefficient of restitution for the scene collision
   * geometry. This determines the ratio of initial to final relative velocity
   * between the scene and collidiing object. See @ref staticStageObject_. By
   * default this will always return 0, since kinametic scenes have no dynamics.
   * @return The scalar coefficient of restitution for the scene geometry.
   */
  virtual double getStageRestitutionCoefficient() const { return 0.0; };

  /** @brief Set the coefficient of restitution for the scene collision
   * geometry. See @ref staticStageObject_. By default does nothing since
   * kinametic scenes have no dynamics.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  virtual void setStageRestitutionCoefficient(
      CORRADE_UNUSED const double restitutionCoefficient){};

  // ============ Object Transformation functions =============

  /** @brief Set the 4x4 transformation matrix of an object kinematically.
   * Calling this during simulation of a @ref MotionType::DYNAMIC object is not
   * recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param trans The desired 4x4 transform of the object.
   */
  void setTransformation(const int physObjectID, const Magnum::Matrix4& trans);

  /** @brief Set the @ref esp::core::RigidState of an object kinematically.
   * Calling this during simulation of a @ref MotionType::DYNAMIC object is not
   * recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param trans The desired @ref esp::core::RigidState of the object.
   */
  void setRigidState(const int physObjectID,
                     const esp::core::RigidState& rigidState);

  /** @brief Set the 3D position of an object kinematically.
   * Calling this during simulation of a @ref MotionType::DYNAMIC object is not
   * recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param vector The desired 3D position of the object.
   */
  void setTranslation(const int physObjectID, const Magnum::Vector3& vector);

  /** @brief Set the orientation of an object kinematically.
   * Calling this during simulation of a @ref MotionType::DYNAMIC object is not
   * recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param quaternion The desired orientation of the object.
   */
  void setRotation(const int physObjectID,
                   const Magnum::Quaternion& quaternion);

  /** @brief Reset the transformation of the object.
   * !!NOT IMPLEMENTED!!
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   */
  void resetTransformation(const int physObjectID);

  /** @brief Modify the 3D position of an object kinematically by translation.
   * Calling this during simulation of a @ref MotionType::DYNAMIC object is not
   * recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param vector The desired 3D vector by which to translate the object.
   */
  void translate(const int physObjectID, const Magnum::Vector3& vector);

  /** @brief Modify the 3D position of an object kinematically by translation
   * with a vector defined in the object's local coordinate system. Calling this
   * during simulation of a @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param vector The desired 3D vector in the object's ocal coordiante system
   * by which to translate the object.
   */
  void translateLocal(const int physObjectID, const Magnum::Vector3& vector);

  /** @brief Modify the orientation of an object kinematically by applying an
   * axis-angle rotation to it. Calling this during simulation of a @ref
   * MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   * @param normalizedAxis The desired unit vector axis of rotation.
   */
  void rotate(const int physObjectID,
              const Magnum::Rad angleInRad,
              const Magnum::Vector3& normalizedAxis);

  /** @brief Modify the orientation of an object kinematically by applying an
   * axis-angle rotation to it in the local coordinate system. Calling this
   * during simulation of a @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   * @param normalizedAxis The desired unit vector axis of rotation in the local
   * coordinate system.
   */
  void rotateLocal(const int physObjectID,
                   const Magnum::Rad angleInRad,
                   const Magnum::Vector3& normalizedAxis);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the global X axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateX(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the global Y axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateY(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the global Z axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateZ(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the local X axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateXLocal(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the local Y axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateYLocal(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Modify the orientation of an object kinematically by applying a
   * rotation to it about the local Z axis. Calling this during simulation of a
   * @ref MotionType::DYNAMIC object is not recommended.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angleInRad The angle of rotation in radians.
   */
  void rotateZLocal(const int physObjectID, const Magnum::Rad angleInRad);

  /** @brief Get the current 4x4 transformation matrix of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The 4x4 transform of the object.
   */
  Magnum::Matrix4 getTransformation(const int physObjectID) const;

  /** @brief Get the current @ref esp::core::RigidState of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The @ref esp::core::RigidState of the object.
   */
  esp::core::RigidState getRigidState(const int objectID) const;

  /** @brief Get the current 3D position of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The 3D position of the object.
   */
  Magnum::Vector3 getTranslation(const int physObjectID) const;

  /** @brief Get the current orientation of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return A quaternion representation of the object's orientation.
   */
  Magnum::Quaternion getRotation(const int physObjectID) const;

  // ============ Object Setter functions =============
  // Setters that interface with physics need to take

  /** @brief Set the mass of an object.
   * See @ref RigidObject::setMass.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param mass The new mass of the object.
   */
  void setMass(const int physObjectID, const double mass);

  /** @brief Set the center of mass (COM) of an object.
   * Warning: some physics implementations require that object origins coincide
   * with thier COM. See @ref BulletRigidObject. Therefore, be careful modifying
   * this value. See @ref RigidObject::setCOM.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param COM The new 3D center of mass for the object in the local coordinate
   * system.
   */
  void setCOM(const int physObjectID, const Magnum::Vector3& COM);

  /** @brief Set the diagonal of the inertia matrix for an object.
   * If an object is aligned with its principle axii of inertia, the 3x3 inertia
   * matrix can be reduced to a diagonal. See @ref
   * RigidObject::setInertiaVector.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param inertia The new diagonal for the object's inertia matrix.
   */
  void setInertiaVector(const int physObjectID, const Magnum::Vector3& inertia);

  /** @brief Set the scalar friction coefficient for an object.
   * See @ref RigidObject::setFrictionCoefficient.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param frictionCoefficient The new scalar coefficient of friction for the
   * object.
   */
  void setFrictionCoefficient(const int physObjectID,
                              const double frictionCoefficient);

  /** @brief Set the scalar coefficient of restitution for an object.
   * See @ref RigidObject::setRestitutionCoefficient.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param restitutionCoefficient The new scalar coefficient of restitution for
   * the object.
   */
  void setRestitutionCoefficient(const int physObjectID,
                                 const double restitutionCoefficient);

  /** @brief Set the scalar linear damping coefficient for an object.
   * See @ref RigidObject::setLinearDamping.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param linDamping The new scalar linear damping coefficient for the object.
   */
  void setLinearDamping(const int physObjectID, const double linDamping);

  /** @brief Set the scalar angular damping coefficient for an object.
   * See @ref RigidObject::setAngularDamping.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angDamping The new scalar angular damping coefficient for the
   * object.
   */
  void setAngularDamping(const int physObjectID, const double angDamping);

#ifdef ESP_BUILD_WITH_VHACD
  /** @brief Initializes a new VoxelWrapper with a boundary voxelization using
   * VHACD's voxelization libary and assigns it to a rigid body.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param resolution Represents the approximate number of voxels in the new
   * voxelization.
   */
  void generateVoxelization(const int physObjectID,
                            const int resolution = 1000000);

  /** @brief Initializes a new VoxelWrapper with a boundary voxelization using
   * VHACD's voxelization libary and assigns it to the stage's rigid body.
   * @param resolution Represents the approximate number of voxels in the new
   * voxelization.
   */
  void generateStageVoxelization(const int resolution = 1000000);
#endif

  // ============ Object Getter functions =============

  /** @brief Get the mass of an object.
   * See @ref RigidObject::getMass.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Object mass or @ref esp::PHYSICS_ATTR_UNDEFINED if failed.
   */
  double getMass(const int physObjectID) const;

  /** @brief Get the center of mass (COM) of an object.
   * See @ref RigidObject::getCOM.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Object 3D center of mass in the local coordinate system.
   */
  Magnum::Vector3 getCOM(const int physObjectID) const;

  /** @brief Get the diagnoal vector of the inertia matrix of an object.
   * See @ref RigidObject::getInertiaVector.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The diagnoal vector of the inertia matrix of the object.
   */
  Magnum::Vector3 getInertiaVector(const int physObjectID) const;

  /** @brief Get the 3x3 inertia matrix of an object.
   * See @ref RigidObject::getInertiaMatrix.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The 3x3 inertia matrix of the object.
   */
  Magnum::Matrix3 getInertiaMatrix(const int physObjectID) const;

  /** @brief Get the scale of an object set during initialization.
   * See @ref RigidObject::getScale.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scaling of the object relative to its initialy loaded meshes.
   */
  Magnum::Vector3 getScale(const int physObjectID) const;

  /** @brief Get the scalar coefficient of friction of an object.
   * See @ref RigidObject::getFrictionCoefficient.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar coefficient of friction of the object.
   */
  double getFrictionCoefficient(const int physObjectID) const;

  /** @brief Get the scalar coefficient of restitution of an object.
   * See @ref RigidObject::getRestitutionCoefficient.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar coefficient of restitution of the object.
   */
  double getRestitutionCoefficient(const int physObjectID) const;

  /** @brief Get the scalar linear damping coefficient of an object.
   * See @ref RigidObject::getLinearDamping.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar linear damping coefficient of the object.
   */
  double getLinearDamping(const int physObjectID) const;

  /** @brief Get the scalar angular damping coefficient of an object.
   * See @ref RigidObject::getAngularDamping.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar angular damping coefficient of the object
   */
  double getAngularDamping(const int physObjectID) const;

  /** @brief Gets the VoxelWrapper associated with a rigid object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return A pointer to the object's Voxel Wrapper.
   */
  std::shared_ptr<esp::geo::VoxelWrapper> getObjectVoxelization(
      const int physObjectID) const;

  /** @brief Gets the VoxelWrapper associated with the scene.
   * @return A pointer to the scene's Voxel Wrapper.
   */
  std::shared_ptr<esp::geo::VoxelWrapper> getStageVoxelization() const;

  // ============= Platform dependent function =============

  /** @brief Get the scalar collision margin of an object.
   * See @ref BulletRigidObject::getMargin.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The scalar collision margin of the object.
   */
  virtual double getMargin(CORRADE_UNUSED const int physObjectID) const {
    return 0.0;
  };

  /** @brief Set the scalar collision margin of an object.
   * See @ref BulletRigidObject::setMargin. Nothing is set if no implementation
   * using a collision margin is in use.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param  margin The desired collision margin for the object.
   */
  virtual void setMargin(CORRADE_UNUSED const int physObjectID,
                         CORRADE_UNUSED const double margin){};

  // =========== Debug functions ===========

  /** @brief Get the number of objects in @ref PhysicsManager::existingObjects_
   * considered active by the physics simulator currently in use. See @ref
   * RigidObject::isActive.
   * @return  The number of active @ref RigidObject instances.
   */
  int checkActiveObjects();

  /** @brief True if the object is considered active by the simulator physics
   * simulator currently in use. See @ref RigidObject::isActive.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return  Whether or not the object is active.
   */
  bool isObjectAwake(const int objectID) const;

  /**
   * @brief Set the object to sleep or wake.
   */
  void setObjectSleep(const int objectID, bool sleep);

  //============ Interact with objects =============
  // NOTE: engine specifics handled by objects themselves...

  /** @brief Apply a linear 3D force defined in global coordinates to an object.
   * See @ref RigidObject::applyForce.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param force The linear 3D force to apply to the object.
   * @param relPos The global 3D location relative to the object COM at which to
   * apply the force.
   */
  void applyForce(const int physObjectID,
                  const Magnum::Vector3& force,
                  const Magnum::Vector3& relPos);

  /** @brief Apply a linear 3D impulse defined in global coordinates to an
   * object. See @ref RigidObject::applyImpulse. Impulse is applied instantly to
   * modify object velocity (i.e., not integrated through dynamic equations).
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param impulse The linear 3D impulse to apply to the object.
   * @param relPos The global 3D location relative to the object COM at which to
   * apply the impulse.
   */
  void applyImpulse(const int physObjectID,
                    const Magnum::Vector3& impulse,
                    const Magnum::Vector3& relPos);

  /** @brief Apply an internal angular 3D torque to an object.
   * See @ref RigidObject::applyTorque.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param torque The angular torque to apply to the object.
   */
  void applyTorque(const int physObjectID, const Magnum::Vector3& torque);

  /** @brief Apply an internal angular 3D impulse torque to an object.
   * See @ref RigidObject::applyImpulseTorque.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param impulse The angular impulse torque to apply to the object.
   */
  void applyImpulseTorque(const int physObjectID,
                          const Magnum::Vector3& impulse);

  /**
   * @brief Set linear velocity for an object with @ref MotionType::DYNAMIC.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param linVel Linear velocity to set.
   */
  void setLinearVelocity(const int physObjectID, const Magnum::Vector3& linVel);

  /**
   * @brief Set angular velocity for an object with @ref MotionType::DYNAMIC.
   *
   * Does nothing for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param angVel Angular velocity vector corresponding to world unit axis
   * angles.
   */
  void setAngularVelocity(const int physObjectID,
                          const Magnum::Vector3& angVel);

  /**
   * @brief Get linear velocity of an object with @ref MotionType::DYNAMIC.
   *
   * Always zero for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Linear velocity of the object.
   */
  Magnum::Vector3 getLinearVelocity(const int physObjectID) const;

  /**
   * @brief Get angular velocity of an object with @ref MotionType::DYNAMIC.
   *
   * Always zero for @ref MotionType::KINEMATIC or @ref MotionType::STATIC
   * objects.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Angular velocity vector corresponding to world unit axis angles.
   */
  Magnum::Vector3 getAngularVelocity(const int physObjectID) const;

  /**@brief Retrieves a shared pointer to the VelocityControl struct for this
   * object.
   */
  VelocityControl::ptr getVelocityControl(const int physObjectID);

  /** @brief Set bounding box rendering for the object true or false.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param drawables The drawables group with which to render the bounding box.
   * @param drawBB Set rendering of the bounding box to true or false.
   */
  void setObjectBBDraw(int physObjectID, DrawableGroup* drawables, bool drawBB);

  /** @brief Set the voxelization visualization for the object true or false.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param gridName The voxel grid to be visualized.
   * @param drawables The drawables group with which to render the voxelization.
   * @param drawVoxelization Set rendering of the voxelization to true or false.
   */
  void setObjectVoxelizationDraw(int physObjectID,
                                 const std::string& gridName,
                                 DrawableGroup* drawables,
                                 bool drawVoxelization);

  /** @brief Set the voxelization visualization for the scene true or false.
   * @param gridName The voxel grid to be visualized.
   * @param drawables The drawables group with which to render the voxelization.
   * @param drawVoxelization Set rendering of the voxelization to true or false.
   */
  void setStageVoxelizationDraw(const std::string& gridName,
                                DrawableGroup* drawables,
                                bool drawVoxelization);

  /**
   * @brief Get a const reference to the specified object's SceneNode for info
   * query purposes.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_ or @ref
   * PhysicsManager::existingArticulatedObjects_.
   * @return Const reference to the object scene node.
   */
  const scene::SceneNode& getObjectSceneNode(int physObjectID) const;

  /** @overload */
  scene::SceneNode& getObjectSceneNode(int physObjectID);

  /**
   * @brief Get a const reference to the specified ArticulatedLink SceneNode for
   * info query purposes. Default (-1) returns baseLink SceneNode.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingArticulatedObjects_.
   * @param linkId The ArticulatedLink ID or -1 for the base.
   * @return Const reference to the object scene node.
   */
  const scene::SceneNode& getArticulatedLinkSceneNode(int physObjectID,
                                                      int linkId = -1) const;

  /** @overload */
  scene::SceneNode& getArticulatedLinkSceneNode(int physObjectID,
                                                int linkId = -1);

  /**
   * @brief Get a const reference to the specified object's visual SceneNode for
   * info query purposes.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Const reference to the object's visual scene node.
   */
  const scene::SceneNode& getObjectVisualSceneNode(int physObjectID) const;

  /**
   * @brief Get pointers to an object's visual SceneNodes.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return pointers to the object's visual scene nodes.
   */
  std::vector<scene::SceneNode*> getObjectVisualSceneNodes(
      const int objectID) const;

  /**
   * @brief Get pointers to an ArticulatedLink's visual SceneNodes.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingArticulatedObjects_.
   * @return pointers to an ArticulatedLink's visual scene nodes.
   */
  std::vector<scene::SceneNode*> getArticulatedLinkVisualSceneNodes(
      const int objectID,
      const int linkID = -1) const;

  /**
   * @brief Set the desired light setup by name for the passed object
   * @param objectID The id of the object to set
   * @param lightSetupKey The string name of the desired lighting setup to use.
   */
  void setObjectLightSetup(const int objectID,
                           const std::string& lightSetupKey) {
    existingObjects_.at(objectID)->setLightSetup(lightSetupKey);
  }

  /** @brief Render any debugging visualizations provided by the underlying
   * physics simulator implementation. By default does nothing. See @ref
   * BulletPhysicsManager::debugDraw.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  virtual void debugDraw(
      CORRADE_UNUSED const Magnum::Matrix4& projTrans) const {};

  /**
   * @brief Check whether an object is in contact with any other objects or the
   * scene.
   *
   * Not implemented for default @ref PhysicsManager. See @ref
   * BulletPhysicsManager.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param staticAsStage When false, override configured collision groups|masks
   * for STATIC objects and articulated fixed base such that contact with other
   * STATICs such as the stage are considered.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  virtual bool contactTest(CORRADE_UNUSED const int physObjectID,
                           CORRADE_UNUSED bool staticAsStage = true) {
    return false;
  };

  /**
   * @brief Perform discrete collision detection for the scene with the derived
   * PhysicsManager implementation. Not implemented for default @ref
   * PhysicsManager. See @ref BulletPhysicsManager.
   */
  virtual void performDiscreteCollisionDetection(){
      /*Does nothing in base PhysicsManager.*/};

  virtual std::vector<ContactPointData> getContactPoints() const { return {}; }

  /**
   * @brief Manually set the collision group for an object.
   */
  virtual void overrideCollisionGroup(CORRADE_UNUSED const int physObjectID,
                                      CORRADE_UNUSED CollisionGroup
                                          group) const {}

  /**
   * @brief Set an object to collidable or not.
   */
  void setObjectIsCollidable(const int physObjectID, bool collidable) {
    assertIDValidity(physObjectID);
    existingObjects_.at(physObjectID)->setCollidable(collidable);
  };

  /**
   * @brief Get whether or not an object is collision active.
   */
  bool getObjectIsCollidable(const int physObjectID) {
    assertIDValidity(physObjectID);
    return existingObjects_.at(physObjectID)->getCollidable();
  };

  /**
   * @brief Set the stage to collidable or not.
   */
  void setStageIsCollidable(bool collidable) {
    staticStageObject_->setCollidable(collidable);
  };

  /**
   * @brief Get whether or not the stage is collision active.
   */
  bool getStageIsCollidable() { return staticStageObject_->getCollidable(); };

  /** @brief Return the library implementation type for the simulator currently
   * in use. Use to check for a particular implementation.
   * @return The implementation type of this simulator.
   */
  const PhysicsSimulationLibrary& getPhysicsSimulationLibrary() const {
    return activePhysSimLib_;
  };

  /**
   * @brief Set the @ref esp::scene::SceneNode::semanticId_ for all visual nodes
   * belonging to an object.
   *
   * @param objectID The object ID and key identifying the object in @ref
   * existingObjects_.
   * @param semanticId The desired semantic id for the object.
   */
  void setSemanticId(int physObjectID, uint32_t semanticId);

  /**
   * @brief Get a copy of the template used to initialize an object.
   *
   * @return The initialization settings of the specified object instance.
   */
  metadata::attributes::ObjectAttributes::ptr getObjectInitAttributes(
      const int physObjectID) const {
    assertIDValidity(physObjectID);
    return existingObjects_.at(physObjectID)->getInitializationAttributes();
  }

  /**
   * @brief Get a copy of the template used to initialize the stage.
   *
   * @return The initialization settings of the stage or nullptr if the stage is
   * not initialized.
   */
  metadata::attributes::StageAttributes::ptr getStageInitAttributes() const {
    return staticStageObject_->getInitializationAttributes();
  }

  /**
   * @brief Get a copy of the template used to initialize this physics manager
   *
   * @return The initialization settings for this physics manager
   */
  metadata::attributes::PhysicsManagerAttributes::ptr
  getInitializationAttributes() const {
    return metadata::attributes::PhysicsManagerAttributes::create(
        *physicsManagerAttributes_.get());
  }

  /**
   * @brief Cast a ray into the collision world and return a @ref RaycastResults
   * with hit information.
   *
   * Note: not implemented here in default PhysicsManager as there are no
   * collision objects without a simulation implementation.
   *
   * @param ray The ray to cast. Need not be unit length, but returned hit
   * distances will be in units of ray length.
   * @param maxDistance The maximum distance along the ray direction to search.
   * In units of ray length.
   * @return The raycast results sorted by distance.
   */
  virtual RaycastResults castRay(const esp::geo::Ray& ray,
                                 CORRADE_UNUSED double maxDistance = 100.0) {
    RaycastResults results;
    results.ray = ray;
    return results;
  }

  Magnum::Vector3 getArticulatedLinkCOM(int objectId, int linkId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)
        ->getLink(linkId)
        .node()
        .translation();
  }

  core::RigidState getArticulatedLinkRigidState(int objectId, int linkId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)
        ->getLink(linkId)
        .getRigidState();
  }

  int getNumArticulatedLinks(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getNumLinks();
  }

  void addArticulatedLinkForce(int objectId,
                               int linkId,
                               Magnum::Vector3 force) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->addArticulatedLinkForce(linkId,
                                                                      force);
  }

  float getArticulatedLinkFriction(int objectId, int linkId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getArticulatedLinkFriction(
        linkId);
  }

  void setArticulatedLinkFriction(int objectId, int linkId, float friction) {
    existingArticulatedObjects_.at(objectId)->setArticulatedLinkFriction(
        linkId, friction);
  }

  void setArticulatedObjectRootState(int objectId,
                                     const Magnum::Matrix4& state) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setRootState(state);
  };

  const Magnum::Matrix4 getArticulatedObjectRootState(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getRootState();
  };

  void setArticulatedObjectForces(int objectId,
                                  const std::vector<float>& forces) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setForces(forces);
  };

  void setArticulatedObjectVelocities(int objectId,
                                      const std::vector<float>& vels) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setVelocities(vels);
  };

  void setArticulatedObjectPositions(int objectId,
                                     const std::vector<float>& positions) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setPositions(positions);
  };

  std::vector<float> getArticulatedObjectPositionLimits(
      int objectId,
      bool upperLimits = false) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getPositionLimits(
        upperLimits);
  };

  void setAutoClampJointLimits(int objectId, bool autoClamp) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setAutoClampJointLimits(
        autoClamp);
  };

  bool getAutoClampJointLimits(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getAutoClampJointLimits();
  };
  std::vector<float> getArticulatedObjectPositions(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getPositions();
  };

  std::vector<float> getArticulatedObjectVelocities(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getVelocities();
  };

  std::vector<float> getArticulatedObjectForces(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getForces();
  };

  void resetArticulatedObject(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->reset();
  };

  void setArticulatedObjectSleep(int objectId, bool sleep) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setSleep(sleep);
  };

  bool getArticulatedObjectSleep(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getSleep();
  }

  void setArticulatedObjectMotionType(int objectId, MotionType mt) {
    CHECK(existingArticulatedObjects_.count(objectId));
    existingArticulatedObjects_.at(objectId)->setMotionType(mt);
  };

  MotionType getArticulatedObjectMotionType(int objectId) {
    CHECK(existingArticulatedObjects_.count(objectId));
    return existingArticulatedObjects_.at(objectId)->getMotionType();
  };

  virtual int getNumActiveContactPoints() { return -1; }
  virtual int getNumActiveOverlappingPairs() { return -1; }
  virtual std::string getStepCollisionSummary() { return "not implemented"; }

 protected:
  /** @brief Check that a given object ID is valid (i.e. it refers to an
   * existing object). Terminate the program and report an error if not. This
   * function is intended to unify object ID checking for @ref PhysicsManager
   * functions.
   * @param physObjectID The object ID to validate.
   */
  virtual void assertIDValidity(const int physObjectID) const {
    CHECK(existingObjects_.count(physObjectID) > 0);
  };

  /** @brief Check if a particular mesh can be used as a collision mesh for a
   * particular physics implemenation. Always True for base @ref PhysicsManager
   * class, since the mesh has already been successfully loaded by @ref
   * esp::assets::ResourceManager.
   * @param meshData The mesh to validate.
   * @return true if valid, false otherwise.
   */
  virtual bool isMeshPrimitiveValid(const assets::CollisionMeshData& meshData);

  /** @brief Acquire a new ObjectID by recycling the ID of an object removed
   * with @ref removeObject or by incrementing @ref nextObjectID_. See @ref
   * addObject.
   * @return The newly allocated ObjectID.
   */
  int allocateObjectID();

  /** @brief Recycle the ID of an object removed with @ref removeObject by
   * adding it to the list of available IDs: @ref recycledObjectIDs_.
   * @param physObjectID The ID to recycle.
   * @return The recycled object ID.
   */
  int deallocateObjectID(int physObjectID);

  /**
   * @brief Finalize physics initialization. Setup staticStageObject_ and
   * initialize any other physics-related values for physics-based scenes.
   * Overidden by instancing class if physics is supported.
   */
  virtual bool initPhysicsFinalize();

  /**
   * @brief Finalize stage initialization for kinematic stage.  Overidden by
   * instancing class if physics is supported.
   *
   * @param initAttributes the attributes structure defining physical
   * properties of the stage.
   * @return true if successful and false otherwise
   */

  virtual bool addStageFinalize(
      const metadata::attributes::StageAttributes::ptr& initAttributes);

  /** @brief Create and initialize a @ref RigidObject, assign it an ID and add
   * it to existingObjects_ map keyed with newObjectID
   * @param newObjectID valid object ID for the new object
   * @param initAttributes The physical object's template defining its
   * physical parameters.
   * @param objectNode Valid, existing scene node
   * @return whether the object has been successfully initialized and added to
   * existingObjects_ map
   */
  virtual bool makeAndAddRigidObject(
      int newObjectID,
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      scene::SceneNode* objectNode);

  /** @brief Set the voxelization visualization for a scene node to be true or
   * false.
   * @param gridName The name of the grid to be drawn.
   * @param rigidBase The rigidBase of the object or scene.
   * @param drawables The drawables group with which to render the voxelization.
   * @param drawVoxelization Set rendering of the voxelization to true or false.
   */
  void setVoxelizationDraw(const std::string& gridName,
                           esp::physics::RigidBase* rigidBase,
                           DrawableGroup* drawables,
                           bool drawVoxelization);

  /** @brief A reference to a @ref esp::assets::ResourceManager which holds
   * assets that can be accessed by this @ref PhysicsManager*/
  assets::ResourceManager& resourceManager_;

  //! URDF importer implementation and model cache.
  std::unique_ptr<URDFImporter> urdfImporter_;

  /** @brief A pointer to the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes describing
   * this physics manager */
  const metadata::attributes::PhysicsManagerAttributes::cptr
      physicsManagerAttributes_;

  /** @brief The current physics library implementation used by this
   * @ref PhysicsManager. Can be used to correctly cast the @ref PhysicsManager
   * to its derived type if necessary.*/
  PhysicsSimulationLibrary activePhysSimLib_ = NONE;  // default

  /**
   * @brief The @ref scene::SceneNode which is the parent of all members of the
   * scene graph which exist in the physical world. Used to keep track of all
   * SceneNode's that have physical properties.
   * */
  scene::SceneNode* physicsNode_ = nullptr;

  /**
   * @brief The @ref scene::SceneNode which represents the static collision
   * geometry of the physical world. Only one @ref staticStageObject_ may
   * exist in a physical world. This @ref RigidStage can only have @ref
   * MotionType::STATIC as it is loaded as static geometry with simulation
   * efficiency in mind. See
   * @ref addStage.
   * */
  physics::RigidStage::uptr staticStageObject_ = nullptr;

  //! ==== Rigid object memory management ====

  /** @brief Maps object IDs to all existing physical object instances in the
   * world.
   */
  std::map<int, RigidObject::uptr> existingObjects_;

  // TODO: should these be separate maps or somehow combined? What about ids?
  /** @brief Maps articulated object IDs to all existing physical object
   * instances in the world.
   */
  std::map<int, ArticulatedObject::uptr> existingArticulatedObjects_;

  /** @brief A counter of unique object ID's allocated thus far. Used to
   * allocate new IDs when  @ref recycledObjectIDs_ is empty without needing to
   * check @ref existingObjects_ explicitly.*/
  int nextObjectID_ = 0;

  /** @brief A list of available object IDs tracked by @ref deallocateObjectID
   * which were previously used by objects since removed from the world with
   * @ref removeObject. These IDs will be re-allocated with @ref
   * allocateObjectID before new IDs are acquired with @ref nextObjectID_. */
  std::vector<int> recycledObjectIDs_;

  //! Utilities

  /** @brief Tracks whether or not this @ref PhysicsManager has already been
   * initialized with @ref initPhysics. */
  bool initialized_ = false;

  /** @brief The fixed amount of time over which to integrate the simulation in
   * discrete steps within @ref stepPhysics. Lower values result in better
   * stability at the cost of worse efficiency and vice versa. */
  double fixedTimeStep_ = 1.0 / 240.0;

  /** @brief The current simulation time. Tracks the total amount of time
   * simulated with @ref stepPhysics up to this point. */
  double worldTime_ = 0.0;

  ESP_SMART_POINTERS(PhysicsManager)
};

}  // namespace physics

}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSMANAGER_H_
