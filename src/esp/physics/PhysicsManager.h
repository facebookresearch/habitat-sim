// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSMANAGER_H_
#define ESP_PHYSICS_PHYSICSMANAGER_H_

/** @file
 * @brief Class @ref PhysicsManager, enum @ref
 * PhysicsManager::PhysicsSimulationLibrary
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
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/assets/MeshMetaData.h"
#include "esp/gfx/DrawableGroup.h"
#include "esp/metadata/URDFParser.h"
#include "esp/physics/objectWrappers/ManagedArticulatedObject.h"
#include "esp/physics/objectWrappers/ManagedRigidObject.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace assets {
class ResourceManager;
}
//! core physics simulation namespace
namespace sim {
class Simulator;
}
namespace metadata {
namespace attributes {
class PhysicsManagerAttributes;
}
}  // namespace metadata

namespace physics {

/** @brief Holds information about one ray hit instance. */
struct RayHitInfo {
  /** @brief The id of the object hit by this ray. Stage hits are -1. */
  int objectId{};

  /** @brief  The first impact point of the ray in world space. */
  Magnum::Vector3 point;

  /** @brief The collision object normal at the point of impact. */
  Magnum::Vector3 normal;

  /** @brief  Distance along the ray direction from the ray origin (in units of
   * ray length). */
  double rayDistance{};

  ESP_SMART_POINTERS(RayHitInfo)
};

/** @brief Holds information about all ray hit instances from a ray cast. */
struct RaycastResults {
  std::vector<RayHitInfo> hits;
  esp::geo::Ray ray;

  bool hasHits() const { return hits.size() > 0; }

  void sortByDistance() {
    std::sort(hits.begin(), hits.end(),
              [](const RayHitInfo& A, const RayHitInfo& B) {
                return A.rayDistance < B.rayDistance;
              });
  }

  ESP_SMART_POINTERS(RaycastResults)
};

/** @brief based on Bullet b3ContactPointData */
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

/** @brief describes the type of a rigid constraint.*/
enum class RigidConstraintType {
  /** @brief lock a point in one frame to a point in another with no orientation
   * constraint
   */
  PointToPoint,
  /** @brief fix one frame to another constraining relative position and
   * orientation
   */
  Fixed
};  // enum class RigidConstraintType

/** @brief Stores rigid constraint parameters for creation and updates.*/
struct RigidConstraintSettings {
 public:
  RigidConstraintSettings() = default;

  /** @brief The type of constraint described by these settings. Determines
   * which parameters to use for creation and update.
   */
  RigidConstraintType constraintType = RigidConstraintType::PointToPoint;

  /** @brief The maximum impulse applied by this constraint. Should be tuned
   * relative to physics timestep.
   */
  double maxImpulse = 1000.0;

  /** @brief objectIdA must always be >= 0. For mixed type constraints, objectA
   * must be the ArticulatedObject.
   */
  int objectIdA = ID_UNDEFINED;

  /** @brief objectIdB == ID_UNDEFINED indicates "world". */
  int objectIdB = ID_UNDEFINED;

  /** @brief  link of objectA if articulated. ID_UNDEFINED(-1) refers to base.
   */
  int linkIdA = ID_UNDEFINED;

  /** @brief link of objectB if articulated. ID_UNDEFINED(-1) refers to base.*/
  int linkIdB = ID_UNDEFINED;

  /** @brief constraint point in local space of respective objects*/
  Mn::Vector3 pivotA{}, pivotB{};

  /** @brief  constraint orientation frame in local space of respective objects
   * for RigidConstraintType::Fixed
   */
  Mn::Matrix3x3 frameA{}, frameB{};

  ESP_SMART_POINTERS(RigidConstraintSettings)
};  // struct RigidConstraintSettings

class RigidObjectManager;
class ArticulatedObjectManager;

/**
 * @brief Kinematic and dynamic scene and object manager. Responsible for
 * tracking, updating, and synchronizing the state of the physical world and all
 * non-static geometry in the scene as well as interfacing with specific
 * physical simulation implementations.
 *
 * The physical world in this case consists of any objects which can be
 * manipulated:addObject : (kinematically or dynamically) or simulated and
 * anything such objects must be aware of (e.g. static scene collision
 * geometry).
 *
 * May eventually manage multiple physical scenes, but currently
 * assumes only one unique physical world can exist.
 */
class PhysicsManager : public std::enable_shared_from_this<PhysicsManager> {
 public:
  //! ==== physics engines ====

  /**
   * @brief The specific physics implementation used by the current @ref
   * PhysicsManager. Each entry suggests a derived class of @ref PhysicsManager
   * and @ref RigidObject implementing the specific interface to a simulation
   * library.
   */
  enum class PhysicsSimulationLibrary {

    /**
     * The default implemenation of kineamtics through the base @ref
     * PhysicsManager class. Supports @ref MotionType::STATIC and
     * @ref MotionType::KINEMATIC objects of base class @ref
     * RigidObject. If the derived @ref PhysicsManager class for a desired @ref
     * PhysicsSimulationLibrary fails to initialize, it will default to @ref
     * PhysicsSimulationLibrary::NoPhysics.
     */
    NoPhysics,

    /**
     * An implemenation of dynamics through the Bullet Physics library.
     * Supports @ref MotionType::STATIC, @ref
     * MotionType::KINEMATIC, and @ref
     * MotionType::DYNAMIC objects of @ref RigidObject derived
     * class @ref BulletRigidObject. Suggests the use of @ref PhysicsManager
     * derived class
     * @ref BulletPhysicsManager
     */
    Bullet
  };

  /**
   * @brief Construct a #ref PhysicsManager with access to specific resource
   * assets.
   *
   * @param _resourceManager The @ref esp::assets::ResourceManager which
   * tracks the assets this
   * @param _physicsManagerAttributes The PhysicsManagerAttributes template used
   * to instantiate this physics manager.
   * @ref PhysicsManager will have access to.
   */
  explicit PhysicsManager(
      assets::ResourceManager& _resourceManager,
      const std::shared_ptr<
          const metadata::attributes::PhysicsManagerAttributes>&
          _physicsManagerAttributes);

  /** @brief Destructor*/
  virtual ~PhysicsManager();

  /**
   * @brief Set a pointer to this physics manager's owning simulator.
   * */
  void setSimulator(esp::sim::Simulator* _simulator) {
    simulator_ = _simulator;
  }

  /**
   * @brief Initialization: load physical properties and setup the world.
   * @param node  The scene graph node which will act as the parent of all
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
   * contain several components (e.g. GLB hierarchy).
   *
   * @param initAttributes The attributes structure defining physical
   * properties of the scene.  Must be a copy of the attributes stored in the
   * Attributes Manager.
   * @param stageInstanceAttributes The stage instance attributes that was used
   * to create this stage. Might be empty.
   * @return true if successful and false otherwise
   */
  bool addStage(
      const metadata::attributes::StageAttributes::ptr& initAttributes,
      const metadata::attributes::SceneObjectInstanceAttributes::cptr&
          stageInstanceAttributes);

  /**
   * @brief Instance and place a physics object from a @ref
   * esp::metadata::attributes::SceneObjectInstanceAttributes file.
   * @param objInstAttributes The attributes that describe the desired state to
   * set this object.
   * @param attributesHandle The handle of the object attributes used as the key
   * to query @ref esp::metadata::managers::ObjectAttributesManager.
   * @param defaultCOMCorrection The default value of whether COM-based
   * translation correction needs to occur.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObjectInstance(
      const esp::metadata::attributes::SceneObjectInstanceAttributes::cptr&
          objInstAttributes,
      const std::string& attributesHandle,
      bool defaultCOMCorrection = false,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager.  This method
   * will query for a drawable group from simulator.
   *
   * @param attributesHandle The handle of the object attributes used as the key
   * to query @ref esp::metadata::managers::ObjectAttributesManager.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(const std::string& attributesHandle,
                scene::SceneNode* attachmentNode = nullptr,
                const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::metadata::managers::ObjectAttributesManager by template
   * ID.  This method will query for a drawable group from simulator.
   *
   * @param attributesID The ID of the object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(int attributesID,
                scene::SceneNode* attachmentNode = nullptr,
                const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /** @brief Queries simulator for drawables, if simulator exists, otherwise
   * passes nullptr, before instancing a physical object from an object
   * properties template in the @ref
   * esp::metadata::managers::ObjectAttributesManager by template handle.
   * @param objectAttributes The object's template in @ref
   * esp::metadata::managers::ObjectAttributesManager.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObjectQueryDrawables(
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /** @brief Instance a physical object from an ObjectAttributes template.
   * @param objectAttributes The object's template to use to instantiate the
   * object.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   * @param attachmentNode If supplied, attach the new physical object to an
   * existing SceneNode.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      DrawableGroup* drawables,
      scene::SceneNode* attachmentNode = nullptr,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Create an object wrapper appropriate for this physics manager.
   * Overridden if called by dynamics-library-enabled PhysicsManager
   */
  virtual ManagedRigidObject::ptr getRigidObjectWrapper();

  /** @brief Remove an object instance from the pysical scene by ID, destroying
   * its scene graph node and removing it from @ref
   * PhysicsManager::existingObjects_.
   *  @param objectId The ID (key) of the object instance in @ref
   * PhysicsManager::existingObjects_.
   * @param deleteObjectNode If true, deletes the object's scene node. Otherwise
   * detaches the object from simulation.
   * @param deleteVisualNode If true, deletes the object's visual node.
   * Otherwise detaches the object from simulation. Is not considered if
   * deleteObjectNode==true.
   */
  virtual void removeObject(int objectId,
                            bool deleteObjectNode = true,
                            bool deleteVisualNode = true);

  /** @brief Get the number of objects mapped in @ref
   * PhysicsManager::existingObjects_.
   *  @return The size of @ref PhysicsManager::existingObjects_.
   */
  int getNumRigidObjects() const { return existingObjects_.size(); }

  /** @brief Get a list of existing object IDs (i.e., existing keys in @ref
   * PhysicsManager::existingObjects_.)
   *  @return List of object ID keys from @ref PhysicsManager::existingObjects_.
   */
  std::vector<int> getExistingObjectIDs() const {
    std::vector<int> v;
    v.reserve(existingObjects_.size());
    for (const auto& bro : existingObjects_) {
      v.push_back(bro.first);
    }
    return v;
  }

  //============= ArticulatedObject functions =============

  /**
   * @brief Instance and place an @ref ArticulatedObject from a @ref
   * esp::metadata::attributes::SceneAOInstanceAttributes file.
   * @param aObjInstAttributes The attributes that describe the desired initial
   * state to set for this articulated object.
   * @param attributesHandle The handle of the @ref ArticulatedObject attributes
   * used as the key to query @ref esp::metadata::managers::AOAttributesManager
   * for the attributes.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObjectInstance(
      const std::shared_ptr<
          const esp::metadata::attributes::SceneAOInstanceAttributes>&
          aObjInstAttributes,
      const std::string& attributesHandle,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Instance an @ref ArticulatedObject from an
   * @ref esp::metadata::attributes::ArticulatedObjectAttributes retrieved from the
   * @ref esp::metadata::managers::AOAttributesManager by the given
   * @p attributesHandle . This method calls @p
   * addArticulatedObjectQueryDrawables to provide drawables.
   *
   * @param attributesHandle The handle of the ArticulatedObjectAttributes to
   * use to create the desired @ref ArticulatedObject
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObject(
      const std::string& attributesHandle,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Instance an @ref ArticulatedObject from an
   * @ref esp::metadata::attributes::ArticulatedObjectAttributes retrieved from the
   * @ref esp::metadata::managers::AOAttributesManager by the given
   * @p attributesID . This method calls @p
   * addArticulatedObjectQueryDrawables to provide drawables.
   *
   * @param attributesID The ID of the ArticulatedObjectAttributes to
   * use to create the desired @ref ArticulatedObject
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObject(
      int attributesID,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Queries simulator for drawables, if simulator exists, otherwise
   * passes nullptr, before instancing an articulated object from an
   * @ref esp::metadata::attributes::ArticulatedObjectAttributes template
   *
   * @param artObjAttributes The @ref ArticulatedObject's template to use to create it.
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObjectQueryDrawables(
      const esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
          artObjAttributes,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * ArticulatedObject in the world based on the urdf filepath specified in @ref
   * esp::metadata::attributes::ArticulatedObjectAttributes. This version
   * requires drawables to be provided.
   *
   * Not implemented in base PhysicsManager.
   * @param artObjAttributes The @ref ArticulatedObject's template to use to create it.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized @ref ArticulatedObject.
   * @param forceReload If true, force the reload of the source URDF from file,
   * replacing the cached model if it exists.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  virtual int addArticulatedObject(
      CORRADE_UNUSED const
          esp::metadata::attributes::ArticulatedObjectAttributes::ptr&
              artObjAttributes,
      CORRADE_UNUSED DrawableGroup* drawables,
      CORRADE_UNUSED bool forceReload = false,
      CORRADE_UNUSED const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
    return ID_UNDEFINED;
  }

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * ArticulatedObject in the world.  This version will query an existing
   * simulator for drawables and therefore does not require drawables to be
   * specified.
   *
   * Not implemented in base PhysicsManager.
   * @param filepath The fully-qualified filename for the URDF file describing
   * the model the articulated object is to be built from.
   * @param fixedBase Whether the base of the @ref ArticulatedObject should be
   * fixed.
   * @param globalScale A scale multiplier to be applied uniformly in 3
   * dimensions to the entire @ref ArticulatedObject.
   * @param massScale A scale multiplier to be applied to the mass of the all
   * the components of the @ref ArticulatedObject.
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param maintainLinkOrder If true, maintain the order of link definitions
   * from the URDF file as the link indices.
   * @param intertiaFromURDF If true, load the link inertia matrices from the
   * URDF file instead of computing automatically from collision shapes.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObjectFromURDF(
      const std::string& filepath,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      bool maintainLinkOrder = false,
      bool intertiaFromURDF = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * ArticulatedObject in the world.
   *
   * Not implemented in base PhysicsManager.
   * @param filepath The fully-qualified filename for the URDF file describing
   * the model the articulated object is to be built from.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized @ref ArticulatedObject.
   * @param fixedBase Whether the base of the @ref ArticulatedObject should be
   * fixed.
   * @param globalScale A scale multiplier to be applied uniformly in 3
   * dimensions to the entire @ref ArticulatedObject.
   * @param massScale A scale multiplier to be applied to the mass of the all
   * the components of the @ref ArticulatedObject.
   * @param forceReload If true, reload the source URDF from file, replacing the
   * cached model.
   * @param maintainLinkOrder If true, maintain the order of link definitions
   * from the URDF file as the link indices.
   * @param intertiaFromURDF If true, load the link inertia matrices from the
   * URDF file instead of computing automatically from collision shapes.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return The instanced @ref ArticulatedObject 's ID, mapping to the articulated
   * object in @ref PhysicsManager::existingObjects_ if successful, or
   * @ref esp::ID_UNDEFINED. These values come from the same pool used
   * by rigid objects.
   */
  int addArticulatedObjectFromURDF(
      const std::string& filepath,
      DrawableGroup* drawables,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      bool maintainLinkOrder = false,
      bool intertiaFromURDF = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  //! Remove an @ref ArticulatedObject from the world by unique id.
  virtual void removeArticulatedObject(int objectId);

  //! Get the current number of instanced articulated objects in the world.
  int getNumArticulatedObjects() { return existingArticulatedObjects_.size(); }

  ArticulatedObject& getArticulatedObject(int objectId) {
    auto existAOIter = getArticulatedObjIteratorOrAssert(objectId);
    return *existAOIter->second;
  }

  /**
   * @brief Create an articulated object wrapper appropriate for this physics
   * manager. Overridden if called by dynamics-library-enabled PhysicsManager
   */
  virtual ManagedArticulatedObject::ptr getArticulatedObjectWrapper();

  /** @brief Get a list of existing object IDs for articulated objects (i.e.,
   * existing keys in @ref PhysicsManager::existingArticulatedObjects_.)
   *  @return List of object ID keys from @ref
   * PhysicsManager::existingArticulatedObjects_.
   */
  std::vector<int> getExistingArticulatedObjectIds() const {
    std::vector<int> v;
    v.reserve(existingArticulatedObjects_.size());
    for (const auto& bro : existingArticulatedObjects_) {
      v.push_back(bro.first);
    }
    return v;
  }

  //============ Simulator functions =============

  /** @brief Step the physical world forward in time. Time may only advance in
   * increments of @ref fixedTimeStep_.
   * @param dt The desired amount of time to advance the physical world.
   */
  virtual void stepPhysics(double dt = 0.0);

  /** @brief Defers the update of the scene graph nodes until updateNodes is
   * called This is needed to do ownership transfer of the scene graph to a
   * background thread.
   */
  virtual void deferNodesUpdate();

  /** @brief Syncs the state of physics simulation to the rendering scene graph.
   */
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
  virtual double getTimestep() const { return fixedTimeStep_; }

  /** @brief Get the current @ref worldTime_ of the physical world. See @ref
   * stepPhysics.
   * @return The amount of time, @ref worldTime_, by which the physical world
   * has advanced.
   */
  virtual double getWorldTime() const { return worldTime_; }

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
  virtual double getStageFrictionCoefficient() const { return 0.0; }

  /** @brief Set the friction coefficient of the scene collision geometry. See
   * @ref staticStageObject_.
   * @param frictionCoefficient The scalar friction coefficient of the scene
   * geometry.
   */
  virtual void setStageFrictionCoefficient(
      CORRADE_UNUSED const double frictionCoefficient) {}

  /** @brief Get the current coefficient of restitution for the scene collision
   * geometry. This determines the ratio of initial to final relative velocity
   * between the scene and collidiing object. See @ref staticStageObject_. By
   * default this will always return 0, since kinametic scenes have no dynamics.
   * @return The scalar coefficient of restitution for the scene geometry.
   */
  virtual double getStageRestitutionCoefficient() const { return 0.0; }

  /** @brief Set the coefficient of restitution for the scene collision
   * geometry. See @ref staticStageObject_. By default does nothing since
   * kinametic scenes have no dynamics.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  virtual void setStageRestitutionCoefficient(
      CORRADE_UNUSED const double restitutionCoefficient) {}

  // =========== Debug functions ===========

  /** @brief Get the number of objects in @ref PhysicsManager::existingObjects_
   * considered active by the physics simulator currently in use. See @ref
   * RigidObject::isActive.
   * @return  The number of active @ref RigidObject instances.
   */
  int checkActiveObjects();

  /** @brief Set bounding box rendering for the object true or false.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param drawables The drawables group with which to render the bounding box.
   * @param drawBB Set rendering of the bounding box to true or false.
   */
  void setObjectBBDraw(int physObjectID, DrawableGroup* drawables, bool drawBB);

  /**
   * @brief Get the root node of an object's visual SceneNode subtree.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The visual root node.
   */
  const scene::SceneNode& getObjectVisualSceneNode(int physObjectID) const {
    auto objIter = getConstRigidObjIteratorOrAssert(physObjectID);
    return *objIter->second->visualNode_;
  }

  /** @brief Render any debugging visualizations provided by the underlying
   * physics simulator implementation. By default does nothing. See @ref
   * BulletPhysicsManager::debugDraw.
   * @param projTrans The composed projection and transformation matrix for the
   * render camera.
   */
  virtual void debugDraw(
      CORRADE_UNUSED const Magnum::Matrix4& projTrans) const {}

  /**
   * @brief Check whether an object is in contact with any other objects or the
   * scene.
   *
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  virtual bool contactTest(const int physObjectID) {
    const auto existingObjsIter = existingObjects_.find(physObjectID);
    bool existingObjFound = (existingObjsIter != existingObjects_.end());
    const auto existingArtObjsIter =
        existingArticulatedObjects_.find(physObjectID);
    CORRADE_INTERNAL_ASSERT(
        existingObjFound ||
        (existingArtObjsIter != existingArticulatedObjects_.end()));
    if (existingObjFound) {
      return existingObjsIter->second->contactTest();
    } else {
      return existingArtObjsIter->second->contactTest();
    }
    return false;
  }

  /**
   * @brief Perform discrete collision detection for the scene with the derived
   * PhysicsManager implementation. Not implemented for default @ref
   * PhysicsManager. See @ref BulletPhysicsManager.
   */
  virtual void performDiscreteCollisionDetection() {
    /*Does nothing in base PhysicsManager.*/
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
  }

  /**
   * @brief Query the number of contact points that were active during the
   * most recent collision detection check.
   *
   * Not implemented for default PhysicsManager.
   * @return the number of active contact points.
   */
  virtual int getNumActiveContactPoints() { return -1; }

  /**
   * @brief See BulletPhysicsManager.h getNumActiveOverlappingPairs
   */
  virtual int getNumActiveOverlappingPairs() { return -1; }

  /**
   * @brief See BulletPhysicsManager.h getStepCollisionSummary
   */
  virtual std::string getStepCollisionSummary() { return "not implemented"; }

  /**
   * @brief Query physics simulation implementation for contact point data from
   * the most recent collision detection cache.
   *
   * Not implemented for default PhysicsManager implementation.
   * @return a vector with each entry corresponding to a single contact point.
   */
  virtual std::vector<ContactPointData> getContactPoints() const { return {}; }

  /**
   * @brief Set the stage to collidable or not.
   *
   * @param collidable Whether or not the object should be collision active
   */
  void setStageIsCollidable(bool collidable) {
    staticStageObject_->setCollidable(collidable);
  }

  /**
   * @brief Get whether or not the stage is collision active.
   *
   * @return Whether or not the stage is set to be collision active
   */
  bool getStageIsCollidable() { return staticStageObject_->getCollidable(); }

  /** @brief Return the library implementation type for the simulator
   * currently in use. Use to check for a particular implementation.
   * @return The implementation type of this simulator.
   */
  const PhysicsSimulationLibrary& getPhysicsSimulationLibrary() const {
    return activePhysSimLib_;
  }

  /**
   * @brief Get a copy of the template used to initialize the stage.
   *
   * @return The initialization settings of the stage or nullptr if the
   * stage is not initialized.
   */
  metadata::attributes::StageAttributes::ptr getStageInitAttributes() const {
    return staticStageObject_->getInitializationAttributes();
  }

  /**
   * @brief Get a copy of the template used to initialize this physics
   * manager
   *
   * @return The initialization settings for this physics manager
   */
  std::shared_ptr<metadata::attributes::PhysicsManagerAttributes>
  getInitializationAttributes() const;

  /**
   * @brief Cast a ray into the collision world and return a @ref
   * RaycastResults with hit information.
   *
   * Note: not implemented here in default PhysicsManager as there are no
   * collision objects without a simulation implementation.
   *
   * @param ray The ray to cast. Need not be unit length, but returned hit
   * distances will be in units of ray length.
   * @param maxDistance The maximum distance along the ray direction to
   * search. In units of ray length.
   * @return The raycast results sorted by distance.
   */
  virtual RaycastResults castRay(const esp::geo::Ray& ray,
                                 CORRADE_UNUSED double maxDistance = 100.0) {
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
    RaycastResults results;
    results.ray = ray;
    return results;
  }

  /**
   * @brief returns the wrapper manager for the currently created rigid
   * objects.
   * @return RigidObject wrapper manager.
   */
  std::shared_ptr<RigidObjectManager> getRigidObjectManager() const {
    return rigidObjectManager_;
  }

  /**
   * @brief returns the wrapper manager for the currently created articulated
   * objects.
   * @return ArticulatedObject wrapper manager
   */
  std::shared_ptr<ArticulatedObjectManager> getArticulatedObjectManager() {
    return articulatedObjectManager_;
  }

  /**
   * @brief Check if @p physObjectID represents an existing rigid object.
   * @param physObjectID Object ID to check
   * @return Whether rigid object exists with this id or not.
   */
  inline bool isValidRigidObjectId(const int physObjectID) const {
    return (existingObjects_.count(physObjectID) > 0);
  }

  /**
   * @brief Check if @p physObjectID represents an existing articulated object.
   * @param physObjectID Object ID to check
   * @return Whether articulated object exists with this id or not.
   */
  inline bool isValidArticulatedObjectId(const int physObjectID) const {
    return (existingArticulatedObjects_.count(physObjectID) > 0);
  }

  //============= Object Rigid Constraint API =============

  /**
   * @brief Create a rigid constraint between two objects or an object and the
   * world.
   *
   * Note: Method not implemented for base PhysicsManager.
   *
   * @param settings The datastructure defining the constraint parameters.
   *
   * @return The id of the newly created constraint or ID_UNDEFINED if failed.
   */
  virtual int createRigidConstraint(
      CORRADE_UNUSED const RigidConstraintSettings& settings) {
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
    return ID_UNDEFINED;
  }

  /**
   * @brief Update the settings of a rigid constraint.
   *
   * Note: Method not implemented for base PhysicsManager.
   *
   * @param constraintId The id of the constraint to update.
   * @param settings The new settings of the constraint.
   */
  virtual void updateRigidConstraint(
      CORRADE_UNUSED int constraintId,
      CORRADE_UNUSED const RigidConstraintSettings& settings) {
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
  }

  /**
   * @brief Remove a rigid constraint by id.
   *
   * Note: Method not implemented for base PhysicsManager.
   *
   * @param constraintId The id of the constraint to remove.
   */
  virtual void removeRigidConstraint(CORRADE_UNUSED int constraintId) {
    ESP_ERROR() << "Not implemented in base PhysicsManager. Install with "
                   "--bullet to use this feature.";
  }

  /**
   * @brief Get a copy of the settings for an existing rigid constraint.
   *
   * @param constraintId The id of the constraint.
   *
   * @return The settings of the constraint.
   */
  RigidConstraintSettings getRigidConstraintSettings(int constraintId) const {
    auto rigidCnstrntSettingsIter = rigidConstraintSettings_.find(constraintId);
    ESP_CHECK(rigidCnstrntSettingsIter != rigidConstraintSettings_.end(),
              "No RigidConstraint exists with constraintId =" << constraintId);
    return rigidCnstrntSettingsIter->second;
  }
  /**
   * @brief This will populate the passed @p sceneInstanceAttrs with the current
   * stage, object and articulated object instances reflecting the current
   * state of the physics world.
   * @param sceneInstanceAttrs A copy of the intialization attributes that
   * created the current scene.  The various object instance attributes will be
   * overwritten by the current scene state data.
   */
  void buildCurrentStateSceneAttributes(
      const metadata::attributes::SceneInstanceAttributes::ptr&
          sceneInstanceAttrs) const;

  /**
   * @brief Compute a trajectory visualization for the passed points.
   * @param trajVisName The name to use for the trajectory visualization
   * @param pts The points of a trajectory, in order
   * @param colorVec Array of colors for trajectory tube.
   * @param numSegments The number of the segments around the circumference of
   * the tube. Must be greater than or equal to 3.
   * @param radius The radius of the tube.
   * @param smooth Whether to smooth the points in the trajectory or not. Will
   * build a much bigger mesh
   * @param numInterp The number of interpolations between each trajectory
   * point, if smoothed
   * @return The ID of the object created for the visualization
   */
  int addTrajectoryObject(const std::string& trajVisName,
                          const std::vector<Mn::Vector3>& pts,
                          const std::vector<Mn::Color3>& colorVec,
                          int numSegments = 3,
                          float radius = .001,
                          bool smooth = false,
                          int numInterp = 10);
  /**
   * @brief Remove a trajectory visualization by name.
   * @param trajVisName The name of the trajectory visualization to remove.
   * @return whether successful or not.
   */
  bool removeTrajVisByName(const std::string& trajVisName) {
    auto trajVisIter = trajVisIDByName.find(trajVisName);
    if (trajVisIter == trajVisIDByName.end()) {
      ESP_DEBUG() << "No trajectory named" << trajVisName
                  << "exists.  Ignoring.";
      return false;
    }
    return removeTrajVisObjectAndAssets(trajVisIter->second, trajVisName);
  }

  /**
   * @brief Remove a trajectory visualization by object ID.
   * @param trajVisObjID The object ID of the trajectory visualization to
   * remove.
   * @return whether successful or not.
   */
  bool removeTrajVisByID(int trajVisObjID) {
    auto trajVisIter = trajVisNameByID.find(trajVisObjID);
    if (trajVisIter == trajVisNameByID.end()) {
      ESP_DEBUG() << "No trajectory object with ID:" << trajVisObjID
                  << "exists.  Ignoring.";
      return false;
    }
    return removeTrajVisObjectAndAssets(trajVisObjID, trajVisIter->second);
  }

 protected:
  /**
   * @brief Internal use only. Remove a trajectory object, its mesh, and all
   * references to it.
   * @param trajVisObjID The object ID of the trajectory visualization to
   * remove.
   * @param trajVisName The name of the trajectory visualization to remove.
   * @return whether successful or not.
   */
  bool removeTrajVisObjectAndAssets(int trajVisObjID,
                                    const std::string& trajVisName) {
    removeObject(trajVisObjID);
    // TODO : support removing asset by removing from resourceDict_ properly
    // using trajVisName
    trajVisIDByName.erase(trajVisName);
    trajVisNameByID.erase(trajVisObjID);
    return true;
  }

  /** @brief Retrieve an iterator to a given object ID's value if it is valid
   * (i.e. it refers to an existing rigid object). Terminate the program and
   * report an error if not. This function is intended to unify object ID
   * checking for @ref PhysicsManager functions.
   * @param physObjectID The object ID to validate.
   * @return iterator to map entry or to end of map if DNE
   */
  std::map<int, RigidObject::ptr>::iterator getRigidObjIteratorOrAssert(
      const int physObjectID) {
    auto objIter = existingObjects_.find(physObjectID);
    CORRADE_INTERNAL_ASSERT(objIter != existingObjects_.end());
    return objIter;
  }

  /** @brief Retrieve an iterator to a given articulated object ID's value if it
   * is valid (i.e. it refers to an existing articulated object). Terminate the
   * program and report an error if not. This function is intended to unify
   * object ID checking for @ref PhysicsManager functions.
   * @param physObjectID The articulated object ID to validate.
   * @return iterator to map entry or to end of map if DNE
   */
  std::map<int, ArticulatedObject::ptr>::iterator
  getArticulatedObjIteratorOrAssert(const int physObjectID) {
    auto aObjIter = existingArticulatedObjects_.find(physObjectID);
    CORRADE_INTERNAL_ASSERT(aObjIter != existingArticulatedObjects_.end());
    return aObjIter;
  }

  /** @brief Retrieve an iterator to a given object ID's value if it is valid
   * (i.e. it refers to an existing rigid object). Terminate the program and
   * report an error if not. This function is intended to unify object ID
   * checking for @ref PhysicsManager functions.
   * @param physObjectID The object ID to validate.
   * @return iterator to map entry or to end of map if DNE
   */
  std::map<int, RigidObject::ptr>::const_iterator
  getConstRigidObjIteratorOrAssert(const int physObjectID) const {
    auto objIter = existingObjects_.find(physObjectID);
    CORRADE_INTERNAL_ASSERT(objIter != existingObjects_.end());
    return objIter;
  }

  /** @brief Retrieve an iterator to a given articulated object ID's value if it
   * is valid (i.e. it refers to an existing articulated object). Terminate the
   * program and report an error if not. This function is intended to unify
   * object ID checking for @ref PhysicsManager functions.
   * @param physObjectID The articulated object ID to validate.
   * @return iterator to map entry or to end of map if DNE
   */
  std::map<int, ArticulatedObject::ptr>::const_iterator
  getConstArticulatedObjIteratorOrAssert(const int physObjectID) const {
    auto aObjIter = existingArticulatedObjects_.find(physObjectID);
    CORRADE_INTERNAL_ASSERT(aObjIter != existingArticulatedObjects_.end());
    return aObjIter;
  }

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

  /** @brief Create and initialize a @ref RigidObject, assign it an ID and
   * add it to existingObjects_ map keyed with newObjectID
   * @param newObjectID valid object ID for the new object
   * @param objectAttributes The physical object's template
   * @param objectNode Valid, existing scene node
   * @return whether the object has been successfully initialized and added
   * to existingObjects_ map
   */
  virtual bool makeAndAddRigidObject(
      int newObjectID,
      const esp::metadata::attributes::ObjectAttributes::ptr& objectAttributes,
      scene::SceneNode* objectNode);

  /** @brief A reference to a @ref esp::assets::ResourceManager which holds
   * assets that can be accessed by this @ref PhysicsManager*/
  assets::ResourceManager& resourceManager_;

  //! URDF importer implementation and model cache.
  std::unique_ptr<URDFImporter> urdfImporter_;

  /**@brief A pointer to this physics manager's owning simulator.
   */
  esp::sim::Simulator* simulator_ = nullptr;

  /** @brief A pointer to the @ref
   * esp::metadata::attributes::PhysicsManagerAttributes describing
   * this physics manager
   */
  const std::shared_ptr<const metadata::attributes::PhysicsManagerAttributes>
      physicsManagerAttributes_;

  /** @brief The current physics library implementation used by this
   * @ref PhysicsManager. Can be used to correctly cast the @ref PhysicsManager
   * to its derived type if necessary.
   */
  PhysicsSimulationLibrary activePhysSimLib_ =
      PhysicsSimulationLibrary::NoPhysics;  // default

  /**
   * @brief The @ref scene::SceneNode which is the parent of all members of the
   * scene graph which exist in the physical world. Used to keep track of all
   * SceneNode's that have physical properties.
   */
  scene::SceneNode* physicsNode_ = nullptr;

  /**
   * @brief The @ref scene::SceneNode which represents the static collision
   * geometry of the physical world. Only one @ref staticStageObject_ may
   * exist in a physical world. This @ref RigidStage can only have @ref
   * MotionType::STATIC as it is loaded as static geometry with simulation
   * efficiency in mind. See
   * @ref addStage.
   */
  physics::RigidStage::ptr staticStageObject_ = nullptr;

  //! ==== Rigid object memory management ====

  /** @brief This manager manages the wrapper objects used to provide safe,
   * direct user access to all existing physics objects.
   */
  std::shared_ptr<RigidObjectManager> rigidObjectManager_;

  /** @brief This manager manages the wrapper objects used to provide safe,
   * direct user access to all existing physics objects.
   */
  std::shared_ptr<ArticulatedObjectManager> articulatedObjectManager_;

  /** @brief Maps object IDs to all existing physical object instances in
   * the world.
   */
  std::map<int, RigidObject::ptr> existingObjects_;

  /** @brief Maps articulated object IDs to all existing physical object
   * instances in the world.
   */
  std::map<int, ArticulatedObject::ptr> existingArticulatedObjects_;

  /** @brief A counter of unique object ID's allocated thus far. Used to
   * allocate new IDs when  @ref recycledObjectIDs_ is empty without needing
   * to check @ref existingObjects_ explicitly.*/
  int nextObjectID_ = 0;

  /** @brief A list of available object IDs tracked by @ref
   * deallocateObjectID which were previously used by objects since removed
   * from the world with
   * @ref removeObject. These IDs will be re-allocated with @ref
   * allocateObjectID before new IDs are acquired with @ref nextObjectID_.
   */
  std::vector<int> recycledObjectIDs_;

  /** @brief Tmaps constraint ids to their settings */
  std::unordered_map<int, RigidConstraintSettings> rigidConstraintSettings_;

  //! Maps holding IDs and Names of trajectory visualizations
  std::unordered_map<std::string, int> trajVisIDByName;
  std::unordered_map<int, std::string> trajVisNameByID;

  //! Utilities

  /** @brief Tracks whether or not this @ref PhysicsManager has already been
   * initialized with @ref initPhysics. */
  bool initialized_ = false;

  /** @brief The fixed amount of time over which to integrate the simulation
   * in discrete steps within @ref stepPhysics. Lower values result in
   * better stability at the cost of worse efficiency and vice versa. */
  double fixedTimeStep_ = 1.0 / 240.0;

  /** @brief The current simulation time. Tracks the total amount of time
   * simulated with @ref stepPhysics up to this point.
   */
  double worldTime_ = 0.0;

 public:
  ESP_SMART_POINTERS(PhysicsManager)
};

}  // namespace physics

}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSMANAGER_H_
