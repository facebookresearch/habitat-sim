// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::PhysicsManager, enum @ref
 * esp::physics::PhysicsManager::PhysicsSimulationLibrary
 */

#include <map>
#include <memory>
#include <string>
#include <vector>

/* Bullet Physics Integration */
#include <Magnum/Trade/MeshData3D.h>

#include "RigidObject.h"
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
  explicit PhysicsManager(assets::ResourceManager* _resourceManager) {
    resourceManager_ = _resourceManager;
  };

  /** @brief Destructor*/
  virtual ~PhysicsManager();

  /**
   * @brief Initialization: load physical properties and setup the world.
   *
   *
   * @param node    The scene graph node which will act as the parent of all
   * physical scene and object nodes.
   * @param physicsManagerAttributes A structure containing values for physical
   * parameters necessary to initialize the physical scene and simulator.
   */
  virtual bool initPhysics(
      scene::SceneNode* node,
      const assets::PhysicsManagerAttributes& physicsManagerAttributes);

  /**
   * @brief Reset the simulation and physical world.
   * Sets the @ref worldTime_ to 0.0, does not change physical state.
   */
  virtual void reset() {
    /* TODO: reset object states or clear them? Other? */
    worldTime_ = 0.0;
  };

  /** @brief Stores references to a set of drawable elements. */
  using DrawableGroup = gfx::DrawableGroup;

  /**
   * @brief Initialize static scene collision geometry from loaded mesh data.
   * Only one 'scene' may be initialized per simulated world, but this scene may
   * contain several components (e.g. GLB heirarchy).
   *
   * @param physicsSceneAttributes a structure defining physical properties of
   * the scene.
   * @param meshGroup collision meshs for the scene.
   * @return true if successful and false otherwise
   */
  virtual bool addScene(
      const assets::PhysicsSceneAttributes& physicsSceneAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::assets::ResourceManager::physicsObjectLibrary_.
   *  @anchor addObject_string
   *  @param configFile The filename of the object's physical properties file
   * used as the key to query @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_
   *  @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   *  @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  int addObject(const std::string& configFile,
                DrawableGroup* drawables,
                const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{
                    assets::ResourceManager::DEFAULT_LIGHTING_KEY});

  /** @brief Instance a physical object from an object properties template in
   * the @ref esp::assets::ResourceManager::physicsObjectLibrary_ by object
   * library index. Queries the properties filename and calls @ref
   * addObject(const std::string& configFile, DrawableGroup* drawables).
   *  @param objectLibIndex The index of the object's template in @ref
   * esp::assets::ResourceManager::physicsObjectLibrary_
   *  @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized object.
   *  @return the instanced object's ID, mapping to it in @ref
   * PhysicsManager::existingObjects_ if successful, or @ref esp::ID_UNDEFINED.
   */
  virtual int addObject(
      const int objectLibIndex,
      DrawableGroup* drawables,
      const Magnum::ResourceKey& lightSetup = Magnum::ResourceKey{
          assets::ResourceManager::DEFAULT_LIGHTING_KEY});

  /** @brief Remove an object instance from the pysical scene by ID, destroying
   * its scene graph node and removing it from @ref
   * PhysicsManager::existingObjects_.
   *  @param physObjectID The ID (key) of the object instance in @ref
   * PhysicsManager::existingObjects_.
   * @param deleteSceneNode If true, deletes the object's scene node. Otherwise
   * detaches the object from simulation.
   */
  virtual void removeObject(const int physObjectID,
                            bool deleteSceneNode = true);

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

  /** @brief Set the @ref MotionType of an object, allowing or disallowing its
   * manipulation by dynamic processes or kinematic control.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param  mt The desired @ref MotionType of the object to set.
   * @return True if set was successful and False otherwise.
   */
  bool setObjectMotionType(const int physObjectID, MotionType mt);

  /** @brief Get the @ref MotionType of an object.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The object's @ref MotionType
   */
  MotionType getObjectMotionType(const int physObjectID) const;

  //============ Simulator functions =============

  /** @brief Step the physical world forward in time. Time may only advance in
   * increments of @ref fixedTimeStep_.
   * @param dt The desired amount of time to advance the physical world.
   */
  virtual void stepPhysics(double dt = 0.0);

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

  // =========== Scene Getter/Setter functions ===========

  /** @brief Get the current friction coefficient of the scene collision
   * geometry. See @ref staticSceneObject_.
   * @return The scalar friction coefficient of the scene geometry.
   */
  virtual double getSceneFrictionCoefficient() const { return 0.0; };

  /** @brief Set the friction coefficient of the scene collision geometry. See
   * @ref staticSceneObject_.
   * @param frictionCoefficient The scalar friction coefficient of the scene
   * geometry.
   */
  virtual void setSceneFrictionCoefficient(
      CORRADE_UNUSED const double frictionCoefficient){};

  /** @brief Get the current coefficient of restitution for the scene collision
   * geometry. This determines the ratio of initial to final relative velocity
   * between the scene and collidiing object. See @ref staticSceneObject_. By
   * default this will always return 0, since kinametic scenes have no dynamics.
   * @return The scalar coefficient of restitution for the scene geometry.
   */
  virtual double getSceneRestitutionCoefficient() const { return 0.0; };

  /** @brief Set the coefficient of restitution for the scene collision
   * geometry. See @ref staticSceneObject_. By default does nothing since
   * kinametic scenes have no dynamics.
   * @param restitutionCoefficient The scalar coefficient of restitution to set.
   */
  virtual void setSceneRestitutionCoefficient(
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

  /** @brief Set the uniform scale for an object.
   * See @ref RigidObject::setScale.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param scale The new scalar uniform scale for the object relative to its
   * initially loaded meshes.
   */
  void setScale(const int physObjectID, const double scale);

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

  /** @brief Get the scalar uniform scale of an object.
   * See @ref RigidObject::getScale.
   * @param  physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return The uniform scale of the object relative to its initialy loaded
   * meshes.
   */
  double getScale(const int physObjectID) const;

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
  bool isActive(const int physObjectID) const;

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

  /** @brief Set bounding box rendering for the object true or false.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @param drawables The drawables group with which to render the bounding box.
   * @param drawBB Set rendering of the bounding box to true or false.
   */
  void setObjectBBDraw(int physObjectID, DrawableGroup* drawables, bool drawBB);

  /**
   * @brief Get a const reference to the specified object's SceneNode for info
   * query purposes.
   * @param physObjectID The object ID and key identifying the object in @ref
   * PhysicsManager::existingObjects_.
   * @return Const reference to the object scene node.
   */
  const scene::SceneNode& getObjectSceneNode(int physObjectID) const;

  /** @overload */
  scene::SceneNode& getObjectSceneNode(int physObjectID);

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
   * @return Whether or not the object is in contact with any other collision
   * enabled objects.
   */
  virtual bool contactTest(CORRADE_UNUSED const int physObjectID) {
    return false;
  };

  /** @brief Return the library implementation type for the simulator currently
   * in use. Use to check for a particular implementation.
   * @return The implementation type of this simulator.
   */
  const PhysicsSimulationLibrary& getPhysicsSimulationLibrary() const {
    return activePhysSimLib_;
  };

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

  /** @brief Create and initialize an @ref RigidObject and assign it an ID.
   * @param meshGroup The object's mesh.
   * @param physicsObjectAttributes The physical object's template defining its
   * physical parameters.
   */
  virtual int makeRigidObject(
      const std::vector<assets::CollisionMeshData>& meshGroup,
      assets::PhysicsObjectAttributes physicsObjectAttributes);

  /** @brief A pointer to a @ref esp::assets::ResourceManager which holds assets
   * that can be accessed by this @ref PhysicsManager*/
  assets::ResourceManager* resourceManager_;

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
   * geometry of the physical world. Only one @ref staticSceneObject_ may exist
   * in a physical world. This @ref RigidObject can only have @ref
   * MotionType::STATIC as it is loaded as static geometry with simulation
   * efficiency in mind. See
   * @ref addScene.
   * */
  physics::RigidObject::uptr staticSceneObject_ = nullptr;

  //! ==== Rigid object memory management ====

  /** @brief Maps object IDs to all existing physical object instances in the
   * world.
   */
  std::map<int, physics::RigidObject::uptr> existingObjects_;

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

  /** @brief Determines the maximum number of @ref fixedTimeStep_ allowed per
   * each call of @ref stepPhysics only for @ref BulletPhysicsManager. Has no
   * effect on @ref PhysicsManager base class. Candidate for removal... */
  int maxSubSteps_ = 10;

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
