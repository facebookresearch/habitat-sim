// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Class @ref esp::physics::RigidObject, enum @ref
 * esp::physics::MotionType, enum @ref esp::physics::RigidObjectType
 */

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/DebugTools/ForceRenderer.h>
#include "esp/assets/Asset.h"
#include "esp/assets/Attributes.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/FRLInstanceMeshData.h"
#include "esp/assets/GenericInstanceMeshData.h"
#include "esp/assets/MeshData.h"
#include "esp/core/esp.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace physics {

/**
@brief Motion type of a @ref RigidObject.
Defines its treatment by the simulator and operations which can be performed on
it.
*/
enum MotionType {
  /**
   * Refers to an error (such as a query to non-existing object).
   */
  ERROR_MOTIONTYPE,

  /**
   * The object is not expected to move and should not allow kinematic updates.
   * Likely treated as static collision geometry. See @ref
   * RigidObjectType::SCENE.
   */
  STATIC,

  /**
   * The object is expected to move kinematically, but is not simulated. Default
   * behavior of @ref RigidObject with no physics simulator defined.
   */
  KINEMATIC,

  /**
   * The object is simulated and can, but should not be, updated kinematically .
   * Default behavior of @ref RigidObject with a physics simulator defined. See
   * @ref BulletRigidObject.
   */
  DYNAMIC

};

/**
@brief Category of a @ref RigidObject. Defines treatment of the object in @ref
PhysicsManager. Also denotes the status of an object as initialized or not.
*/
enum RigidObjectType {
  /**
   * The object is not initialized yet. Set as default on construction.
   */
  NONE,

  /**
   * The object is a @ref MotionType::STATIC scene collision geometry.
   * See @PhysicsManager::addScene.
   */
  SCENE,

  /**
   * The object is a standard rigid object and should be present in @ref
   * @PhysicsManager::existingObjects_. See @PhysicsManager::addObject.
   */
  OBJECT

};

/**
@brief A @ref scene::SceneNode representing an individual rigid object instance.
This may be a @ref MotionType::STATIC scene collision geometry or an object of
any @ref MotionType which can interact with other members of a physical world.
Must have a collision mesh.
By default, a RigidObject is @ref MotionType::KINEMATIC without an underlying
simulator implementation, but derived classes can be used to introduce specific
implementations of dynamics.
*/
class RigidObject : public scene::SceneNode {
 public:
  RigidObject(scene::SceneNode* parent);

  // TODO: Currently a RigidObject is either a scene
  // or an object, but cannot be both (tracked by _isScene/_isObject_)
  // there is probably a better way to abstract this
  virtual bool initializeScene(
      const assets::PhysicsSceneAttributes& physicsSceneAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  virtual bool initializeObject(
      const assets::PhysicsObjectAttributes& physicsObjectAttributes,
      const std::vector<assets::CollisionMeshData>& meshGroup);

  ~RigidObject(){};

  //! Check whether object is being actively simulated, or sleeping
  virtual bool isActive();
  virtual void setActive(){};

  // attempt to set the motion type. Return false=failure, true=success.
  virtual bool setMotionType(MotionType mt);
  MotionType getMotionType() { return objectMotionType_; };

  //! Force interaction
  virtual void applyForce(const Magnum::Vector3& force,
                          const Magnum::Vector3& relPos);
  // Impulse Force interaction
  virtual void applyImpulse(const Magnum::Vector3& impulse,
                            const Magnum::Vector3& relPos);

  //! Torque interaction
  virtual void applyTorque(const Magnum::Vector3& torque);
  // Impulse Torque interaction
  virtual void applyImpulseTorque(const Magnum::Vector3& impulse);

  //! (Prototype) For visualizing & debugging
  void debugForce(Magnum::SceneGraph::DrawableGroup3D& debugDrawables);
  //! (Prototype) For visualizing & debugging
  void setDebugForce(Magnum::Vector3& force);

  virtual bool removeObject();

  // ==== Transformations ===
  //! Need to overwrite a bunch of functions to update physical states
  virtual SceneNode& setTransformation(const Magnum::Matrix4& transformation);
  virtual SceneNode& setTranslation(const Magnum::Vector3& vector);
  virtual SceneNode& setRotation(const Magnum::Quaternion& quaternion);

  virtual SceneNode& resetTransformation();
  virtual SceneNode& translate(const Magnum::Vector3& vector);
  virtual SceneNode& translateLocal(const Magnum::Vector3& vector);

  virtual SceneNode& rotate(const Magnum::Rad angleInRad,
                            const Magnum::Vector3& normalizedAxis);
  virtual SceneNode& rotateLocal(const Magnum::Rad angleInRad,
                                 const Magnum::Vector3& normalizedAxis);

  virtual SceneNode& rotateX(const Magnum::Rad angleInRad);
  virtual SceneNode& rotateY(const Magnum::Rad angleInRad);
  virtual SceneNode& rotateZ(const Magnum::Rad angleInRad);
  virtual SceneNode& rotateXLocal(const Magnum::Rad angleInRad);
  virtual SceneNode& rotateYLocal(const Magnum::Rad angleInRad);
  virtual SceneNode& rotateZLocal(const Magnum::Rad angleInRad);

  // ==== Getter/Setter functions ===
  //! For kinematic objects they are dummies, for dynamic objects
  //! implemented in physics-engine specific ways
  virtual double getMass() { return 0.0; }
  virtual double getScale() { return 0.0; }
  virtual double getFrictionCoefficient() { return 0.0; }
  virtual double getRestitutionCoefficient() { return 0.0; }
  virtual double getLinearDamping() { return 0.0; }
  virtual double getAngularDamping() { return 0.0; }
  virtual Magnum::Vector3 getCOM();
  // Get local inertia
  virtual Magnum::Vector3 getInertiaVector();
  virtual Magnum::Matrix3 getInertiaMatrix();

  virtual void setMass(const double){};
  virtual void setCOM(const Magnum::Vector3&){};
  virtual void setInertia(const Magnum::Vector3&){};
  virtual void setScale(const double){};
  virtual void setFrictionCoefficient(const double){};
  virtual void setRestitutionCoefficient(const double){};
  virtual void setLinearDamping(const double){};
  virtual void setAngularDamping(const double){};

  // public Attributes object for user convenience.
  assets::Attributes attributes_;

 protected:
  MotionType objectMotionType_;
  RigidObjectType rigidObjectType_ = NONE;

  // used only if isObject_
  // assets::PhysicsObjectMetaData physicsObjectMetaData_;

  //! Needed after changing the pose from Magnum side
  //! Not exposed to end user
  virtual void syncPose();

  ESP_SMART_POINTERS(RigidObject)
};

}  // namespace physics
}  // namespace esp
