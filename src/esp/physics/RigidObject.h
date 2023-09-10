// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDOBJECT_H_
#define ESP_PHYSICS_RIGIDOBJECT_H_

/** @file
 * @brief Class @ref esp::physics::RigidObject, enum @ref
 * esp::physics::MotionType, enum @ref esp::physics::RigidObjectType, struct
 * @ref VelocityControl
 */

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include "esp/assets/Asset.h"
#include "esp/assets/BaseMesh.h"
#include "esp/assets/GenericSemanticMeshData.h"
#include "esp/core/Esp.h"
#include "esp/core/RigidState.h"
#include "esp/metadata/attributes/ObjectAttributes.h"
#include "esp/scene/SceneNode.h"

#include "esp/physics/RigidBase.h"

namespace esp {

namespace assets {

class ResourceManager;
}  // namespace assets
namespace physics {

/**@brief Convenience struct for applying constant velocity control to a rigid
 * body. */
struct VelocityControl {
 public:
  virtual ~VelocityControl() = default;

  /**@brief Constant linear velocity. */
  Magnum::Vector3 linVel;
  /**@brief Constant angular velocity. */
  Magnum::Vector3 angVel;
  /**@brief Whether or not to set linear control velocity before stepping. */
  bool controllingLinVel = false;
  /**
   * @brief Whether or not to set linear control velocity in local space.
   * Useful for commanding actions such as "forward", or "strafe".
   */
  bool linVelIsLocal = false;

  /**@brief Whether or not to set angular control velocity before stepping. */
  bool controllingAngVel = false;

  /**
   * @brief Whether or not to set angular control velocity in local space.
   * Useful for commanding actions such as "roll" and "yaw".
   */
  bool angVelIsLocal = false;

  /**
   * @brief Compute the result of applying constant control velocities to the
   * provided object transform.
   *
   * For efficiency this function does not support transforms with scaling.
   *
   * Default implementation uses explicit Euler integration.
   * @param dt The discrete timestep over which to integrate.
   * @param rigidState The initial state of the object before
   * applying velocity control.
   * @return The new state of the object after applying velocity control over
   * dt.
   */
  virtual core::RigidState integrateTransform(
      float dt,
      const core::RigidState& rigidState);

  ESP_SMART_POINTERS(VelocityControl)
};

/**
 * @brief A @ref RigidBase representing an individual rigid object instance
 * attached to a SceneNode, updating its state through simulation. This may be a
 * @ref esp::physics::MotionType::STATIC scene collision geometry or an object
 * of any @ref MotionType which can interact with other members of a physical
 * world. Must have a collision mesh. By default, a RigidObject is @ref
 * MotionType::KINEMATIC without an underlying simulator implementation. Derived
 * classes can be used to introduce specific implementations of dynamics.
 */
class RigidObject : public RigidBase {
 public:
  /**
   * @brief Constructor for a @ref esp::physics::RigidObject.
   * @param rigidBodyNode The @ref scene::SceneNode this feature will be
   * attached to.
   */
  RigidObject(scene::SceneNode* rigidBodyNode,
              int objectId,
              const assets::ResourceManager& resMgr);

  /**
   * @brief Virtual destructor for a @ref esp::physics::RigidObject.
   */
  ~RigidObject() override = default;

  /**
   * @brief Initializes the @ref esp::physics::RigidObject that inherits from
   * this class
   * @param initAttributes The template structure defining relevant
   * physical parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialize(metadata::attributes::AbstractObjectAttributes::ptr
                      initAttributes) override;

  /**
   * @brief Finalize the creation of the RigidObject.
   * @return whether successful finalization.
   */
  bool finalizeObject() override;

  /**
   * @brief Get a copy of the template used to initialize this object.
   *
   * @return A copy of the @ref esp::metadata::attributes::ObjectAttributes
   * template used to create this object.
   */
  std::shared_ptr<metadata::attributes::ObjectAttributes>
  getInitializationAttributes() const {
    return RigidBase::getInitializationAttributes<
        metadata::attributes::ObjectAttributes>();
  };

 private:
  /**
   * @brief Finalize the initialization of this @ref esp::physics::RigidObject's
   * geometry. This is overridden by inheriting class specific to certain
   * physics libraries. Necessary to support kinematic objects without any
   * dynamics support.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override;

  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after @ref RigidObject is created. Overridden by inheriting class specific
   * to certain physics libraries. Necessary to support kinematic objects
   * without any dynamics support.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override { return true; }

 public:
  /**
   * @brief Set whether this object is COM corrected, which determines how the
   * intial transformation is applied when placing the object.
   */

  void setIsCOMCorrected(bool _isCOMCorrected) {
    isCOMCorrected_ = _isCOMCorrected;
  }

  /**
   * @brief Reverses the COM correction transformation for objects that require
   * it. Currently a simple passthrough for stages and Articulated Objects.
   */
  Magnum::Vector3 getUncorrectedTranslation() const override {
    auto translation = getTranslation();
    auto rotation = getRotation();
    if (isCOMCorrected_) {
      translation += rotation.transformVector(visualNode_->translation());
    }
    return translation;
  }

  /**
   * @brief Set the @ref MotionType of the object. If the object is @ref
   * ObjectType::SCENE it can only be @ref esp::physics::MotionType::STATIC. If
   * the object is
   * @ref ObjectType::OBJECT is can also be set to @ref
   * esp::physics::MotionType::KINEMATIC. Only if a dervied @ref PhysicsManager
   * implementing dynamics is in use can the object be set to @ref
   * esp::physics::MotionType::DYNAMIC.
   * @param mt The desirved @ref MotionType.
   * @return true if successfully set, false otherwise.
   */
  void setMotionType(MotionType mt) override;

  /**
   * @brief Retrieves a reference to the VelocityControl struct for this object.
   */
  VelocityControl::ptr getVelocityControl() { return velControl_; };

  /**
   * @brief Set the object's state from a @ref
   * esp::metadata::attributes::SceneObjectInstanceAttributes
   */
  void resetStateFromSceneInstanceAttr() override;

 protected:
  /**
   * @brief Whether or not this object's placement should be COM corrected.
   */
  bool isCOMCorrected_ = false;

  /**
   * @brief Convenience variable: specifies a constant control velocity (linear
   * | angular) applied to the rigid body before each step.
   */
  VelocityControl::ptr velControl_;

 public:
  ESP_SMART_POINTERS(RigidObject)
};  // class RigidObject

}  // namespace physics
}  // namespace esp
#endif
