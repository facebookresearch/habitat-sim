// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_PHYSICSOBJECTBASE_H_
#define ESP_PHYSICS_PHYSICSOBJECTBASE_H_

#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include "esp/assets/ResourceManager.h"

/** @file
 * @brief Class @ref esp::physics::PhysicsObjectBase is the base class for any
 * physics-based construct, and holds basic accounting info and accessors, along
 * with scene node access.
 */

namespace esp {

namespace physics {

/**
@brief Motion type of a @ref RigidObject.
Defines its treatment by the simulator and operations which can be performed on
it.
*/
enum class MotionType {
  /**
   * Refers to an error (such as a query to non-existing object) or an
   * unknown/unspecified value.
   */
  UNDEFINED = -1,

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

class PhysicsObjectBase : public Magnum::SceneGraph::AbstractFeature3D {
 public:
  PhysicsObjectBase(scene::SceneNode* bodyNode,
                    int objectId,
                    const assets::ResourceManager& resMgr)
      : Magnum::SceneGraph::AbstractFeature3D(*bodyNode),
        objectId_(objectId),
        resMgr_(resMgr) {}

  ~PhysicsObjectBase() override = default;

  /**
   * @brief Get the scene node being attached to.
   */
  scene::SceneNode& node() { return object(); }
  const scene::SceneNode& node() const { return object(); }

  // Overloads to avoid confusion
  scene::SceneNode& object() {
    return static_cast<scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }
  const scene::SceneNode& object() const {
    return static_cast<const scene::SceneNode&>(
        Magnum::SceneGraph::AbstractFeature3D::object());
  }

  /**
   * @brief Set the @ref MotionType of the object. If the object is @ref
   * ObjectType::SCENE it can only be @ref MotionType::STATIC. If the object is
   * @ref ObjectType::OBJECT is can also be set to @ref MotionType::KINEMATIC.
   * Only if a dervied @ref PhysicsManager implementing dynamics is in use can
   * the object be set to @ref MotionType::DYNAMIC.
   * @param mt The desirved @ref MotionType.
   * @return true if successfully set, false otherwise.
   */
  virtual bool setMotionType(MotionType mt) = 0;
  /**
   * @brief Get the @ref MotionType of the object. See @ref setMotionType.
   * @return The object's current @ref MotionType.
   */
  MotionType getMotionType() const { return objectMotionType_; };

  /**
   * @brief Get object's ID
   */
  int getObjectID() const { return objectId_; }

 protected:
  /** @brief The @ref MotionType of the object. Determines what operations can
   * be performed on this object. */
  MotionType objectMotionType_{MotionType::UNDEFINED};

  /** @brief Access for the object to its own PhysicsManager id.
   */
  int objectId_;

  /** @brief Reference to the ResourceManager for internal access to the
   * object's asset data.
   */
  const assets::ResourceManager& resMgr_;

 public:
  ESP_SMART_POINTERS(PhysicsObjectBase)
};  // class PhysicsObjectBase

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_PHYSICSOBJECTBASE_H_
