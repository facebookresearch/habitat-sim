// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDSCENE_H_
#define ESP_PHYSICS_RIGIDSCENE_H_

#include "esp/physics/RigidBase.h"

/** @file
 * @brief Class @ref esp::physics::RigidScene
 */
namespace esp {
namespace physics {
class RigidScene : public RigidBase {
 public:
  RigidScene(scene::SceneNode* rigidBodyNode);

  /**
   * @brief Virtual destructor for a @ref RigidScene.
   */
  virtual ~RigidScene(){};

  /**
   * @brief Initializes the @ref RigidObject or @ref RigidScene that inherits
   * from this class
   * @param physicsAttributes The template structure defining relevant
   * phyiscal parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  virtual bool initialize(
      const assets::ResourceManager& resMgr,
      const assets::AbstractPhysicsAttributes::ptr physicsAttributes) override;

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidScene
   * geometry.  This is overridden by inheriting objects
   * @param resMgr Reference to resource manager, to access relevant components
   * pertaining to the scene object
   * @return true if initialized successfully, false otherwise.
   */
  virtual bool initializationFinalize(
      CORRADE_UNUSED const assets::ResourceManager& resMgr) override {
    return true;
  }

 public:
  /**
   * @brief Set the @ref MotionType of the object. If the object is @ref
   * ObjectType::SCENE it can only be @ref MotionType::STATIC. If the object is
   * @ref ObjectType::OBJECT is can also be set to @ref MotionType::KINEMATIC.
   * Only if a dervied @ref PhysicsManager implementing dynamics is in use can
   * the object be set to @ref MotionType::DYNAMIC.
   * @param mt The desirved @ref MotionType.
   * @return true if successfully set, false otherwise.
   */
  virtual bool setMotionType(MotionType mt) override {
    return mt == MotionType::STATIC;  // only option and default option
  }

 public:
  ESP_SMART_POINTERS(RigidScene)
};
}  // namespace physics
}  // namespace esp

#endif