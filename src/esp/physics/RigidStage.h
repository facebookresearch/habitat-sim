// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDSTAGE_H_
#define ESP_PHYSICS_RIGIDSTAGE_H_

#include "esp/physics/RigidBase.h"

/** @file
 * @brief Class @ref esp::physics::RigidScene
 */
namespace esp {
namespace physics {
class RigidStage : public RigidBase {
 public:
  RigidStage(scene::SceneNode* rigidBodyNode);

  /**
   * @brief Virtual destructor for a @ref RigidScene.
   */
  virtual ~RigidStage() {}

  /**
   * @brief Initializes the @ref RigidScene that inherits
   * from this class
   * @param resMgr a reference to ResourceManager object
   * @param handle The handle for the template structure defining relevant
   * phyiscal parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialize(const assets::ResourceManager& resMgr,
                  const std::string& handle) override;

  /**
   * @brief Get a copy of the template used to initialize this scene object.
   *
   * @return A copy of the @ref PhysicsSceneAttributes template used to create
   * this scene object.
   */
  std::shared_ptr<Attrs::StageAttributes> getInitializationAttributes() const {
    return RigidBase::getInitializationAttributes<Attrs::StageAttributes>();
  };
  /**
   * @brief Finalize the creation of this @ref RigidScene
   * @return whether successful finalization.
   */
  bool finalizeObject() override { return finalizeObject_LibSpecific(); }

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidScene
   * geometry.  This is overridden by inheriting class specific to certain
   * physics libraries.Necessary to support kinematic objects without any
   * dynamics support.
   * @param resMgr Reference to resource manager, to access relevant components
   * pertaining to the scene object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific(
      CORRADE_UNUSED const assets::ResourceManager& resMgr) override {
    return true;
  }
  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after@ref RigidScene is created.  Called from finalizeObject.  Overridden
   * by inheriting class specific to certain physics libraries. Necessary to
   * support kinematic objects without any dynamics support.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override { return true; }

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
  bool setMotionType(MotionType mt) override {
    return mt == MotionType::STATIC;  // only option and default option
  }

 public:
  ESP_SMART_POINTERS(RigidStage)
};
}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDSTAGE_H_
