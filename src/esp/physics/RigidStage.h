// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_RIGIDSTAGE_H_
#define ESP_PHYSICS_RIGIDSTAGE_H_

#include "esp/metadata/attributes/StageAttributes.h"
#include "esp/physics/RigidBase.h"

/** @file
 * @brief Class @ref esp::physics::RigidStage
 */
namespace esp {
namespace physics {

/**
 * @brief A @ref RigidBase representing an individual rigid stage instance
 * attached to a SceneNode. This construction currently may only be
 * @ref esp::physics::MotionType::STATIC.
 */
class RigidStage : public RigidBase {
 public:
  RigidStage(scene::SceneNode* rigidBodyNode,
             const assets::ResourceManager& resMgr);

  /**
   * @brief Virtual destructor for a @ref RigidStage.
   */
  ~RigidStage() override = default;

  /**
   * @brief Initializes the @ref RigidStage that inherits
   * from this class
   * @param initAttributes The template structure defining relevant
   * physical parameters for this object
   * @return true if initialized successfully, false otherwise.
   */
  bool initialize(metadata::attributes::AbstractObjectAttributes::ptr
                      initAttributes) override;

  /**
   * @brief Get a copy of the template used to initialize this stage object.
   *
   * @return A copy of the @ref esp::metadata::attributes::StageAttributes
   * template used to create this stage object.
   */
  std::shared_ptr<metadata::attributes::StageAttributes>
  getInitializationAttributes() const {
    return RigidBase::getInitializationAttributes<
        metadata::attributes::StageAttributes>();
  };
  /**
   * @brief Finalize the creation of this @ref RigidStage
   * @return whether successful finalization.
   */
  bool finalizeObject() override { return finalizeObject_LibSpecific(); }

 private:
  /**
   * @brief Finalize the initialization of this @ref RigidStage
   * geometry.  This is overridden by inheriting class specific to certain
   * physics libraries.Necessary to support kinematic objects without any
   * dynamics support.
   * @return true if initialized successfully, false otherwise.
   */
  bool initialization_LibSpecific() override { return true; }
  /**
   * @brief any physics-lib-specific finalization code that needs to be run
   * after@ref RigidStage is created.  Called from finalizeObject.  Overridden
   * by inheriting class specific to certain physics libraries. Necessary to
   * support kinematic objects without any dynamics support.
   * @return whether successful finalization.
   */
  bool finalizeObject_LibSpecific() override { return true; }

 public:
  /**
   * @brief Currently not supported. Set or reset the stages's state using the
   * object's specified @p sceneInstanceAttributes_.
   */
  void resetStateFromSceneInstanceAttr() override {}

  /**
   * @brief Currently ignored for stage objects.
   * @param mt The desirved @ref MotionType.
   */
  void setMotionType(CORRADE_UNUSED MotionType mt) override {
    ESP_WARNING() << "Stages cannot have their "
                     "motion type changed from MotionType::STATIC, so "
                     "aborting; motion type is unchanged.";
  }

 public:
  ESP_SMART_POINTERS(RigidStage)
};
}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_RIGIDSTAGE_H_
