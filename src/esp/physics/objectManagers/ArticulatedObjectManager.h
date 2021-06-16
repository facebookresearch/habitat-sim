// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H
#define ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H

#include "PhysicsObjectBaseManager.h"
#include "esp/physics/bullet/objectWrappers/ManagedBulletArticulatedObject.h"
#include "esp/physics/objectWrappers/ManagedArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality shared for
 * managing all @ref esp::physics::ManagedArticulatedObject wrappers.
 */

class ArticulatedObjectManager
    : public esp::physics::PhysicsObjectBaseManager<ManagedArticulatedObject> {
 public:
  explicit ArticulatedObjectManager();

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * BulletArticulatedObject in the world.  This version does not require
   * drawables to be specified.
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
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return A reference to the created ArticulatedObject
   */
  std::shared_ptr<ManagedArticulatedObject> addArticulatedObjectFromURDF(
      const std::string& filepath,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

  /**
   * @brief Cast to BulletArticulatedObject version.  Load, parse, and import a
   * URDF file instantiating an @ref BulletArticulatedObject in the world.  This
   * version does not require drawables to be specified.
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
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return A reference to the created ArticulatedObject
   */
  std::shared_ptr<ManagedBulletArticulatedObject>
  addBulletArticulatedObjectFromURDF(
      const std::string& filepath,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY) {
    return std::static_pointer_cast<ManagedBulletArticulatedObject>(
        addArticulatedObjectFromURDF(filepath, fixedBase, globalScale,
                                     massScale, forceReload, lightSetup));
  }

  /**
   * @brief Load, parse, and import a URDF file instantiating an @ref
   * BulletArticulatedObject in the world.
   * @param filepath The fully-qualified filename for the URDF file
   * describing the model the articulated object is to be built from.
   * @param drawables Reference to the scene graph drawables group to enable
   * rendering of the newly initialized @ref ArticulatedObject.
   * @param fixedBase Whether the base of the @ref ArticulatedObject should
   * be fixed.
   * @param globalScale A scale multiplier to be applied uniformly in 3
   * dimensions to the entire @ref ArticulatedObject.
   * @param massScale A scale multiplier to be applied to the mass of the
   * all the components of the @ref ArticulatedObject.
   * @param forceReload If true, reload the source URDF from file, replacing
   * the cached model.
   * @param lightSetup The string name of the desired lighting setup to use.
   *
   * @return A reference to the created ArticulatedObject
   */
  std::shared_ptr<ManagedArticulatedObject>
  addArticulatedObjectFromURDFWithDrawables(
      const std::string& filepath,
      gfx::DrawableGroup* drawables,
      bool fixedBase = false,
      float globalScale = 1.0,
      float massScale = 1.0,
      bool forceReload = false,
      const std::string& lightSetup = DEFAULT_LIGHTING_KEY);

 protected:
  /**
   * @brief This method will remove articulated objects from physics manager.
   * The wrapper has already been removed by the time this method is called
   * (this is called from @ref
   * esp::core::ManagedContainerBase::deleteObjectInternal)
   *
   * @param objectID the ID of the managed object to remove
   * @param objectHandle the string key of the managed object to remove.
   */
  void deleteObjectInternalFinalize(
      int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {
    if (auto physMgr = this->getPhysicsManager()) {
      if (physMgr->isValidArticulatedObjectId(objectID)) {
        physMgr->removeArticulatedObject(objectID);
      }
    }
  }  // deleteObjectInternalFinalize

 public:
  ESP_SMART_POINTERS(ArticulatedObjectManager)
};  // class ArticulatedObjectManager

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H
