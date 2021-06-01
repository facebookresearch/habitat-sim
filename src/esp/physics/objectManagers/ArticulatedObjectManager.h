// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H
#define ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H

#include "PhysicsObjectBaseManager.h"
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

 protected:
  /**
   * @brief Used Internally.  Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param objectHandle Unused for wrapper objects.  All wrappers use the name
   * of their underlying objects.
   * @param builtFromConfig Unused for wrapper objects.  All wrappers are
   * constructed from scratch.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  std::shared_ptr<ManagedArticulatedObject> initNewObjectInternal(
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool builtFromConfig) override {
    return ManagedArticulatedObject::create();
  }  // ArticulatedObjectManager::initNewObjectInternal(
 public:
  ESP_SMART_POINTERS(ArticulatedObjectManager)
};  // class ArticulatedObjectManager

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDOBJECTMANAGER_H
