// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_ARTICULATEDLINKMANAGER_H
#define ESP_PHYSICS_ARTICULATEDLINKMANAGER_H

#include "RigidBaseManager.h"
#include "esp/physics/objectWrappers/ManagedArticulatedLink.h"

namespace esp {
namespace physics {

/**
 * @brief Class template defining responsibilities and functionality shared for
 * managing all @ref esp::physics::ManagedArticulatedLink wrappers.  This
 * manager will be owned by the Articulated Object that the underlying links all
 * belong to.
 */

class ArticulatedLinkManager
    : public esp::physics::RigidBaseManager<ManagedArticulatedLink> {
 public:
  explicit ArticulatedLinkManager()
      : esp::physics::RigidBaseManager<
            ManagedArticulatedLink>::RigidBaseManager("ArticulatedObject") {
    // build this manager's copy constructor map
    this->copyConstructorMap_["ManagedArticulatedLink"] =
        &ArticulatedLinkManager::createObjectCopy<ManagedArticulatedLink>;
  }

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
  std::shared_ptr<ManagedArticulatedLink> initNewObjectInternal(
      CORRADE_UNUSED const std::string& objectHandle,
      CORRADE_UNUSED bool builtFromConfig) override {
    return ManagedArticulatedLink::create();
  }  // ArticulatedLinkManager::initNewObjectInternal(
 public:
  ESP_SMART_POINTERS(ArticulatedLinkManager)
};  // class ArticulatedLinkManager

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_ARTICULATEDLINKMANAGER_H
