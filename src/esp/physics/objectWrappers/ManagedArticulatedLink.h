// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_PHYSICS_MANAGEDARTICULATEDLINK_H_
#define ESP_PHYSICS_MANAGEDARTICULATEDLINK_H_

#include "ManagedRigidBase.h"
#include "esp/physics/ArticulatedObject.h"

namespace esp {
namespace physics {

/**
 * @brief Class describing wrapper for ArticulatedObject constructions.
 * Provides bindings for all ArticulatedObject-specific functionality.
 */
class ManagedArticulatedLink : public esp::physics::AbstractManagedRigidBase<
                                   esp::physics::ArticulatedLink> {
 public:
  ManagedArticulatedLink()
      : AbstractManagedRigidBase<esp::physics::ArticulatedLink>(
            "ManagedArticulatedLink") {}

 public:
  ESP_SMART_POINTERS(ManagedArticulatedLink)
};

}  // namespace physics
}  // namespace esp

#endif  // ESP_PHYSICS_MANAGEDARTICULATEDLINK_H_
