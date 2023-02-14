// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_RIGIDSTATE_H_
#define ESP_CORE_RIGIDSTATE_H_

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>

#include "Esp.h"

namespace esp {
namespace core {
/**
 * @brief describes the state of a rigid object as a composition of rotation
 * (quaternion) and translation.
 */
struct RigidState {
  RigidState() = default;
  RigidState(const Magnum::Quaternion& _rotation,
             const Magnum::Vector3& _translation)
      : rotation(_rotation), translation(_translation){};

  Magnum::Quaternion rotation;
  Magnum::Vector3 translation;

  ESP_SMART_POINTERS(RigidState)
};
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_RIGIDSTATE_H_
