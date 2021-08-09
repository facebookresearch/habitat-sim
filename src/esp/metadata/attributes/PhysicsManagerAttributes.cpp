// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManagerAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

PhysicsManagerAttributes::PhysicsManagerAttributes(const std::string& handle)
    : AbstractAttributes("PhysicsManagerAttributes", handle) {
  setSimulator("bullet");
  setTimestep(0.008);
  setGravity({0, -9.8, 0});
  setFrictionCoefficient(0.4);
}  // PhysicsManagerAttributes ctor

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
