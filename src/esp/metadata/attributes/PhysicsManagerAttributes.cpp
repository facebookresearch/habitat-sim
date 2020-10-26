// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManagerAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

PhysicsManagerAttributes::PhysicsManagerAttributes(const std::string& handle)
    : AbstractAttributes("PhysicsManagerAttributes", handle) {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}  // PhysicsManagerAttributes ctor

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
