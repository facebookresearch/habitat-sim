// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PhysicsManagerAttributes.h"

namespace esp {
namespace assets {
namespace attributes {

const std::string PhysicsManagerAttributes::JSONConfigTestString =
    R"({
      "physics simulator": "bullet_test",
      "timestep": 1.0,
      "gravity": [1,2,3],
      "friction coefficient": 1.4,
      "restitution coefficient": 1.1
    })";

PhysicsManagerAttributes::PhysicsManagerAttributes(const std::string& handle)
    : AbstractAttributes("PhysicsManagerAttributes", handle) {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}  // PhysicsManagerAttributes ctor

}  // namespace attributes
}  // namespace assets
}  // namespace esp
