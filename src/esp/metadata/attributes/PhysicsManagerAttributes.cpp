// Copyright (c) Meta Platforms, Inc. and its affiliates.
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
  setRestitutionCoefficient(0.1);
}  // PhysicsManagerAttributes ctor

void PhysicsManagerAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("physics_simulator", jsonObj, allocator);
  writeValueToJson("timestep", jsonObj, allocator);
  writeValueToJson("gravity", jsonObj, allocator);
  writeValueToJson("friction_coefficient", jsonObj, allocator);
  writeValueToJson("restitution_coefficient", jsonObj, allocator);
}  // PhysicsManagerAttributes::writeValuesToJson

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
