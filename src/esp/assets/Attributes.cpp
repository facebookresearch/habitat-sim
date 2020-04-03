// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Attributes.h"

namespace esp {
namespace assets {

//----------------------------------------//
//  Derived attribute implementations
//----------------------------------------//

PhysicsObjectAttributes::PhysicsObjectAttributes() {
  // fill necessary attribute defaults
  setMass(1.0);
  setMargin(0.01);
  setScale({1.0, 1.0, 1.0});
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setLinearDamping(0.2);
  setAngularDamping(0.2);
  setOriginHandle("");
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  setRequiresLighting(true);
}

PhysicsSceneAttributes::PhysicsSceneAttributes() {
  setGravity({0, -9.8, 0});
  setFrictionCoefficient(0.4);
  setRestitutionCoefficient(0.05);
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
}

PhysicsManagerAttributes::PhysicsManagerAttributes() {
  setSimulator("none");
  setTimestep(0.01);
  setMaxSubsteps(10);
}

}  // namespace assets
}  // namespace esp
