// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Attributes.h"

namespace esp {
namespace assets {

//----------------------------------------//
//  Derived attribute implementations
//----------------------------------------//

AbstractPhysAttributes::AbstractPhysAttributes(const std::string& originHandle)
    : Configuration() {
  setOriginHandle(originHandle);
  setFrictionCoefficient(0.5);
  setRestitutionCoefficient(0.1);
  setRenderMeshHandle("");
  setCollisionMeshHandle("");
}
// PhysicsAttributes is abstract; virtual destructor deleted; definition
// required so instancing class can destroy base
// AbstractPhysAttributes::~AbstractPhysAttributes() {}

PhysicsObjectAttributes::PhysicsObjectAttributes(
    const std::string& originHandle)
    : AbstractPhysAttributes(originHandle) {
  // fill necessary attribute defaults
  setMass(1.0);
  setMargin(0.04);
  setScale({1.0, 1.0, 1.0});
  setCOM({0, 0, 0});
  setInertia({0, 0, 0});
  setLinearDamping(0.2);
  setAngularDamping(0.2);

  setBoundingBoxCollisions(false);
  setJoinCollisionMeshes(true);
  setRequiresLighting(true);
  setIsVisible(true);
  setIsCollidable(true);
}

// PhysicsPrimitiveObjAttributes is abstract; virtual destructor deleted;
// definition required so instancing class can destroy base
AbstractPhysPrimObjAttributes::~AbstractPhysPrimObjAttributes() {}

PhysicsSceneAttributes::PhysicsSceneAttributes(const std::string& originHandle)
    : AbstractPhysAttributes(originHandle) {
  setGravity({0, -9.8, 0});
  // TODO do these defaults need to be maintained here?
  setFrictionCoefficient(0.4);
  setRestitutionCoefficient(0.05);
}

PhysicsManagerAttributes::PhysicsManagerAttributes(
    const std::string& originHandle)
    : Configuration() {
  setSimulator("none");
  setOriginHandle(originHandle);
  setTimestep(0.01);
  setMaxSubsteps(10);
}

}  // namespace assets
}  // namespace esp
