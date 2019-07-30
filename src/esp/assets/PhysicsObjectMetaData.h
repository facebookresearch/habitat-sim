// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

namespace esp {

namespace assets {

// for each physical object we store:
// render mesh, collision mesh, physical parameters:
//“mass”
//“COM”: in object local space (unless we preprocess the mesh such that this is
// origin) inertia tensor “friction coefficient” “restitution coefficient”
struct PhysicsObjectMetaData {
  // construct this is loadObject() from default
  // ALEX TODO: initialize inertia to identity be default...
  PhysicsObjectMetaData()
      : mass(1.0),
        scale(1.0),
        COM(Magnum::Vector3d(0)),
        frictionCoefficient(0.5),
        restitutionCoefficient(0.6),
        margin(0.01),
        linDamping(0.2),
        angDamping(0.2),
        inertia(Magnum::Vector3(0., 0., 0.)){};

  // copy constructor
  PhysicsObjectMetaData(const PhysicsObjectMetaData& val) {
    renderMeshHandle = val.renderMeshHandle;
    collisionMeshHandle = val.collisionMeshHandle;
    mass = val.mass;
    scale = val.scale;
    COM = Magnum::Vector3d(val.COM);
    inertia = val.inertia;
    frictionCoefficient = val.frictionCoefficient;
    restitutionCoefficient = val.restitutionCoefficient;
  };

  // mesh handles provide keys to find the meshes in
  // resourceManager->resourceDict_
  std::string renderMeshHandle;
  std::string collisionMeshHandle;

  // physical properties of objects
  double mass;
  double margin;
  double scale;  // Alex: uniform scale. Think about mass->scale ratio defaults.
                 // Include a full transformation matrix option?
  Magnum::Vector3d COM;
  Magnum::Vector3 inertia;
  double frictionCoefficient;
  double restitutionCoefficient;

  double linDamping;
  double angDamping;
};

}  // namespace assets
}  // namespace esp
