// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

namespace esp {

namespace assets {

struct PhysicsSceneMetaData {
  PhysicsSceneMetaData()
      : simulator("none"),
        gravity_(Magnum::Vector3d(0, -9.81, 0)),
        timestep_(0.01),
        frictionCoefficient_(0.4),
        maxSubsteps_(10) {
    objectLibraryPaths = std::vector<std::string>();
  };

  PhysicsSceneMetaData(const PhysicsSceneMetaData& val) {
    simulator = val.simulator;
    gravity_ = val.gravity_;
    frictionCoefficient_ = val.frictionCoefficient_;
    timestep_ = val.timestep_;
    maxSubsteps_ = val.maxSubsteps_;
    objectLibraryPaths = val.objectLibraryPaths;
  }
  // Alex TODO: pending MetaData and objectLibrary redesign...
  // relative paths to habitat-sim/data for object library members referenced by
  // this config
  std::vector<std::string> objectLibraryPaths;
  std::string simulator;
  Magnum::Vector3d gravity_;
  double timestep_;
  double frictionCoefficient_;
  int maxSubsteps_;
};

// for each physical object we store:
// render mesh, collision mesh, physical parameters:
//“mass”
//“COM”: in object local space (unless we preprocess the mesh such that this is
// origin) inertia tensor “friction coefficient” “restitution coefficient”
struct PhysicsObjectMetaData {
  PhysicsObjectMetaData()
      : mass_(1.0),
        margin_(0.01),
        scale_(1.0),
        COM_(Magnum::Vector3d(0)),
        inertia_(Magnum::Vector3(0., 0., 0.)),
        frictionCoefficient_(0.5),
        restitutionCoefficient_(0.6),
        linDamping_(0.2),
        angDamping_(0.2){};

  // copy constructor
  PhysicsObjectMetaData(const PhysicsObjectMetaData& val) {
    // originHandle_ = handle; //copying a metadata instance should create a new
    // handle if(originHandle_.empty()){
    //  originHandle_ = val.originHandle_+"_c";
    //  LOG(INFO) << "New handle not provided, generating: " << originHandle_;
    //}
    originHandle_ = val.originHandle_;
    renderMeshHandle_ = val.renderMeshHandle_;
    collisionMeshHandle_ = val.collisionMeshHandle_;
    mass_ = val.mass_;
    scale_ = val.scale_;
    COM_ = Magnum::Vector3d(val.COM_);
    inertia_ = Magnum::Vector3(val.inertia_);
    frictionCoefficient_ = val.frictionCoefficient_;
    restitutionCoefficient_ = val.restitutionCoefficient_;
    linDamping_ = val.linDamping_;
    angDamping_ = val.angDamping_;
  };

  //  origin filename of this metadata AND/OR if programmatically generated, the
  //  handle to its origin object in ResourceManager::physicsObjectLibrary_
  //  NOTE: this metaData instance and its origin may be modified and be
  //  therefore inconsistent.
  std::string originHandle_;
  std::string renderMeshHandle_;
  std::string collisionMeshHandle_;

  // physical properties of objects
  double mass_;
  double margin_;
  double scale_;  // Alex: uniform scale. Think about scale->mass->inertia ratio
                  // defaults.
  Magnum::Vector3d COM_;
  Magnum::Vector3 inertia_;
  double frictionCoefficient_;
  double restitutionCoefficient_;

  double linDamping_;
  double angDamping_;
};

}  // namespace assets
}  // namespace esp
