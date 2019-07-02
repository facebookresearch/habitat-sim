// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <vector>

#include "esp/core/esp.h"
#include <Magnum/GL/Mesh.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Magnum.h>
#include <Magnum/Mesh.h>

namespace esp {
namespace assets {

//! Reference to vertices and
//! Usage: (1) for creating collision mesh in Bullet
struct CollisionMeshData {
  //! Primitive type (has to be triangle for Bullet to work)
  Magnum::MeshPrimitive                               primitive;
  //! Reference to Vertex positions
  Corrade::Containers::ArrayView<Magnum::Vector3>     positions;
  //! Reference to Vertex indices
  Corrade::Containers::ArrayView<Magnum::UnsignedInt> indices;

};

}  // namespace assets
}  // namespace esp
