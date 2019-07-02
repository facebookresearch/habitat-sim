// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <vector>

#include "esp/core/esp.h"
#include <Magnum/GL/Mesh.h>
#include <Corrade/Containers/Optional.h>
#include <Corrade/Containers/Reference.h>
#include <Magnum/Trade/MeshData3D.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Mesh.h>
#include "MeshData.h"

namespace esp {
namespace assets {

//! Raw mesh data storage
struct MeshData {
  //! Vertex positions
  std::vector<vec3f>    vbo;
  //! Vertex normals
  std::vector<vec3f>    nbo;
  //! Texture coordinates
  std::vector<vec2f>    tbo;
  //! Vertex colors
  std::vector<vec3f>    cbo;
  //! Index buffer
  std::vector<uint32_t> ibo;
};

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
