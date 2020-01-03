// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief Struct @ref esp::assets::CollisionMeshData
 */

#include <vector>

#include <Magnum/GL/Mesh.h>
#include <Magnum/Magnum.h>
#include <Magnum/Mesh.h>
#include <Magnum/Trade/MeshData3D.h>
#include "esp/core/esp.h"

namespace esp {
namespace assets {
/**
 * @brief Provides references to geometry and topology for an individual
 * component of an asset for use in generating collision shapes for simulation.
 *
 * Usage: (1) for creating collision mesh/convex in @ref
 * physics::BulletPhysicsManager and @ref physics::BulletRigidObject
 */
struct CollisionMeshData {
  /**
   * @brief Primitive type (has to be triangle for Bullet to work).
   *
   * See @ref BulletRigidObject::constructBulletCompoundFromMeshes.
   */
  Magnum::MeshPrimitive primitive;
  /**
   * @brief Reference to Vertex positions.
   */
  Corrade::Containers::ArrayView<Magnum::Vector3> positions;
  /**
   * @brief Reference to Vertex indices.
   */
  Corrade::Containers::ArrayView<Magnum::UnsignedInt> indices;
};

}  // namespace assets
}  // namespace esp
