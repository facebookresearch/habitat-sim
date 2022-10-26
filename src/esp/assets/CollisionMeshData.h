// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_ASSETS_COLLISIONMESHDATA_H_
#define ESP_ASSETS_COLLISIONMESHDATA_H_

/** @file
 * @brief Struct @ref esp::assets::CollisionMeshData
 */

#include <Corrade/Containers/ArrayView.h>
#include <Magnum/Magnum.h>
#include "esp/core/Esp.h"

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
   * See @ref BulletRigidObject::constructConvexShapesFromMeshes.
   */
  Magnum::MeshPrimitive primitive{};

  /**
   * @brief Reference to vertex positions.
   *
   * Bullet requires positions to be stored in a contiguous array, but MeshData
   * usually doesn't store them like that (and moreover the data might be
   * packed to smaller type). Thus the data are unpacked into a contiguous
   * array which is then referenced here.
   */
  Corrade::Containers::ArrayView<Magnum::Vector3> positions;

  /**
   * @brief Reference to vertex indices.
   *
   * If a MeshData already stores indices in desired type, this view references
   * them. If not (for example because indices are packed to a smaller type),
   * the data are unpacked to an internal data store and this view references
   * that instead.
   */
  Corrade::Containers::ArrayView<Magnum::UnsignedInt> indices;
};

}  // namespace assets
}  // namespace esp

#endif  // ESP_ASSETS_COLLISIONMESHDATA_H_
