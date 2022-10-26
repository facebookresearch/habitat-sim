// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "BulletBase.h"
#include <Magnum/BulletIntegration/Integration.h>

namespace Mn = Magnum;
namespace Cr = Corrade;

namespace esp {
namespace physics {

// recursively create the convex mesh shapes and add them to the compound in a
// flat manner by accumulating transformations down the tree
void BulletBase::constructConvexShapesFromMeshes(
    const Magnum::Matrix4& transformFromParentToWorld,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    const assets::MeshTransformNode& node,
    btCompoundShape* bObjectShape,
    std::vector<std::unique_ptr<btConvexHullShape>>& bObjectConvexShapes) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;
  if (node.meshIDLocal != ID_UNDEFINED) {
    // This node has a mesh, so add it to the compound
    const assets::CollisionMeshData& mesh = meshGroup[node.meshIDLocal];

    bObjectConvexShapes.emplace_back(std::make_unique<btConvexHullShape>());
    // transform points into world space, including any scale/shear in
    // transformFromLocalToWorld.
    for (auto& v : mesh.positions) {
      bObjectConvexShapes.back()->addPoint(
          btVector3(transformFromLocalToWorld.transformPoint(v)), false);
    }
    bObjectConvexShapes.back()->setMargin(0.0);
    bObjectConvexShapes.back()->recalcLocalAabb();
    //! Add to compound shape structure
    if (bObjectShape != nullptr) {
      bObjectShape->addChildShape(btTransform::getIdentity(),
                                  bObjectConvexShapes.back().get());
    }
  }

  for (const auto& child : node.children) {
    constructConvexShapesFromMeshes(transformFromLocalToWorld, meshGroup, child,
                                    bObjectShape, bObjectConvexShapes);
  }
}  // constructConvexShapesFromMeshes

void BulletBase::constructJoinedConvexShapeFromMeshes(
    const Magnum::Matrix4& transformFromParentToWorld,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    const assets::MeshTransformNode& node,
    btConvexHullShape* bConvexShape) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  assert(bConvexShape != nullptr);

  if (node.meshIDLocal != ID_UNDEFINED) {
    const assets::CollisionMeshData& mesh = meshGroup[node.meshIDLocal];

    // add points
    for (auto& v : mesh.positions) {
      bConvexShape->addPoint(
          btVector3(transformFromLocalToWorld.transformPoint(v)), false);
    }
  }
  for (const auto& child : node.children) {
    constructJoinedConvexShapeFromMeshes(transformFromLocalToWorld, meshGroup,
                                         child, bConvexShape);
  }
}

}  // namespace physics
}  // namespace esp
