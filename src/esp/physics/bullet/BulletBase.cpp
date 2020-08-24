// Copyright (c) Facebook, Inc. and its affiliates.
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
    bool join,
    btCompoundShape* bObjectShape) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;
  if (node.meshIDLocal != ID_UNDEFINED) {
    // This node has a mesh, so add it to the compound

    const assets::CollisionMeshData& mesh = meshGroup[node.meshIDLocal];

    if (join) {
      // add all points to a single convex instead of compounding (more
      // stable)
      if (bObjectConvexShapes_.empty()) {
        // create the convex if it does not exist
        bObjectConvexShapes_.emplace_back(
            std::make_unique<btConvexHullShape>());
      }

      // add points
      for (auto& v : mesh.positions) {
        bObjectConvexShapes_.back()->addPoint(
            btVector3(transformFromLocalToWorld.transformPoint(v)), false);
      }
    } else {
      bObjectConvexShapes_.emplace_back(std::make_unique<btConvexHullShape>(
          static_cast<const btScalar*>(mesh.positions.data()->data()),
          mesh.positions.size(), sizeof(Magnum::Vector3)));
      bObjectConvexShapes_.back()->setMargin(0.0);
      bObjectConvexShapes_.back()->recalcLocalAabb();
      //! Add to compound shape stucture
      if (bObjectShape != nullptr) {
        bObjectShape->addChildShape(btTransform{transformFromLocalToWorld},
                                    bObjectConvexShapes_.back().get());
      }
    }
  }

  for (auto& child : node.children) {
    constructConvexShapesFromMeshes(transformFromLocalToWorld, meshGroup, child,
                                    join, bObjectShape);
  }
}  // constructConvexShapesFromMeshes

void BulletBase::constructJoinedConvexShapeFromMeshes(
    const Magnum::Matrix4& transformFromParentToWorld,
    const std::vector<assets::CollisionMeshData>& meshGroup,
    const assets::MeshTransformNode& node,
    btConvexHullShape* bConvexShape) {
  Magnum::Matrix4 transformFromLocalToWorld =
      transformFromParentToWorld * node.transformFromLocalToParent;

  if (bConvexShape == nullptr) {
    Cr::Utility::Debug()
        << "constructJoinedConvexShapeFromMeshes : E - passed in null shape...";
    return;
  }
  if (node.meshIDLocal != ID_UNDEFINED) {
    const assets::CollisionMeshData& mesh = meshGroup[node.meshIDLocal];

    // add points
    for (auto& v : mesh.positions) {
      bConvexShape->addPoint(
          btVector3(transformFromLocalToWorld.transformPoint(v)), false);
    }
  }
  for (auto& child : node.children) {
    constructJoinedConvexShapeFromMeshes(transformFromLocalToWorld, meshGroup,
                                         child, bConvexShape);
  }
}

}  // namespace physics
}  // namespace esp
