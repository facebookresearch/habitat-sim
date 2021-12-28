// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/PlacementHelper.h"
#include "esp/batched_sim/BatchedSimAssert.h"

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

PlacementHelper::PlacementHelper(const ColumnGridSource& columnGrid, const CollisionBroadphaseGrid& colGrid,
  esp::core::Random& random, int maxFailedPlacements)
  : columnGrid_(columnGrid)
  , colGrid_(colGrid)
  , random_(random)
  , maxFailedPlacements_(maxFailedPlacements)
{
}

bool PlacementHelper::place(Magnum::Matrix4& heldObjMat, const FreeObject& freeObject) const {

  BATCHED_SIM_ASSERT(freeObject.collisionSphereLocalOrigins_.size());

  int numFailedPlacements = 0;
  bool success = false;
  while (true) {
    float minCastDownDist = std::numeric_limits<float>::max();
    Cr::Containers::Optional<Mn::Rad> hintAngle;
    
    // todo: smarter caching for held object spheres
    ColumnGridSource::QueryCacheValue queryCache = 0;
    for (int i = 0; i < freeObject.collisionSphereLocalOrigins_.size(); i++) {
      const auto& sphereLocalOrigin = freeObject.collisionSphereLocalOrigins_[i];
      constexpr float sphereRadius = 0.1f; // todo
      auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

      float castDownResult = columnGrid_.castDownTest(sphereWorldOrigin, &queryCache);
      // there's no guarantee of collision-free due to rotation on drop
      // todo: decide how to handle bad collisions
      minCastDownDist = Mn::Math::min(minCastDownDist, castDownResult);
    }
    constexpr float maxPenetrationDist = 0.2f;
    if (minCastDownDist <= -maxPenetrationDist) {
      // continue below, no hint
    } else {
      
      heldObjMat.translation().y() -= minCastDownDist;

      int hitFreeObjectIndex = -1;
      for (int i = 0; i < freeObject.collisionSphereLocalOrigins_.size(); i++) {
        const auto& sphereLocalOrigin = freeObject.collisionSphereLocalOrigins_[i];
        constexpr float sphereRadius = 0.1f; // todo
        auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

        hitFreeObjectIndex = colGrid_.contactTest(sphereWorldOrigin, sphereRadius);

// #ifdef ENABLE_DEBUG_INSTANCES
//         {
//           auto mat = Mn::Matrix4::translation(sphereWorldOrigin)
//             * Mn::Matrix4::scaling(Mn::Vector3(sphereRadius, sphereRadius, sphereRadius));
//           addDebugInstance((hitFreeObjectIndex == -1) ? "sphere_green" : "sphere_orange", b, mat);
//         }
// #endif

        if (hitFreeObjectIndex != -1) {
          break;
        }
      }

      if (hitFreeObjectIndex == -1) {
        success = true;
        break;
      }

      heldObjMat.translation().y() += minCastDownDist; // restore to drop height

      const auto& hitObs = colGrid_.getObstacle(hitFreeObjectIndex);
      auto delta = heldObjMat.translation() - hitObs.pos;
      delta.y() = 0.f;
      constexpr float eps = 1e-3f;
      if (delta.dot() > eps) {
        hintAngle = Mn::Rad(std::atan2(delta.z(), delta.x()));
      }
    }

    numFailedPlacements++;
    if (numFailedPlacements == maxFailedPlacements_) {
      break;
    }

    if (!hintAngle) {
      hintAngle = Mn::Rad(Mn::Deg(random_.uniform_float(0.f, 360.f)));
    } else {
      // use hint angle but also include randomness
      *hintAngle += Mn::Rad(Mn::Deg(random_.uniform_float(-60.f, 60.f)));
    }

    float bumpDist = freeObject.aabb_.size().length();
    
    heldObjMat.translation() += Mn::Vector3(
      Mn::Math::cos(*hintAngle), 0.f, Mn::Math::sin(*hintAngle)) * bumpDist;        
  }

  return success;
}


}  // namespace batched_sim
}  // namespace esp
