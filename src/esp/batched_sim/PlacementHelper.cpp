// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/PlacementHelper.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/core/Check.h"

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

PlacementHelper::PlacementHelper(const ColumnGridSet& columnGridSet, 
  const CollisionBroadphaseGrid& colGrid,
  const serialize::Collection& collection,
  esp::core::Random& random, int maxFailedPlacements)
  : columnGridSet_(columnGridSet)
  , colGrid_(colGrid)
  , collection_(collection)
  , random_(random)
  , maxFailedPlacements_(maxFailedPlacements)
{
  BATCHED_SIM_ASSERT(maxFailedPlacements > 0);
}

bool PlacementHelper::place(Magnum::Matrix4& heldObjMat, const FreeObject& freeObject, 
    const Mn::Vector3* fallbackPos) const {

  BATCHED_SIM_ASSERT(freeObject.collisionSpheres_.size());

  int numFailedPlacements = 0;
  bool success = false;
  
  while (true) {
    float minCastDownDist = std::numeric_limits<float>::max();
    Cr::Containers::Optional<Mn::Rad> hintAngle;
    
    // todo: smarter caching for held object spheres
    ColumnGridSource::QueryCacheValue queryCache = 0;
    // cast a single small sphere down to find a resting position candidate
    {
      // todo: find a sphere near the center of mass, at the base of the object in its
      // current (or resting) rotation. We can use its rotated box to find this.
      const float sphereRadius = 0.015f; // temp hack
      const auto sphereLocalOrigin = Mn::Vector3(
        freeObject.aabb_.center().x(),
        freeObject.aabb_.center().y(),
        freeObject.aabb_.min().z() + sphereRadius);
      float radiusIdx = 0; // temp hack
      auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

      float castDownResult = columnGridSet_.castDownTest(radiusIdx, sphereWorldOrigin, &queryCache);
      // there's no guarantee of collision-free due to rotation on drop
      // todo: decide how to handle bad collisions
      minCastDownDist = Mn::Math::min(minCastDownDist, castDownResult);
    }
    constexpr float maxPenetrationDist = 0.2f;
    // sloppy; we should have a better way of knowing that castDownTest didn't hit anything
    constexpr float maxValidCastDownDist = 1000.f;
    if (minCastDownDist <= -maxPenetrationDist || minCastDownDist > maxValidCastDownDist) {
      // continue below, no hint
    } else {
      
      heldObjMat.translation().y() -= minCastDownDist;

      int hitFreeObjectIndex = -1;
      bool didHitColumnGrid = false;
      for (const auto& sphere : freeObject.collisionSpheres_) {        
        const auto& sphereLocalOrigin = sphere.origin;
        const float sphereRadius = getCollisionRadius(collection_, sphere.radiusIdx);
        auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

        hitFreeObjectIndex = colGrid_.contactTest(sphereWorldOrigin, sphereRadius);
        if (hitFreeObjectIndex != -1) {
          break;
        }

        // The intention of this test is to catch when an object has been dropped near
        // a wall or other part of the static scene, such that it's partially 
        // interpenetrating.
        // todo: consider vertical fixup here
        float castDownResult = columnGridSet_.castDownTest(sphere.radiusIdx, sphereWorldOrigin, &queryCache);
        // sloppy: this is needed because we currently drop tilted objects, and we
        // don't use a sphere at the true base of the object in the earlier castDownTest.
        constexpr float pad = 0.05f;
        const float maxPenetrationDist = sphereRadius + pad;
        if (castDownResult <= -maxPenetrationDist) {
          didHitColumnGrid = true;
          break;
        }
      }

      if (hitFreeObjectIndex == -1 && !didHitColumnGrid) {
        success = true;
        break;
      }

      heldObjMat.translation().y() += minCastDownDist; // restore to drop height

      if (hitFreeObjectIndex != -1 && numFailedPlacements < maxFailedPlacements_ - 1) {
        const auto& hitObs = colGrid_.getObstacle(hitFreeObjectIndex);
        auto delta = heldObjMat.translation() - hitObs.pos;
        delta.y() = 0.f;
        constexpr float eps = 1e-3f;
        if (delta.dot() > eps) {
          hintAngle = Mn::Rad(std::atan2(delta.z(), delta.x()));
        }
      }
    }

    numFailedPlacements++;
    if (numFailedPlacements >= maxFailedPlacements_) {
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

  if (!success && fallbackPos) {
    heldObjMat.translation() = *fallbackPos;
    success = true;
  }

  return success;
}


}  // namespace batched_sim
}  // namespace esp
