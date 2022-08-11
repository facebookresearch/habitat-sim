// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/PlacementHelper.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/GlmUtils.h"

#include "esp/core/Check.h"

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

PlacementHelper::PlacementHelper(const ColumnGridSet& columnGridSet,
                                 const CollisionBroadphaseGrid& colGrid,
                                 const serialize::Collection& collection,
                                 esp::core::Random& random,
                                 int maxFailedPlacements)
    : columnGridSet_(columnGridSet),
      colGrid_(colGrid),
      collection_(collection),
      random_(random),
      maxFailedPlacements_(maxFailedPlacements) {
  BATCHED_SIM_ASSERT(maxFailedPlacements > 0);
}

bool PlacementHelper::place(Magnum::Matrix4& heldObjMat,
                            const FreeObject& freeObject,
                            const Mn::Vector3* fallbackPos) const {
  int numFailedPlacements = 0;
  bool success = false;

  Mn::Vector3 minCornerRotated;
  {
    constexpr int numBoxCorners = 8;
    for (int i = 0; i < numBoxCorners; i++) {
      Mn::Vector3 cornerLocal = getRangeCorner(freeObject.aabb_, i);
      const auto cornerRotated = heldObjMat.transformVector(cornerLocal);
      if (i == 0 || cornerRotated.y() < minCornerRotated.y()) {
        minCornerRotated = cornerRotated;
      }
    }
  }

  while (true) {
    Cr::Containers::Optional<Mn::Rad> hintAngle;

    // todo: smarter caching for held object spheres
    ColumnGridSource::QueryCacheValue queryCache = 0;
    // cast a single small sphere down to find a resting position candidate
    const float sphereRadius = 0.015f;  // temp hack
    const float radiusIdx = 0;          // temp hack

    Mn::Vector3 querySphereWorldOrigin;
    {
      auto centerWorld = heldObjMat.transformPoint(freeObject.aabb_.center());

      auto minCornerWorld = minCornerRotated + heldObjMat.translation();
      // cast down from center of object in XZ, at bottom of object (y of lowest
      // corner), and adjust for sphere radius
      querySphereWorldOrigin = Mn::Vector3(
          centerWorld.x(), minCornerWorld.y() + sphereRadius, centerWorld.z());
    }

    float queryCastDownResult = columnGridSet_.castDownTest(
        radiusIdx, querySphereWorldOrigin, &queryCache);
    // there's no guarantee of collision-free due to rotation on drop
    // todo: decide how to handle bad collisions
    float querySurfaceY =
        querySphereWorldOrigin.y() - queryCastDownResult - sphereRadius;

    // sloppy; we should have a better way of knowing that castDownTest didn't
    // hit anything
    constexpr float maxValidCastDownDist = 1000.f;
    if (queryCastDownResult <= 0.f ||
        queryCastDownResult > maxValidCastDownDist) {
      // two unexpected cases here:
      //   negative queryCastDownResult => object is already penetrating column
      //   grid queryCastDownResult > 1000 => didn't hit anything (outside
      //   environment bounds?)
    } else {
      // Test all spheres and find max y-fixup so that all spheres clear surface
      // by snapToSurfacePadY margin. Disallow a large fixup, as this
      // corresponds to e.g. dropping close to a wall.
      bool didHitColumnGrid = false;
      float fixupY =
          0.f;  // todo: consider -FLT_MAX here (allow negative fixup)
      for (const auto& sphere : freeObject.collisionSpheres_) {
        const auto& sphereLocalOrigin = sphere.origin;
        const float sphereRadius =
            getCollisionRadius(collection_, sphere.radiusIdx);
        auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

        float thisCastDownResult = columnGridSet_.castDownTest(
            sphere.radiusIdx, sphereWorldOrigin, &queryCache);

        float thisSurfaceY =
            sphereWorldOrigin.y() - thisCastDownResult - sphereRadius;

        constexpr float snapToSurfacePadY = 0.001f;  // 1 mm
        float thisFixupY =
            queryCastDownResult - thisCastDownResult + snapToSurfacePadY;

        // This threshold is to distinguish (1) a tilted object's corner being
        // a little too low versus (2) an object dropped next to a wall, with
        // some spheres interpenetrating the wall. We want to fix up for 1 but
        // reject 2.
        constexpr float maxAllowedFixupY = 0.05f;
        if (thisFixupY > maxAllowedFixupY) {
          didHitColumnGrid = true;
          break;
        }

        fixupY = Mn::Math::max(thisFixupY, fixupY);
      }

      int hitFreeObjectIndex = -1;
      if (!didHitColumnGrid) {
        // snap down, apply fixup, and test against collision grid
        heldObjMat.translation().y() -= (queryCastDownResult - fixupY);

        for (const auto& sphere : freeObject.collisionSpheres_) {
          const auto& sphereLocalOrigin = sphere.origin;
          const float sphereRadius =
              getCollisionRadius(collection_, sphere.radiusIdx);
          // perf todo: avoid re-computing sphereWorldOrigin for every sphere
          auto sphereWorldOrigin = heldObjMat.transformPoint(sphereLocalOrigin);

          hitFreeObjectIndex =
              colGrid_.contactTest(sphereWorldOrigin, sphereRadius);
          if (hitFreeObjectIndex != -1) {
            break;
          }
        }

        if (hitFreeObjectIndex == -1) {
          success = true;
          break;
        } else {
          // restore to drop height
          heldObjMat.translation().y() += (queryCastDownResult - fixupY);
        }
      }

      // if we hit an object, compute a hint
      if (hitFreeObjectIndex != -1 &&
          numFailedPlacements < maxFailedPlacements_ - 1) {
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

    heldObjMat.translation() +=
        Mn::Vector3(Mn::Math::cos(*hintAngle), 0.f, Mn::Math::sin(*hintAngle)) *
        bumpDist;
  }

  if (!success && fallbackPos) {
    heldObjMat.translation() = *fallbackPos;
    success = true;
  }

  return success;
}

}  // namespace batched_sim
}  // namespace esp
