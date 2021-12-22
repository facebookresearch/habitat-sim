// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ColumnGridBuilder.h"
#include "esp/sim/Simulator.h"
#include "esp/geo/geo.h"
#include "esp/physics/PhysicsManager.h"
#include "esp/core/logging.h"

#include <Corrade/Utility/Assert.h>
#include <Magnum/Math/Range.h>


namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

ColumnGridSource ColumnGridBuilder::build(esp::sim::Simulator& sim, const Mn::Range3D& aabb, 
    float sphereRadius, float gridSpacing) {
  CORRADE_INTERNAL_ASSERT(gridSpacing > 0.f);
  CORRADE_INTERNAL_ASSERT(sphereRadius > 0.f);

  constexpr float searchStepSize = 0.01;
  Mn::Range3D adjustedAabb = aabb.padded(Mn::Vector3(
    sphereRadius + searchStepSize, 
    sphereRadius + searchStepSize,
    sphereRadius + searchStepSize));

  ColumnGridSource source;
  source.sphereRadius = sphereRadius;
  source.gridSpacing = gridSpacing;

  source.minX = adjustedAabb.min().x();
  source.minZ = adjustedAabb.min().z();

  #if 1
  source.dimX = int(adjustedAabb.sizeX() / gridSpacing) + 1;
  source.dimZ = int(adjustedAabb.sizeZ() / gridSpacing) + 1;
  CORRADE_INTERNAL_ASSERT(source.dimX * gridSpacing >= adjustedAabb.sizeX());
  CORRADE_INTERNAL_ASSERT(source.dimZ * gridSpacing >= adjustedAabb.sizeZ());
  #else
  source.dimX = 1; // temp int(adjustedAabb.sizeX() / gridSpacing) + 1;
  source.dimZ = 1; // temp int(adjustedAabb.sizeZ() / gridSpacing) + 1;
  #endif

  constexpr Mn::Vector3 upVec(0.f, 1.f, 0.f);
  constexpr Mn::Vector3 downVec(0.f, -1.f, 0.f);
  for (int cellZ = 0; cellZ < source.dimZ; cellZ++) {

    if (cellZ % 10 == 0) {
      ESP_DEBUG() << cellZ << "/" << source.dimZ;
    }

    for (int cellX = 0; cellX < source.dimX; cellX++) {

      Mn::Vector3 rayOrigin(
        source.minX + cellX * gridSpacing,
        adjustedAabb.min().y(),
        source.minZ + cellZ * gridSpacing);

      // Note an additional pad equal to the Bullet collision margin is also required
      // here. For gala kinematic tools build, we force collision margin to zero.
      constexpr float contactTestPad = 0.00f + 0.002f;

      float remainingZ = adjustedAabb.sizeY();

      float freeMinY = ColumnGridSource::INVALID_Y;
      float freeMaxY = ColumnGridSource::INVALID_Y;

      if (!sim.contactTestSphere(rayOrigin, sphereRadius + contactTestPad)) {
        freeMinY = rayOrigin.y();
      } else {
        // We're in collision with an obstacle at the bottom of this cell. Record the 
        // top of a "null" free column to simplify bookeeping.
        freeMaxY = rayOrigin.y();
        rayOrigin.y() += searchStepSize;
      }

      while (true) {

        // we have the bottom of a free column, so up-cast to find the top
        if (freeMinY != ColumnGridSource::INVALID_Y) {
          float distToMaxZ = adjustedAabb.max().y() - rayOrigin.y();
          CORRADE_INTERNAL_ASSERT(distToMaxZ > 0.f);
          esp::geo::Ray ray(rayOrigin, upVec);
          esp::physics::RaycastResults upResults = sim.castSphere(ray, sphereRadius, distToMaxZ);
          CORRADE_INTERNAL_ASSERT(upResults.hits.size() <= 1);
          if (!upResults.hasHits()) {
            // End the last free column and we're done with this cell.
            freeMaxY = adjustedAabb.max().y();
            source.appendColumn(cellX, cellZ, freeMinY, freeMaxY);
            break;
          }

          const auto& upHit = upResults.hits.front();

          // We tentatively found the bottom of a filled column. This z value represents the
          // height of a sphere that is contacting the bottom of the obstacle. (It 
          // does *not* represent the height of the surface of the obstacle.)
          // However, we will verify the presence of an obstacle in the next search
          // phase and possible discard this obstacle (so don't add the free column yet).
          freeMaxY = rayOrigin.y() + upHit.rayDistance;

          rayOrigin.y() = upHit.point.y() + searchStepSize;
        }

        // Now search above the hit point until we're clear of the obstacle. This is
        // to verify the obstacle and also find the bottom of the next free column.
        while (true) {
          
          bool stillInCollision = false;
          if (sim.contactTestSphere(rayOrigin, sphereRadius + contactTestPad)) {
            stillInCollision = true;
          } else {
              
            float distDownToFreeColumnMaxZ = rayOrigin.y() - freeMaxY;
            CORRADE_INTERNAL_ASSERT(distDownToFreeColumnMaxZ > 0.f);
            esp::geo::Ray ray(rayOrigin, downVec);
            esp::physics::RaycastResults downResults = sim.castSphere(
              ray, sphereRadius, distDownToFreeColumnMaxZ);
            CORRADE_INTERNAL_ASSERT(downResults.hits.size() <= 1);

            const auto& downHit = downResults.hits.front();

            if (!downResults.hasHits()) {
              // might fail to re-hit the recently-hit obstacle, for grazing hits, 
              // due to numeric inprecision and nondeterminism. Let's ignore this
              // obstacle. Let's start a free column if necessary.
              freeMaxY = ColumnGridSource::INVALID_Y;
              if (freeMinY == ColumnGridSource::INVALID_Y) {
                freeMinY = adjustedAabb.min().y();
              }
              break;
            }

            CORRADE_INTERNAL_ASSERT(downHit.rayDistance > 0.f);
            if (downHit.rayDistance == 0.f) {
              // todo: get rid of this if. See assert above.
              // I think this happens because of collision margin (different result
              // for sphere contact test versus sphere sweep).
              stillInCollision = true;
            } else {

              if (freeMinY != ColumnGridSource::INVALID_Y) {
                CORRADE_INTERNAL_ASSERT(freeMaxY != ColumnGridSource::INVALID_Y);
                // we've validated the recent obstacle (hitting from the bottom *and*
                // the top), so let's add the free column.
                source.appendColumn(cellX, cellZ, freeMinY, freeMaxY);
              }

              // start new free column at top of recently-hit obstacle
              freeMinY = rayOrigin.y() - downHit.rayDistance;
              freeMaxY = ColumnGridSource::INVALID_Y;
              break;
            }
          }

          CORRADE_INTERNAL_ASSERT(stillInCollision);          
          // Try the clearance tests again from slightly higher.
          rayOrigin.y() += searchStepSize;
          if (rayOrigin.y() >= adjustedAabb.max().y()) {
            // We're at the top of our AABB
            break;
          }
        }

        if (rayOrigin.y() >= adjustedAabb.max().y()) {
          if (freeMinY == ColumnGridSource::INVALID_Y) {
            // finish cell without finding the start of another free column
            break;
          } else {
            if (freeMaxY == ColumnGridSource::INVALID_Y) {
              // finish cell with free column extending up to extent max
              freeMaxY = adjustedAabb.max().y();
            } else {
              // we found the top of the free column earlier but didn't validate it.
              // We consider it validated now.
            }
            source.appendColumn(cellX, cellZ, freeMinY, freeMaxY);
            break;
          }
        }

        // continue with next up-cast to find next column
        CORRADE_INTERNAL_ASSERT(freeMinY != ColumnGridSource::INVALID_Y);
        CORRADE_INTERNAL_ASSERT(rayOrigin.y() > freeMinY);
      }
    }
  }

  return source;
}


}  // namespace batched_sim
}  // namespace esp
