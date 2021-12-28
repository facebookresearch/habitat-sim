// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/GlmUtils.h"

#include <Magnum/Math/Range.h>

#include <vector>
#include <cmath>

namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

int CollisionBroadphaseGrid::getCellIndex(int cellX, int cellZ) const {
  // linear memory layout for grid; perf todo: space-filling
  return cellZ * dimX_ + cellX;
}

std::pair<int, int> CollisionBroadphaseGrid::findPointGridCellXZ(
  int gridIndex, float x, float z) const {

  float gridMinX = (gridIndex % 2 == 0) ? minX_ : minX_ - gridSpacing_ * 0.5f;
  float gridMinZ = ((gridIndex / 2) % 2 == 0) ? minZ_ : minZ_ - gridSpacing_ * 0.5f;

  const float cellFloatX = (x - gridMinX) * invGridSpacing_;
  if (cellFloatX < 0.f || cellFloatX >= (float)dimX_) { // perf todo: pre-cache float dimX
    BATCHED_SIM_ASSERT(false);
    return std::make_pair(-1, -1);
  }

  const float cellFloatZ = (z - gridMinZ) * invGridSpacing_;
  if (cellFloatZ < 0.f || cellFloatZ >= (float)dimZ_) {
    return std::make_pair(-1, -1);
  }

  // consider allowing a margin and then clamping, to account for numeric imprecision
  BATCHED_SIM_ASSERT(cellFloatX >= 0.f && cellFloatX < (float)dimX_);
  const int cellX = int(cellFloatX); // note truncation, not founding

  BATCHED_SIM_ASSERT(cellFloatZ >= 0.f && cellFloatZ < (float)dimZ_);
  const int cellZ = int(cellFloatZ);

  return {cellX, cellZ};
}

void CollisionBroadphaseGrid::findObstacleGridCellIndices(float queryMinX, float queryMinZ, float queryMaxX, 
  float queryMaxZ, std::vector<std::pair<int, int>>& gridCellIndices) const {

  gridCellIndices.resize(0);

  for (int gridIndex = 0; gridIndex < NUM_GRIDS; gridIndex++) {

    auto [minCellX, minCellZ] = findPointGridCellXZ(gridIndex, queryMinX, queryMinZ);
    auto [maxCellX, maxCellZ] = findPointGridCellXZ(gridIndex, queryMaxX, queryMaxZ);

    int numCells = (maxCellZ - minCellZ + 1) * (maxCellX - minCellZ + 1);
    for (int cellZ = minCellZ; cellZ <= maxCellZ; cellZ++) {
      for (int cellX = minCellX; cellX <= maxCellX; cellX++) {
        int cellIndex = getCellIndex(cellX, cellZ);
        gridCellIndices.emplace_back(std::make_pair(gridIndex, cellIndex));
      }
    }
  }
}

std::pair<int, int> CollisionBroadphaseGrid::findSphereGridCellIndex(float x, float z) const {

  // use the max corner of the sphere to do lookup
  const float halfCellFloatX = (x + maxSphereRadius_ - minX_) * invGridSpacing_ * 2.f;
  if (halfCellFloatX < 0.f || halfCellFloatX >= (float)(dimX_ * 2)) {
    BATCHED_SIM_ASSERT(false);
    return std::make_pair(-1, -1);
  }

  const float halfCellFloatZ = (z + maxSphereRadius_ - minZ_) * invGridSpacing_ * 2.f;
  if (halfCellFloatZ < 0.f || halfCellFloatZ >= (float)(dimZ_ * 2)) {
    BATCHED_SIM_ASSERT(false);
    return std::make_pair(-1, -1);
  }

  const int halfCellX = int(halfCellFloatX); // note truncation, not rounding
  const int halfCellZ = int(halfCellFloatZ); // note truncation, not rounding
  
  int gridIndex = (halfCellZ % 2 ? 0 : 1) * 2 + (halfCellX % 2 ? 0 : 1);
  int cellX = halfCellX >> 1;
  int cellZ = halfCellZ >> 1;
  BATCHED_SIM_ASSERT(cellX >= 0 && cellX < dimX_);
  BATCHED_SIM_ASSERT(cellZ >= 0 && cellZ < dimZ_);

  int cellIndex = getCellIndex(cellX, cellZ);

  return std::make_pair(gridIndex, cellIndex);
}

CollisionBroadphaseGrid::CollisionBroadphaseGrid(float sphereRadius, float minX, float minZ, 
  float maxX, float maxZ, int maxBytes, float maxGridSpacing, int numObstaclesToReserve)
  : minX_(minX)
  , minZ_(minZ)
  , maxSphereRadius_(sphereRadius) {

  // For each cell in a grid, we cache all the obstacles that overlap it. Obstacles
  // generally span multiple cells.
  // We use a 2D grid in the ground (XZ) plane. We assume limited complexity (number
  // of obstacles) in Y (MAX_OBSTACLES_PER_CELL).
  // For a contactTest query for a given sphere, we want to be able to grab a single
  // cell that contains all obstacles near the sphere.
  // In practice, for any grid, an arbitrary sphere may often lie on the boundary
  // between cells, thus requiring not 1 but up to 4 cells (and careful merging of the
  // obstacles cached at each cell to avoid duplicates).
  // The merging/deduplicating would be too expensive.

  // So, instead of one grid, we'll have 4 grids that redundantly span the query region.
  // Each has a different offset, such that an arbitrary query sphere will fit completely inside 
  // 1 of the 4 grids.

  // To summarize, having 4 redundant grids will:
  // - use 4x memory for caching obstacles
  // - be 4x slower to insert/remove obstacles (but this operation is infrequent)
  // - allow a single fast lookup with no merging/deduplicating

  gridSpacing_ = maxSphereRadius_ * 4.f;
  while (true) {
    // note +2; we need +1 to "round up", and we need another +1 because we want an
    // additional row/column for our grids that are offset (that don't start at minX/minZ).
    dimX_ = int((maxX - minX) / gridSpacing_) + 2;
    dimZ_ = int((maxZ - minZ) / gridSpacing_) + 2;
    const int numCells = dimX_ * dimZ_ * NUM_GRIDS;
    const int bytes = sizeof(Cell) * numCells + sizeof(Obstacle) * numObstaclesToReserve;

    if (bytes > maxBytes) {
      gridSpacing_ *= 1.1f;
    } else {
      break;
    }
  }
  if (maxGridSpacing > 0.f) {
    // if you hit this, you need to reconsider MAX_OBSTACLES_PER_CELL, maxBytes, 
    // numObstaclesToReserve, and/or maxGridSpacing
    BATCHED_SIM_ASSERT(gridSpacing_ <= maxGridSpacing);
  }
  invGridSpacing_ = 1.f / gridSpacing_;

  // if we hit this, we should adapt the minGridSpacing logic above to avoid too-large grids
  BATCHED_SIM_ASSERT(dimX_ * dimZ_ < 10000000);

  const int numCells = dimX_ * dimZ_;
  for (int gridIndex = 0; gridIndex < NUM_GRIDS; gridIndex++) {
    grids_[gridIndex].cells.resize(numCells);
  }

  gridCellIndicesStorage_.reserve(MAX_OBSTACLES_PER_CELL);

  if (numObstaclesToReserve != -1) {
    BATCHED_SIM_ASSERT(numObstaclesToReserve > 0);
    obstacles_.reserve(numObstaclesToReserve);
  }
}

Mn::Range3D CollisionBroadphaseGrid::getWorldRange(int16_t obsIndex) const {

  const auto& obs = safeVectorGet(obstacles_, obsIndex);

  auto rotation = obs.invRotation.invertedNormalized();
  const auto& aabb = *obs.aabb;

  std::array<Mn::Vector3, 8> corners = {
    aabb.backBottomLeft(),
    aabb.backBottomRight(),
    aabb.backTopLeft(),
    aabb.backTopRight(),
    aabb.frontBottomLeft(),
    aabb.frontBottomRight(),
    aabb.frontTopLeft(),
    aabb.frontTopRight(),
  };

  Mn::Vector3 min = rotation.transformVectorNormalized(corners[0]);
  Mn::Vector3 max = min;
  for (int i = 1; i < 8; i++) {
    const auto rotatedCorner = rotation.transformVectorNormalized(corners[i]);
    min = Mn::Vector3(
      Mn::Math::min(min.x(), rotatedCorner.x()),
      Mn::Math::min(min.y(), rotatedCorner.y()),
      Mn::Math::min(min.z(), rotatedCorner.z()));
    max = Mn::Vector3(
      Mn::Math::max(max.x(), rotatedCorner.x()),
      Mn::Math::max(max.y(), rotatedCorner.y()),
      Mn::Math::max(max.z(), rotatedCorner.z()));
  }

  BATCHED_SIM_ASSERT(min.x() <= max.x());
  BATCHED_SIM_ASSERT(min.y() <= max.y());
  BATCHED_SIM_ASSERT(min.z() <= max.z());

  return Mn::Range3D(obs.pos + min, obs.pos + max);
}

CollisionBroadphaseGrid::Cell& CollisionBroadphaseGrid::getCell(std::pair<int, int> gridCellIndex) {
  auto& cell = safeVectorGet(
    safeVectorGet(grids_, gridCellIndex.first).cells,
    gridCellIndex.second);
  return cell;
}

const CollisionBroadphaseGrid::Cell& CollisionBroadphaseGrid::getCell(std::pair<int, int> gridCellIndex) const {
  const auto& cell = safeVectorGet(
    safeVectorGet(grids_, gridCellIndex.first).cells,
    gridCellIndex.second);
  return cell;
}

int16_t CollisionBroadphaseGrid::insertObstacle(const Magnum::Vector3& pos, const Magnum::Quaternion& rotation,
  const Magnum::Range3D* aabb) {

  int16_t obsIndex = obstacles_.size();
  obstacles_.push_back(Obstacle());
  auto& obs = obstacles_.back();
  obs.aabb = aabb;
  obs.pos.x() = NAN;

  reinsertObstacle(obsIndex, pos, rotation);

  return obsIndex;
}

void CollisionBroadphaseGrid::reinsertObstacle(int16_t obsIndex, const Magnum::Vector3& pos, 
  const Magnum::Quaternion& rotation) {

  BATCHED_SIM_ASSERT(isObstacleDisabled(obsIndex));
  auto& obs = safeVectorGet(obstacles_, obsIndex);
  obs.pos = pos;
  obs.invRotation = rotation.invertedNormalized();

  insertRemoveObstacleHelper(obsIndex, /*remove*/false);
}

void CollisionBroadphaseGrid::disableObstacle(int16_t obsIndex) {

  insertRemoveObstacleHelper(obsIndex, /*remove*/true);

  auto& obs = safeVectorGet(obstacles_, obsIndex);
  obs.pos.x() = NAN; // mark as uninserted

}

bool CollisionBroadphaseGrid::isObstacleDisabled(int16_t obsIndex) const {
  const auto& obs = safeVectorGet(obstacles_, obsIndex);
  return std::isnan(obs.pos.x());
}

const CollisionBroadphaseGrid::Obstacle& CollisionBroadphaseGrid::getObstacle(int obsIndex) const {
  const auto& obs = safeVectorGet(obstacles_, obsIndex);
  return obs;
}


void CollisionBroadphaseGrid::insertRemoveObstacleHelper(int16_t obsIndex, bool remove) {

  auto& obs = safeVectorGet(obstacles_, obsIndex);

  auto range = getWorldRange(obsIndex);
  Mn::Vector3 minWorld = range.min();
  Mn::Vector3 maxWorld = range.max();

  obs.worldMinY = minWorld.y();
  obs.worldMaxY = maxWorld.y();

  findObstacleGridCellIndices(minWorld.x(), minWorld.z(), maxWorld.x(), 
    maxWorld.z(), gridCellIndicesStorage_);

  for (const auto& gridCellIndex : gridCellIndicesStorage_) {
    auto& cell = getCell(gridCellIndex);
    if (remove) {
      // linear search for removal
      bool found = false;
      for (int i = 0; i < cell.numObstacles; i++) {
        auto& otherObsIndex = cell.obstacles[i];
        if (otherObsIndex == obsIndex) {
          if (i < cell.numObstacles - 1) {
            // backfill
            otherObsIndex = cell.obstacles[cell.numObstacles - 1];
          }
          cell.numObstacles--;
          numObstacleInstances_--;
          found = true;
          break;
        }
      }
      BATCHED_SIM_ASSERT(found);
    } else {
      BATCHED_SIM_ASSERT(cell.numObstacles < MAX_OBSTACLES_PER_CELL);
      cell.obstacles[cell.numObstacles] = obsIndex;
      cell.numObstacles++;
      numObstacleInstances_++;
    }
  }

}

int CollisionBroadphaseGrid::contactTest(const Magnum::Vector3& spherePos, float sphereRadius) const {

  BATCHED_SIM_ASSERT(sphereRadius <= maxSphereRadius_);

  const float sphereRadiusSq = sphereRadius * sphereRadius;
  const auto& cell = getCell(findSphereGridCellIndex(spherePos.x(), spherePos.z()));

  for (int i = 0; i < cell.numObstacles; i++) {
    int16_t obsIndex = cell.obstacles[i];
    const auto& obs = safeVectorGet(obstacles_, obsIndex);

    // test y extents
    if (obs.worldMaxY < spherePos.y() - sphereRadius
      || obs.worldMinY > spherePos.y() + sphereRadius) {
      continue;
    }

    const auto posLocal = obs.invRotation.transformVectorNormalized(spherePos - obs.pos);

    //bool sphereBoxContactTest(const Magnum::Vector3& sphereOrigin, float sphereRadiusSq, const Magnum::Range3D& aabb);
    if (sphereBoxContactTest(posLocal, sphereRadiusSq, *obs.aabb)) {
#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG  
      debugLastContact_.obsPos = obs.pos;
      debugLastContact_.obsLocalAabb = *obs.aabb;
      debugLastContact_.obsRotation = obs.invRotation.invertedNormalized();
#endif
      return obsIndex;
    }
  }

  return -1;

}




}  // namespace batched_sim
}  // namespace esp
