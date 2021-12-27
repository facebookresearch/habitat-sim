// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/batched_sim/GlmUtils.h"

#include <Magnum/Math/Range.h>

#include <vector>

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
  const float halfCellFloatX = (x + sphereRadius_ - minX_) * invGridSpacing_ * 2.f;
  if (halfCellFloatX < 0.f || halfCellFloatX >= (float)(dimX_ * 2)) {
    BATCHED_SIM_ASSERT(false);
    return std::make_pair(-1, -1);
  }

  const float halfCellFloatZ = (z + sphereRadius_ - minZ_) * invGridSpacing_ * 2.f;
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
  float maxX, float maxZ, int maxBytes, float maxGridSpacing)
  : minX_(minX)
  , minZ_(minZ)
  , sphereRadius_(sphereRadius) {

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

  gridSpacing_ = sphereRadius_ * 4.f;
  while (true) {
    // note +2; we need +1 to "round up", and we need another +1 because we want an
    // additional row/column for our grids that are offset (that don't start at minX/minZ).
    dimX_ = int((maxX - minX) / gridSpacing_) + 2;
    dimZ_ = int((maxZ - minZ) / gridSpacing_) + 2;
    const int numCells = dimX_ * dimZ_ * NUM_GRIDS;
    const int bytes = sizeof(Cell) * numCells;

    if (bytes > maxBytes) {
      gridSpacing_ *= 1.1f;
    } else {
      break;
    }
  }
  // if you hit this, you need to reconsider MAX_OBSTACLES_PER_CELL, maxBytes, and/or maxGridSpacing
  BATCHED_SIM_ASSERT(gridSpacing_ <= maxGridSpacing);
  invGridSpacing_ = 1.f / gridSpacing_;

  // if we hit this, we should adapt the minGridSpacing logic above to avoid too-large grids
  BATCHED_SIM_ASSERT(dimX_ * dimZ_ < 10000000);

  const int numCells = dimX_ * dimZ_;
  for (int gridIndex = 0; gridIndex < NUM_GRIDS; gridIndex++) {
    grids_[gridIndex].cells.resize(numCells);
  }

  gridCellIndicesStorage_.reserve(MAX_OBSTACLES_PER_CELL);
}

std::pair<Mn::Vector3, Mn::Vector3> CollisionBroadphaseGrid::getUnrotatedRangeMinMax(
  const Magnum::Range3D& aabb,
  const Magnum::Quaternion& rotation) {

#if 0
  const auto oneVector = Mn::Vector3(1.f, 1.f, 1.f);
  const auto invRotation = rotation.invertedNormalized();

  Mn::Vector3 signVector = invRotation.transformVectorNormalized(oneVector);

  Mn::Vector3 minCornerLocal(
    signVector.x() > 0.f ? aabb.min().x() : aabb.max().x(),
    signVector.y() > 0.f ? aabb.min().y() : aabb.max().y(),
    signVector.z() > 0.f ? aabb.min().z() : aabb.max().z());

  Mn::Vector3 maxCornerLocal(
    signVector.x() > 0.f ? aabb.max().x() : aabb.min().x(),
    signVector.y() > 0.f ? aabb.max().y() : aabb.min().y(),
    signVector.z() > 0.f ? aabb.max().z() : aabb.min().z());

  Mn::Vector3 minCornerUnrotated = rotation.transformVectorNormalized(minCornerLocal);
  Mn::Vector3 maxCornerUnrotated = rotation.transformVectorNormalized(maxCornerLocal);

  BATCHED_SIM_ASSERT(maxCornerUnrotated.x() >= minCornerUnrotated.x());
  BATCHED_SIM_ASSERT(maxCornerUnrotated.y() >= minCornerUnrotated.y());
  BATCHED_SIM_ASSERT(maxCornerUnrotated.z() >= minCornerUnrotated.z());

  return std::make_pair(minCornerUnrotated, maxCornerUnrotated);
#endif

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
    const auto worldCorner = rotation.transformVectorNormalized(corners[i]);
    min = Mn::Vector3(
      Mn::Math::min(min.x(), worldCorner.x()),
      Mn::Math::min(min.y(), worldCorner.y()),
      Mn::Math::min(min.z(), worldCorner.z()));
    max = Mn::Vector3(
      Mn::Math::max(max.x(), worldCorner.x()),
      Mn::Math::max(max.y(), worldCorner.y()),
      Mn::Math::max(max.z(), worldCorner.z()));
  }

  BATCHED_SIM_ASSERT(min.x() <= max.x());
  BATCHED_SIM_ASSERT(min.y() <= max.y());
  BATCHED_SIM_ASSERT(min.z() <= max.z());

  return std::make_pair(min, max);
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

void CollisionBroadphaseGrid::insertObstacle(const Magnum::Vector3& pos, 
  const Magnum::Quaternion& rotation, const Magnum::Range3D& aabb) {

  auto pair = getUnrotatedRangeMinMax(aabb, rotation);
  Mn::Vector3 minWorld = pos + pair.first;
  Mn::Vector3 maxWorld = pos + pair.second;

  Obstacle obsToInsert;
  obsToInsert.boxDim = aabb.size();
  BATCHED_SIM_ASSERT(obsToInsert.boxDim.x() >= 0.f && obsToInsert.boxDim.y() >= 0.f && obsToInsert.boxDim.z() >= 0.f);
  obsToInsert.boxOrigin = pos + rotation.transformVectorNormalized(aabb.min());
  obsToInsert.invRotation = rotation.invertedNormalized();
  obsToInsert.worldMinY = minWorld.y();
  obsToInsert.worldMaxY = maxWorld.y();

  findObstacleGridCellIndices(minWorld.x(), minWorld.z(), maxWorld.x(), 
    maxWorld.z(), gridCellIndicesStorage_);

  for (const auto& gridCellIndex : gridCellIndicesStorage_) {
    auto& cell = getCell(gridCellIndex);
    BATCHED_SIM_ASSERT(cell.numObstacles < MAX_OBSTACLES_PER_CELL);
    cell.obstacles[cell.numObstacles] = obsToInsert;
    cell.numObstacles++;
  }

}

bool CollisionBroadphaseGrid::contactTest(const Magnum::Vector3& spherePos) const {

  const float sphereRadiusSq = sphereRadius_ * sphereRadius_;
  const auto& cell = getCell(findSphereGridCellIndex(spherePos.x(), spherePos.z()));

  Mn::Range3D aabb(
    Mn::Vector3(Mn::Math::ZeroInit),
    Mn::Vector3(Mn::Math::ZeroInit));

  for (int i = 0; i < cell.numObstacles; i++) {
    const auto& obs = cell.obstacles[i];

    // test y extents
    if (obs.worldMaxY < spherePos.y() - sphereRadius_
      || obs.worldMinY > spherePos.y() + sphereRadius_) {
      continue;
    }

    const auto posLocal = obs.invRotation.transformVectorNormalized(spherePos - obs.boxOrigin);

    // aabb.min is origin
    aabb.max() = obs.boxDim;

    //bool sphereBoxContactTest(const Magnum::Vector3& sphereOrigin, float sphereRadiusSq, const Magnum::Range3D& aabb);
    if (sphereBoxContactTest(posLocal, sphereRadiusSq, aabb)) {
#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG  
      debugLastContact_.obsPos = obs.boxOrigin;
      debugLastContact_.obsLocalAabb = Mn::Range3D(Mn::Vector3(Mn::Math::ZeroInit), obs.boxDim);
      debugLastContact_.obsRotation = obs.invRotation.invertedNormalized();
#endif
      return true;
    }
  }

  return false;

}




}  // namespace batched_sim
}  // namespace esp
