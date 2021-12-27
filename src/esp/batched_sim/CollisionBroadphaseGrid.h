// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_COLLISIONBROADPHASEGRID_H_
#define ESP_BATCHEDSIM_COLLISIONBROADPHASEGRID_H_

#include "esp/batched_sim/BatchedSimAssert.h"

#include <Corrade/Utility/Assert.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Range.h>

#include <vector>
#include <array>

#ifndef NDEBUG
#define COLLISIONBROADPHASEGRID_ENABLE_DEBUG
#endif

namespace esp {
namespace batched_sim {

class CollisionBroadphaseGrid {
 public:

#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG
  struct DebugContact {
    Magnum::Vector3 obsPos;
    Magnum::Quaternion obsRotation;
    Magnum::Range3D obsLocalAabb;
  };
#endif

  CollisionBroadphaseGrid() = default;
  CollisionBroadphaseGrid(float sphereRadius, float minX, float minZ, 
    float maxX, float maxZ, 
    int maxBytes=1024*1024, float maxGridSpacing=1.0);

  static constexpr int NUM_GRIDS = 4;

  // struct InsertionHandle {
  //   int32_t cellIndex = -1;
  //   uint8_t obstacleIndexes[NUM_GRIDS] = {255, 255, 255, 255};
  // };

  bool contactTest(const Magnum::Vector3& spherePos) const; // see sphereRadius arg to constructor

  void insertObstacle(const Magnum::Vector3& pos, const Magnum::Quaternion& rotation,
    const Magnum::Range3D& aabb);

#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG
  const DebugContact& getLastContact() const { return debugLastContact_; }
#endif

  //// private API exposed only for testing ////
 public:

  static constexpr int MAX_OBSTACLES_PER_CELL = 8;

  struct Obstacle {
    float worldMinY = 0.f;
    float worldMaxY = 0.f;
    // perf todo: consider storing index into rotations array
    Magnum::Quaternion invRotation;
    // the position of the min box corner in world space.
    // beware, this is generally not the same as the obstacle model's origin
    Magnum::Vector3 boxOrigin;
    Magnum::Vector3 boxDim;
  };

  struct Cell {
    int numObstacles = 0;
    std::array<Obstacle, MAX_OBSTACLES_PER_CELL> obstacles;
  };

  struct Grid {
    std::vector<Cell> cells;
  };

  int getCellIndex(int cellX, int cellZ) const;

  std::pair<int, int> findSphereGridCellIndex(float x, float z) const;
  
  std::pair<int, int> findPointGridCellXZ(
    int gridIndex, float x, float z) const;

  void findObstacleGridCellIndices(float queryMinX, float queryMinZ, float queryMaxX, 
    float queryMaxZ, std::vector<std::pair<int, int>>& gridCellIndices) const;

  std::pair<Magnum::Vector3, Magnum::Vector3> getUnrotatedRangeMinMax(
    const Magnum::Range3D& aabb,
    const Magnum::Quaternion& rotation);

  Cell& getCell(std::pair<int, int> gridCellIndex);
  const Cell& getCell(std::pair<int, int> gridCellIndex) const;

  // private members exposed only for testing
 public:


  float sphereRadius_ = 0.f;
  float minX_ = 0.f;
  float minZ_ = 0.f;
  float gridSpacing_ = 0.f;
  float invGridSpacing_ = 0.f;
  int dimX_ = 0;
  int dimZ_ = 0;
  std::array<Grid, NUM_GRIDS> grids_;
  std::vector<std::pair<int, int>> gridCellIndicesStorage_;

#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG
  mutable DebugContact debugLastContact_;
#endif

};

}  // namespace batched_sim
}  // namespace esp

#endif
