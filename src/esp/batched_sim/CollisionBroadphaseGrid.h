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

  // perf todo: make smaller
  struct Obstacle {
    float worldMinY = 0.f;
    float worldMaxY = 0.f;
    Magnum::Quaternion invRotation;
    Magnum::Vector3 pos;
    const Magnum::Range3D* aabb = nullptr;
  };

  CollisionBroadphaseGrid() = default;
  // disallow copy
  CollisionBroadphaseGrid(const CollisionBroadphaseGrid& rhs) = delete;
  CollisionBroadphaseGrid(CollisionBroadphaseGrid&& rhs) = default;
  CollisionBroadphaseGrid& operator=(CollisionBroadphaseGrid&&) = default;

  CollisionBroadphaseGrid(float sphereRadius, float minX, float minZ, 
    float maxX, float maxZ, 
    int maxBytes=1024*1024, float maxGridSpacing=0.f, int numObstaclesToReserve=-1);

  static constexpr int NUM_GRIDS = 4;

  // struct InsertionHandle {
  //   int32_t cellIndex = -1;
  //   uint8_t obstacleIndexes[NUM_GRIDS] = {255, 255, 255, 255};
  // };

  int contactTest(const Magnum::Vector3& spherePos, float sphereRadius) const; // see sphereRadius arg to constructor

  // return obstacleIndex
  // note aabb is expected to be persistent (don't pass in a temporary!)
  int16_t insertObstacle(const Magnum::Vector3& pos, const Magnum::Quaternion& rotation,
    const Magnum::Range3D* aabb);

  void disableObstacle(int16_t obstacleIndex);
  bool isObstacleDisabled(int16_t obsIndex) const;
  void reinsertObstacle(int16_t obstacleIndex, const Magnum::Vector3& pos, const Magnum::Quaternion& rotation);  

  // todo: get rid of this; sharing obstacle is enough
#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG
  const DebugContact& getLastContact() const { return debugLastContact_; }
#endif
  int getNumObstacleInstances() { return numObstacleInstances_; }

  const Obstacle& getObstacle(int obsIndex) const;

  void removeAllObstacles();

  //// private API exposed only for testing ////
 public:

  static constexpr int MAX_OBSTACLES_PER_CELL = 12;


  struct Cell {
    int16_t numObstacles = 0;
    std::array<int16_t, MAX_OBSTACLES_PER_CELL> obstacles;
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

  Magnum::Range3D getWorldRange(int16_t obsIndex) const;

  Cell& getCell(std::pair<int, int> gridCellIndex);
  const Cell& getCell(std::pair<int, int> gridCellIndex) const;

  void insertRemoveObstacleHelper(int16_t obsIndex, bool remove);

  // private members exposed only for testing
 public:


  float maxSphereRadius_ = 0.f;
  float minX_ = 0.f;
  float minZ_ = 0.f;
  float gridSpacing_ = 0.f;
  float invGridSpacing_ = 0.f;
  int dimX_ = 0;
  int dimZ_ = 0;
  std::vector<Obstacle> obstacles_;
  std::array<Grid, NUM_GRIDS> grids_;
  std::vector<std::pair<int, int>> gridCellIndicesStorage_;
  int numObstacleInstances_ = 0;

#ifdef COLLISIONBROADPHASEGRID_ENABLE_DEBUG
  mutable DebugContact debugLastContact_;
#endif

};

}  // namespace batched_sim
}  // namespace esp

#endif
