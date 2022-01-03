// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_COLUMNGRID_H_
#define ESP_BATCHEDSIM_COLUMNGRID_H_

#include "esp/batched_sim/BatchedSimAssert.h"

#include <Corrade/Utility/Assert.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include <vector>
#include <limits>

namespace esp {
namespace batched_sim {

class ColumnGridSource {
 public:

  // disallow copy
  ColumnGridSource(const ColumnGridSource& rhs) = delete;
  ColumnGridSource(ColumnGridSource&& rhs) = default;
  ColumnGridSource& operator=(ColumnGridSource&&) = default;

  // todo: make this a struct so it can get initialized to 0 or whatever
  using QueryCacheValue = int8_t;

  // This is generally used as a special invalid value, but we also require it to be
  // large and positive. todo: rename to "LARGE_Y" or something.
  static constexpr float INVALID_Y = std::numeric_limits<float>::max();
  struct Column {
    float freeMinY = INVALID_Y;
    float freeMaxY = INVALID_Y;

    bool operator==(const Column& rhs) const {
      return freeMinY == rhs.freeMinY && freeMaxY == rhs.freeMaxY;
    }
  };
  struct Layer {
    // size is dim.x * dim.y
    std::vector<Column> columns;

    bool operator==(const Layer& rhs) const {
      return columns == rhs.columns;
    }
  };

  float getMaxX() const {
    return minX + dimX * gridSpacing;
  }
  float getMaxZ() const {
    return minZ + dimZ * gridSpacing;
  }

  struct Patch {
    int localCellShift = 0;
    int numLayers = 0;
  };

  float minX = 0.f;
  float minZ = 0.f;
  float gridSpacing = 0.f;
  float invGridSpacing = 0.f;
  float sphereRadius = 0.f;
  int dimX = 0;
  int dimZ = 0;
  int patchShift = 31; // temp shift all bits away; only one patch
  std::vector<Patch> patches;
  std::vector<Layer> layers;

  ColumnGridSource() {
    patches.push_back(Patch());
  }

  void load(const std::string& filepath);
  void save(const std::string& filepath);

  bool contactTest(const Magnum::Vector3& pos,
    ColumnGridSource::QueryCacheValue* queryCache) const;

  // returns distance down to contact (or up to contact-free)
  // positive indicates contact-free; negative indicates distance up to be contact-free
  float castDownTest(const Magnum::Vector3& pos,
    ColumnGridSource::QueryCacheValue* queryCache) const;

  void ensureLayer(int layerIdx) {
    // sanity-check: this data structure shouldn't have too many layers
    BATCHED_SIM_ASSERT(layerIdx < 24);
    if (layers.size() < layerIdx + 1) {
      // only increase capacity minimally
      layers.reserve(layerIdx + 1);
      const int oldSize = layers.size();
      layers.resize(layerIdx + 1);
      // set column array to correct size
      for (int i = oldSize; i < layers.size(); i++) {
        layers[i].columns.resize(dimX * dimZ);
      }
      BATCHED_SIM_ASSERT(patches.size() == 1);
      patches[0].numLayers = layers.size();
    }
  }

  Column debugGetColumn(int cellX, int cellZ, int layerIdx) const {
    // temp: just index based on entire grid dim
    BATCHED_SIM_ASSERT(patches.size() == 1);
    int cellIdx = getCellIndex(cellX, cellZ);
    if (layerIdx >= layers.size()) {
      return Column{INVALID_Y, INVALID_Y};
    }
    return layers[layerIdx].columns[cellIdx];
  }

  void appendColumn(int cellX, int cellZ, float freeMinY, float freeMaxY) {
    BATCHED_SIM_ASSERT(freeMaxY > freeMinY);
    int cellIdx = getCellIndex(cellX, cellZ);
    int layerIdx = 0;
    // search for a layer that has an empty/invalid column at this cell
    while (true) {
      ensureLayer(layerIdx);
      auto& col = layers[layerIdx].columns[cellIdx];
      if (col.freeMinY == INVALID_Y) {
        // we found an empty/invalid column
        col.freeMinY = freeMinY;
        col.freeMaxY = freeMaxY;
        break;
      } else {
        BATCHED_SIM_ASSERT(col.freeMinY != INVALID_Y);
        // new column should be higher than existing columns
        BATCHED_SIM_ASSERT(freeMinY > col.freeMaxY);
        layerIdx++;
      }
    }
  }

 public: // temp everything public
  int getCellIndex(int cellX, int cellZ) const {
    BATCHED_SIM_ASSERT(cellX >= 0 && cellX < dimX);
    BATCHED_SIM_ASSERT(cellZ >= 0 && cellZ < dimZ);
    return cellZ * dimX + cellX; // todo: more clever memory layout
  }

  int getLocalCellIndex(int localCellX, int localCellZ) const  {
    // temp: just index based on entire grid dim
    BATCHED_SIM_ASSERT(patches.size() == 1);
    return getCellIndex(localCellX, localCellZ);
  }

  int getPatchIndex(int patchX, int patchZ) const {
    BATCHED_SIM_ASSERT(patchX == 0 && patchZ == 0);
    return 0;
  }

  // todo: use an explicit integer type for all these indices

  // avoid bit-manipulation confusion by avoiding very large integers
  static constexpr int MAX_INTEGER_MATH_COORD = (1 << 30) - 1;

  static constexpr int globalToLocalCellMask = 0xFFFFFFFF;

  const Column& getColumn(const Patch& patch, int localCellIdx, int layerIndex) const {
    // temp: just index based on entire grid dim
    BATCHED_SIM_ASSERT(patches.size() == 1);
    BATCHED_SIM_ASSERT(layerIndex >= 0 && layerIndex < layers.size());
    BATCHED_SIM_ASSERT(localCellIdx >= 0 && localCellIdx < layers[layerIndex].columns.size());
    return layers[layerIndex].columns[localCellIdx];
  }
};

class ColumnGridSet {
 public:

  ColumnGridSet() = default;
  // disallow copy
  ColumnGridSet(const ColumnGridSet& rhs) = delete;
  ColumnGridSet(ColumnGridSet&& rhs) = default;
  ColumnGridSet& operator=(ColumnGridSet&&) = default;

  void load(const std::string& filepathBase);

  const std::vector<float>& getSphereRadii() const { return sphereRadii_; }
  const ColumnGridSource& getColumnGrid(int radiusIdx) const;

  bool contactTest(int radiusIdx, const Magnum::Vector3& pos,
    ColumnGridSource::QueryCacheValue* queryCache) const;

  // returns distance down to contact (or up to contact-free)
  // positive indicates contact-free; negative indicates distance up to be contact-free
  float castDownTest(int radiusIdx, const Magnum::Vector3& pos,
    ColumnGridSource::QueryCacheValue* queryCache) const;
 private:
  std::vector<ColumnGridSource> columnGrids_;
  std::vector<float> sphereRadii_;
};

}  // namespace batched_sim
}  // namespace esp

#endif
