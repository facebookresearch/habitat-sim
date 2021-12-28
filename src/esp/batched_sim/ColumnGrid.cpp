// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "ColumnGrid.h"
#include "esp/batched_sim/BatchedSimAssert.h"
#include "esp/core/logging.h"

#include <Corrade/Utility/Assert.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

#include <cstdio>

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace batched_sim {

namespace {

static constexpr int CURRENT_FILE_VERSION = 1;
static constexpr uint64_t MAGIC = 0xFACEB00501234567;

struct FileHeader {
  uint64_t magic = MAGIC;
  uint32_t version = CURRENT_FILE_VERSION;
  float minX = 0.f;
  float minZ = 0.f;
  float gridSpacing = 0.f;
  float sphereRadius = 0.f;
  uint32_t dimX = 0;
  uint32_t dimZ = 0;
  uint32_t numLayers = 0;
};

}

bool ColumnGridSource::contactTest(const Mn::Vector3& pos,
  ColumnGridSource::QueryCacheValue* queryCache) const {

  return castDownTest(pos, queryCache) < 0.f;
}

float ColumnGridSource::castDownTest(const Mn::Vector3& pos,
  ColumnGridSource::QueryCacheValue* queryCache) const {

  // todo: think about this more (for objects vs interiors)
  constexpr float outsideVolumeRetVal = -INVALID_Y;

  const float cellFloatX = (pos.x() - minX) * invGridSpacing;
  if (cellFloatX < 0.f || cellFloatX >= (float)dimX) { // perf todo: pre-cache float dimX
    return outsideVolumeRetVal;
  }

  const float cellFloatZ = (pos.z() - minZ) * invGridSpacing;
  if (cellFloatZ < 0.f || cellFloatZ >= (float)dimZ) {
    return outsideVolumeRetVal;
  }

  // todo: compare y bounds and early out

  float queryY = pos.y();

  // consider allowing a margin and then clamping, to account for numeric imprecision
  BATCHED_SIM_ASSERT(cellFloatX >= 0.f && cellFloatX < (float)dimX);
  BATCHED_SIM_ASSERT(cellFloatX < (float)MAX_INTEGER_MATH_COORD);
  const int globalCellX = int(cellFloatX); // note truncation, not founding
  BATCHED_SIM_ASSERT(globalCellX >= 0 && globalCellX <= MAX_INTEGER_MATH_COORD);

  BATCHED_SIM_ASSERT(cellFloatZ >= 0.f && cellFloatZ < (float)dimZ);
  const int globalCellZ = int(cellFloatZ);
  BATCHED_SIM_ASSERT(globalCellZ >= 0 && globalCellZ <= MAX_INTEGER_MATH_COORD);

  const int patchX = globalCellX >> patchShift;
  const int patchZ = globalCellZ >> patchShift;
  const int patchIdx = getPatchIndex(patchX, patchZ);

  const Patch& patch = patches[patchIdx];
  if (patch.numLayers == 0) {
    return outsideVolumeRetVal;
  }

  const int localCellX = (globalCellX & globalToLocalCellMask) >> patch.localCellShift;
  const int localCellZ = (globalCellZ & globalToLocalCellMask) >> patch.localCellShift;

  const int localCellIdx = getLocalCellIndex(localCellX, localCellZ);

  int layerIndex = Mn::Math::clamp(int(*queryCache), 0, patch.numLayers - 1);
  const auto& col = getColumn(patch, localCellIdx, layerIndex);

  if (queryY >= col.freeMinY) {
    if (queryY <= col.freeMaxY) {
      *queryCache = layerIndex;
      return queryY - col.freeMinY;
    }
    // search up
    while (true) {
      layerIndex++;
      if (layerIndex >= patch.numLayers) {
        *queryCache = patch.numLayers - 1;
        return outsideVolumeRetVal;
      }
      const auto& col = getColumn(patch, localCellIdx, layerIndex);
      if (queryY < col.freeMinY) {
        *queryCache = layerIndex;
        return queryY - col.freeMinY; // in between two free columns
      }
      if (queryY <= col.freeMaxY) {
        *queryCache = layerIndex;
        return queryY - col.freeMinY;
      }
    }
  } else {
    // search down
    float prevColFreeMinY = col.freeMinY;
    while (true) {
      layerIndex--;
      if (layerIndex < 0) {
        *queryCache = 0;
        return outsideVolumeRetVal;
      }
      const auto& col = getColumn(patch, localCellIdx, layerIndex);
      if (queryY > col.freeMaxY) { // in between two free columns
        *queryCache = layerIndex;
        BATCHED_SIM_ASSERT(queryY < prevColFreeMinY);
        return queryY - prevColFreeMinY;
      }
      if (queryY >= col.freeMinY) {
        *queryCache = layerIndex;
        return queryY - col.freeMinY;
      }
      prevColFreeMinY = col.freeMinY;
    }
  }

}

void ColumnGridSource::load(const std::string& filepath) {

  FILE* fp = fopen(filepath.c_str(), "rb");
  if (!fp) {
    ESP_ERROR() << "unable to open file for loading at " << filepath;
    return;
  }

  FileHeader header{};
  size_t readLen = fread(&header, sizeof(FileHeader), 1, fp);
  if (readLen != 1 || header.magic != MAGIC) {
    fclose(fp);
    ESP_ERROR() << "file read error for " << filepath;
    return;
  }
  if (header.version != CURRENT_FILE_VERSION) {
    fclose(fp);
    ESP_ERROR() << "on-disk version is " << header.version 
      << " instead of current version " << CURRENT_FILE_VERSION;
    return;
  }

  dimX = header.dimX;
  dimZ = header.dimZ;
  gridSpacing = header.gridSpacing;
  minX = header.minX;
  minZ = header.minZ;
  sphereRadius = header.sphereRadius;

  BATCHED_SIM_ASSERT(dimX > 0);
  BATCHED_SIM_ASSERT(dimZ > 0);
  BATCHED_SIM_ASSERT(sphereRadius > 0.f);
  BATCHED_SIM_ASSERT(gridSpacing > 0.f);
  invGridSpacing = 1.f / gridSpacing;
  ensureLayer(header.numLayers - 1);

  for (int i = 0; i < layers.size(); i++) {
    size_t readLen = fread(layers[i].columns.data(), sizeof(Column), dimX * dimZ, fp);
    if (readLen != dimX * dimZ) {
      ESP_ERROR() << "file read error for " << filepath;
      return;
    }
  }

  fclose(fp);
}

void ColumnGridSource::save(const std::string& filepath) {

  FILE* fp = fopen(filepath.c_str(), "wb");
  if (!fp) {
    ESP_ERROR() << "unable to open file for saving at " << filepath;
    return;
  }

  FileHeader header{};
  header.dimX = dimX;
  header.dimZ = dimZ;
  header.gridSpacing = gridSpacing;
  header.minX = minX;
  header.minZ = minZ;
  header.sphereRadius = sphereRadius;
  header.numLayers = layers.size();

  fwrite(&header, sizeof(FileHeader), 1, fp);

  for (int i = 0; i < layers.size(); i++) {
    fwrite(layers[i].columns.data(), sizeof(Column), dimX * dimZ, fp);
  }

  fclose(fp);
}

}  // namespace batched_sim
}  // namespace esp
