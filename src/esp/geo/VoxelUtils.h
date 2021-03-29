
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GEO_VOXEL_UTILITY_H_
#define ESP_GEO_VOXEL_UTILITY_H_

#include "VoxelWrapper.h"
#include "esp/core/esp.h"
#include "esp/geo/geo.h"

namespace esp {
namespace geo {

void generateManhattanDistanceSDF(const std::string& gridName);

}
}  // namespace esp
#endif
