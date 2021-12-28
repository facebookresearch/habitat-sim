// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_PLACEMENTHELPER_H_
#define ESP_BATCHEDSIM_PLACEMENTHELPER_H_

#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/core/random.h"
#include "esp/batched_sim/EpisodeSet.h"

namespace esp {
namespace batched_sim {

class PlacementHelper {
public:
  PlacementHelper(const ColumnGridSource& columnGrid, const CollisionBroadphaseGrid& colGrid,
    esp::core::Random& random, int maxFailedPlacements);

  bool place(Magnum::Matrix4& heldObjMat, const FreeObject& freeObject) const;

private:
  const ColumnGridSource& columnGrid_;
  const CollisionBroadphaseGrid& colGrid_;
  esp::core::Random& random_;
  int maxFailedPlacements_;
};

}  // namespace batched_sim
}  // namespace esp

#endif
