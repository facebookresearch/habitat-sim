// Copyright (c) Meta Platforms, Inc. All Rights Reserved
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_PLACEMENTHELPER_H_
#define ESP_BATCHEDSIM_PLACEMENTHELPER_H_

#include "esp/batched_sim/ColumnGrid.h"
#include "esp/batched_sim/CollisionBroadphaseGrid.h"
#include "esp/core/random.h"
#include "esp/batched_sim/EpisodeSet.h"
#include "esp/batched_sim/SerializeCollection.h"

namespace esp {
namespace batched_sim {

class PlacementHelper {
public:
  PlacementHelper(const ColumnGridSet& columnGridSet, const CollisionBroadphaseGrid& colGrid,
    const serialize::Collection& collection, esp::core::Random& random, int maxFailedPlacements);

  bool place(Magnum::Matrix4& heldObjMat, const FreeObject& freeObject, const Mn::Vector3* fallbackPos=nullptr) const;

private:
  const ColumnGridSet& columnGridSet_;
  const CollisionBroadphaseGrid& colGrid_;
  const serialize::Collection& collection_;
  esp::core::Random& random_;
  int maxFailedPlacements_;
};

}  // namespace batched_sim
}  // namespace esp

#endif
