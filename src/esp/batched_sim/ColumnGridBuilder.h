// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BATCHEDSIM_COLUMNGRIDBUILDER_H_
#define ESP_BATCHEDSIM_COLUMNGRIDBUILDER_H_

#include "ColumnGrid.h"

#include <Magnum/Magnum.h>
#include <Corrade/Utility/Assert.h>

#include <vector>
#include <limits>

namespace esp {
namespace sim {
class Simulator;
}

namespace batched_sim {

class ColumnGridBuilder {

 public:
  ColumnGridSource build(esp::sim::Simulator& sim, const Magnum::Range3D& aabb, 
    float sphereRadius, float gridSpacing);
};

}  // namespace batched_sim
}  // namespace esp

#endif
