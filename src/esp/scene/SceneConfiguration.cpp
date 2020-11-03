// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneConfiguration.h"

namespace esp {
namespace scene {

auto operator==(const SceneConfiguration& a, const SceneConfiguration& b) -> bool {
  return a.dataset == b.dataset && a.id == b.id && a.filepaths == b.filepaths &&
         a.sceneUpDir.isApprox(b.sceneUpDir) &&
         a.sceneFrontDir.isApprox(b.sceneFrontDir) &&
         std::abs(a.sceneScaleUnit - b.sceneScaleUnit) < 1e-9f;
}
auto operator!=(const SceneConfiguration& a, const SceneConfiguration& b) -> bool {
  return !(a == b);
}

}  // namespace scene
}  // namespace esp
