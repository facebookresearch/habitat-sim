// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SIM_RENDERINSTANCEHELPER_H_
#define ESP_SIM_RENDERINSTANCEHELPER_H_

#include <string>
#include <vector>

#include "esp/core/Esp.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace sim {
class Simulator;
}
}  // namespace esp

namespace esp {
namespace sim {

class RenderInstanceHelper {
 public:
  RenderInstanceHelper(Simulator& sim, bool useXYZWOrientations = true);

  int AddInstance(const std::string& assetFilepath, int semanticId);
  void ClearAllInstances();
  int GetNumInstances();
  void SetWorldPoses(float* positions,
                     size_t positionsSize,
                     float* orientations,
                     size_t orientationsSize);

 private:
  Simulator* sim_ = nullptr;
  std::vector<scene::SceneNode*> instances_;
  bool isXYZW_ = false;

  ESP_SMART_POINTERS(RenderInstanceHelper)
};

}  // namespace sim
}  // namespace esp

#endif
