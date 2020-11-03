// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneManager.h"
#include "esp/core/esp.h"

namespace esp {
namespace scene {

auto SceneManager::initSceneGraph() -> int {
  sceneGraphs_.emplace_back(std::make_unique<SceneGraph>());
  int index = sceneGraphs_.size() - 1;
  return index;
}

auto SceneManager::getSceneGraph(int sceneID) -> SceneGraph& {
  ASSERT(sceneID >= 0 && sceneID < sceneGraphs_.size());
  return (*(sceneGraphs_[sceneID].get()));
}

auto SceneManager::getSceneGraph(int sceneID) const -> const SceneGraph& {
  ASSERT(sceneID >= 0 && sceneID < sceneGraphs_.size());
  return (*(sceneGraphs_[sceneID].get()));
}

}  // namespace scene
}  // namespace esp
