// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "SceneManager.h"
#include "esp/core/Esp.h"

namespace esp {
namespace scene {

int SceneManager::initSceneGraph() {
  sceneGraphs_.emplace_back(std::make_unique<SceneGraph>());
  int index = sceneGraphs_.size() - 1;
  return index;
}

SceneGraph& SceneManager::getSceneGraph(int sceneID) {
  CORRADE_INTERNAL_ASSERT(sceneID >= 0 && sceneID < sceneGraphs_.size());
  return (*(sceneGraphs_[sceneID].get()));
}

const SceneGraph& SceneManager::getSceneGraph(int sceneID) const {
  CORRADE_INTERNAL_ASSERT(sceneID >= 0 && sceneID < sceneGraphs_.size());
  return (*(sceneGraphs_[sceneID].get()));
}

}  // namespace scene
}  // namespace esp
