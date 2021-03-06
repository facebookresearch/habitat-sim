// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SCENE_SCENEMANAGER_H_
#define ESP_SCENE_SCENEMANAGER_H_

#include <memory>
#include <vector>

#include "esp/core/esp.h"
#include "esp/gfx/magnum.h"

#include "SceneGraph.h"
#include "SceneNode.h"

namespace esp {
namespace scene {

// TODO:
// make SceneManager a singleton class
class SceneManager {
 public:
  SceneManager() = default;
  ~SceneManager() { LOG(INFO) << "Deconstructing SceneManager"; }

  // returns the scene ID
  int initSceneGraph();

  // returns the scene graph
  SceneGraph& getSceneGraph(int sceneID);
  const SceneGraph& getSceneGraph(int sceneID) const;

 protected:
  // Each item within is a base node, parent of all in that scene, for easy
  // manipulation (e.g., rotate the entire scene)
  std::vector<std::unique_ptr<SceneGraph>> sceneGraphs_;

  ESP_SMART_POINTERS(SceneManager)
};
}  // namespace scene
}  // namespace esp

#endif  // ESP_SCENE_SCENEMANAGER_H_
