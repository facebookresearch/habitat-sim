// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include <Corrade/Utility/Assert.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/DebugStl.h>

#include "SceneGraph.h"

namespace esp {
namespace scene {

SceneGraph::SceneGraph()
    : rootNode_{world_},
      defaultRenderCameraNode_{rootNode_},
      defaultRenderCamera_{defaultRenderCameraNode_} {}

// set transformation, projection matrix, viewport to the default camera
void SceneGraph::setDefaultRenderCamera(sensor::Sensor& sensor) {
  ASSERT(sensor.isVisualSensor());

  sensor.setTransformationMatrix(defaultRenderCamera_)
      .setProjectionMatrix(defaultRenderCamera_)
      .setViewport(defaultRenderCamera_);
}

bool SceneGraph::isRootNode(SceneNode& node) {
  auto parent = node.parent();
  // if the parent is null, it means the node is the world_ node.
  CORRADE_ASSERT(parent != nullptr,
                 "SceneGraph::isRootNode: the node is illegal.", false);
  return (parent->parent() == nullptr ? true : false);
}

}  // namespace scene
}  // namespace esp
