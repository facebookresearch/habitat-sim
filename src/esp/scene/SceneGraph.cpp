// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include <Magnum/Math/Algorithms/GramSchmidt.h>
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

  sensor.setModelViewMatrix(defaultRenderCamera_)
      .setProjectionMatrix(defaultRenderCamera_)
      .setViewport(defaultRenderCamera_);
}

}  // namespace scene
}  // namespace esp
