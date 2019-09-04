// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"
#include "esp/scene/SceneGraph.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace gfx {

class Renderer {
 public:
  Renderer();

  // draw the scene graph with the camera specified by user
  void draw(RenderCamera& camera, scene::SceneGraph& sceneGraph);

  // draw the scene graph with the visual sensor provided by user
  void draw(sensor::Sensor& visualSensor, scene::SceneGraph& sceneGraph);

  /**
   * @brief Binds a @ref RenderTarget to the sensor
   */
  void bindRenderTarget(const sensor::Sensor::ptr& sensor);

  // draw the scene graph with the default camera in scene graph
  // user needs to set the default camera so that it has correct
  // modelview matrix, projection matrix to render the scene
  // See setDefaultRenderCamera(...) in SceneGraph for more details
  // void draw(scene::SceneGraph& sceneGraph);

  ESP_SMART_POINTERS_WITH_UNIQUE_PIMPL(Renderer)
};

}  // namespace gfx
}  // namespace esp
