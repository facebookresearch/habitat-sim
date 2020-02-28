// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/SceneGraph/FeatureGroup.h>
#include <Magnum/SceneGraph/SceneGraph.h>

#include "esp/core/esp.h"

namespace esp {
namespace gfx {

class Drawable;
class RenderCamera;

/**
 * @brief Group of drawables, and shared group parameters.
 */
class DrawableGroup : public Magnum::SceneGraph::DrawableGroup3D {
 public:
  virtual ~DrawableGroup(){};

  /**
   * @brief Prepare to draw group with given @ref RenderCamera
   *
   * @return Whether the @ref DrawableGroup is in a valid state to be drawn
   */
  virtual bool prepareForDraw(const RenderCamera&) { return true; }

  ESP_SMART_POINTERS(DrawableGroup)
};

}  // namespace gfx
}  // namespace esp
