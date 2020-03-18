// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/SceneGraph/FeatureGroup.h>

#include "esp/core/esp.h"
#include "esp/gfx/RenderCamera.h"

namespace esp {
namespace gfx {

class RenderCamera;

/**
 * @brief Group of drawables, and shared group parameters.
 */
class DrawableGroup : public MagnumDrawableGroup {
 public:
  virtual ~DrawableGroup(){};

  /**
   * @brief Prepare to draw group with given @ref RenderCamera
   *
   * @return Whether the @ref DrawableGroup is in a valid state to be drawn
   */
  virtual bool prepareForDraw(const RenderCamera&) { return true; }

  // TODO: this should be overridden and used to compute absolute transforms of
  // shadow receivers and transformations relative to the camera in one step
  virtual std::vector<
      std::pair<std::reference_wrapper<MagnumDrawable>, Magnum::Matrix4>>
  getDrawableTransforms(RenderCamera& camera) {
    return camera.drawableTransformations(*this);
  }

  ESP_SMART_POINTERS(DrawableGroup)
};

}  // namespace gfx
}  // namespace esp
