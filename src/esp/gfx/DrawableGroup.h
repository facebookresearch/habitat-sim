// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Magnum/SceneGraph/SceneGraph.h>

#include "esp/core/esp.h"

// #include "esp/gfx/Shader.h"

namespace esp {
namespace gfx {

class Drawable;
class RenderCamera;

// STUB (todo: remove)
class Shader {
 public:
  bool prepareForDraw(const DrawableGroup& drawables,
                      const RenderCamera& camera) {
    return false;
  }
  ESP_SMART_POINTERS(Shader);
};

/**
 * @brief Group of drawables, and shared group parameters.
 */
class DrawableGroup : public Magnum::SceneGraph::DrawableGroup3D {
 public:
  /**
   * @brief Constructor
   *
   * @param shader Shader all @ref Drawables in this group will use.
   */
  explicit DrawableGroup(Shader::ptr shader = nullptr)
      : Magnum::SceneGraph::DrawableGroup3D{}, shader_{std::move(shader)} {}

  /**
   * @brief Add a drawable to this group.
   *
   * @return Reference to self (for method chaining)
   *
   * If the drawable is part of another group, it is removed from it.
   */
  DrawableGroup& add(Drawable& drawable);

  /**
   * @brief Removes the drawable from this group.
   *
   * @return Reference to self (for method chaining)
   *
   * The drawable must be part of this group.
   */
  DrawableGroup& remove(Drawable& drawable);

  /**
   * @brief Get the shader this group is using.
   */
  Shader::ptr getShader() { return shader_; }

  /**
   * @brief Set the shader this group uses.
   * @return Reference to self (for method chaining)
   */
  DrawableGroup& setShader(Shader::ptr shader) {
    shader_ = std::move(shader);
    return *this;
  }

  /**
   * @brief Prepare to draw group with given @ref RenderCamera
   *
   * @return Whether the @ref DrawableGroup is in a valid state to be drawn
   */
  bool prepareForDraw(const RenderCamera& camera);

 private:
  Shader::ptr shader_ = nullptr;

  ESP_SMART_POINTERS(DrawableGroup);
};

}  // namespace gfx
}  // namespace esp
