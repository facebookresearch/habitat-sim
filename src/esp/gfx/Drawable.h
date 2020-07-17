// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "esp/core/esp.h"
#include "magnum.h"

namespace esp {
namespace scene {
class SceneNode;
}
namespace gfx {

class DrawableGroup;

/**
 * @brief Drawable for use with @ref DrawableGroup.
 *
 * Drawable will retrieve its shader from its group, and draw
 * itself with the shader.
 */
class Drawable : public Magnum::SceneGraph::Drawable3D {
 public:
  /**
   * @brief Constructor
   *
   * @param node Node which will be made drawable.
   * @param mesh Mesh to draw when on render.
   * @param group Drawable group this drawable will be added to.
   */
  Drawable(scene::SceneNode& node,
           Magnum::GL::Mesh& mesh,
           DrawableGroup* group = nullptr);
  virtual ~Drawable();

  virtual scene::SceneNode& getSceneNode() { return node_; }

  /**
   * @brief Get the @ref DrawableGroup this drawable is in.
   *
   * This overrides Magnum::SceneGraph::Drawable so that the derived @ref
   * DrawableGroup can be used
   */
  DrawableGroup* drawables();

  /**
   * @brief Get the drawable id
   */
  uint64_t getDrawableId() { return drawableId_; }

  /**
   * @brief Return if this drawable object is selected in the editor (e.g.,
   * viewer)
   * @return the drawable object state, if it is selected
   */
  bool isSelected() { return selected_; }

  /**
   *  @brief Set the state of the drawable, whether it is selected in the editor
   * (e.g., viewer)
   *  @param selected true if selected, otherwise false
   */
  void setSelected(bool selected) { selected_ = selected; }

  virtual void setLightSetup(const Magnum::ResourceKey& lightSetup){};

 protected:
  /**
   * @brief Draw the object using given camera
   *
   * @param transformationMatrix  Transformation relative to camera.
   * @param camera                Camera to draw from.
   *
   * Each derived drawable class needs to implement this draw() function.
   * It's nothing more than drawing itself with its group's shader.
   */
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) = 0;

  static uint64_t drawableIdCounter;
  uint64_t drawableId_;
  bool selected_ = false;

  scene::SceneNode& node_;
  Magnum::GL::Mesh& mesh_;
};

}  // namespace gfx
}  // namespace esp
