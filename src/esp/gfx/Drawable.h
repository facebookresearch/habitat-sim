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

  scene::SceneNode& node_;
  Magnum::GL::Mesh& mesh_;

  /**
   * Why a friend class?
   * because DrawableGroup will have to update the state, attachedToGroup
   * directly, and ONLY this class can do it
   */
  friend class DrawableGroup;

  // system needs to make sure a Drawable is attaching to
  // esp::gfx::DrawableGroup (a sub-class of magnum base class),
  // not the magnum base drawable group
  bool attachedToGroup_ = false;
};

}  // namespace gfx
}  // namespace esp
