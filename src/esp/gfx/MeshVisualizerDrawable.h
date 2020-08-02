// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once
#include <Magnum/Shaders/MeshVisualizer.h>
#include "Drawable.h"
#include "esp/gfx/Drawable.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace gfx {

class MeshVisualizerDrawable : public Drawable {
 public:
  /**
   * @brief Constructor
   *
   * @param node   Node, to which the drawable is attached
   * @param shader Shader for the mesh visualizer
   * @param mesh   Mesh to draw when on render.
   * @param group  Drawable group this drawable will be added to.
   */
  explicit MeshVisualizerDrawable(scene::SceneNode& node,
                                  Magnum::Shaders::MeshVisualizer3D& shader,
                                  Magnum::GL::Mesh& mesh,
                                  gfx::DrawableGroup* group);

 protected:
  /**
   * @brief Draw the object using given camera
   *
   * @param transformationMatrix  Transformation relative to camera.
   * @param camera                Camera to draw from.
   *
   */
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;
  Magnum::Shaders::MeshVisualizer3D& shader_;
};

}  // namespace gfx
}  // namespace esp
