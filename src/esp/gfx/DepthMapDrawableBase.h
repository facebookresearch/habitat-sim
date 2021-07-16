// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#ifndef ESP_GFX_PBR_DEPTHMAP_DRAWABLE_BASE_H_
#define ESP_GFX_PBR_DEPTHMAP_DRAWABLE_BASE_H_

#include "Drawable.h"
#include "ShaderManager.h"

namespace esp {
namespace gfx {

class DepthMapDrawableBase : public Drawable {
 public:
  /**
   * @brief Constructor, to create a DepthMapDrawableBase for the given object
   * using shader and mesh. Adds drawable to given group
   */
  explicit DepthMapDrawableBase(scene::SceneNode& node,
                                Magnum::GL::Mesh* mesh,
                                ShaderManager& shaderManager,
                                DrawableType type,
                                DrawableGroup* group);

 protected:
  /**
   * @brief Draw the object using given camera
   *
   * @param transformationMatrix  Transformation relative to camera.
   * @param camera                Camera to draw from.
   *
   */
  void draw(const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& camera) override = 0;
  ShaderManager& shaderManager_;
};

}  // namespace gfx
}  // namespace esp

#endif
