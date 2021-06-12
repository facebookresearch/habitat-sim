// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#ifndef ESP_GFX_PBR_DEPTHMAP_DRAWABLE_H_
#define ESP_GFX_PBR_DEPTHMAP_DRAWABLE_H_

#include "DepthMapDrawableBase.h"
#include "DepthMapShader.h"
#include "ShaderManager.h"

namespace esp {
namespace gfx {

class DepthMapDrawable : public DepthMapDrawableBase {
 public:
  /**
   * @brief Constructor, to create a DepthMapDrawable for the given object using
   * shader and mesh. Adds drawable to given group
   */
  explicit DepthMapDrawable(scene::SceneNode& node,
                            Magnum::GL::Mesh& mesh,
                            ShaderManager& shaderManager,
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
            Magnum::SceneGraph::Camera3D& camera) override;
  Magnum::Resource<Magnum::GL::AbstractShaderProgram, DepthMapShader> shader_;
};

}  // namespace gfx
}  // namespace esp

#endif
