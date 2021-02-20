// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_GFX_PTEXMESHDRAWABLE_H_
#define ESP_GFX_PTEXMESHDRAWABLE_H_

#include "Drawable.h"
#include "esp/gfx/ShaderManager.h"

namespace esp {
namespace assets {
class PTexMeshData;
}
namespace gfx {

class PTexMeshShader;

class PTexMeshDrawable : public Drawable {
 public:
  explicit PTexMeshDrawable(scene::SceneNode& node,
                            assets::PTexMeshData& ptexMeshData,
                            int submeshID,
                            ShaderManager& shaderManager,
                            DrawableGroup* group = nullptr);

  static constexpr char SHADER_KEY[] = "PTexMeshShader";
  Magnum::GL::Mesh& getVisualizerMesh() override {
    return visualizerTriangleMesh_;
  }

 protected:
  void draw(const Magnum::Matrix4& transformationMatrix,
            Magnum::SceneGraph::Camera3D& camera) override;

  Magnum::GL::Texture2D& atlasTexture_;
#ifndef CORRADE_TARGET_APPLE
  Magnum::GL::BufferTexture& adjFacesBufferTexture_;
#endif
  uint32_t tileSize_;
  float exposure_;
  float gamma_;
  float saturation_;
  Magnum::GL::Mesh& visualizerTriangleMesh_;
  PTexMeshShader* shader_ = nullptr;
};

}  // namespace gfx
}  // namespace esp

#endif  // ESP_GFX_PTEXMESHDRAWABLE_H_
