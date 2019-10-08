// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include "Drawable.h"
#include "PTexMeshShader.h"

namespace esp {
namespace assets {
class PTexMeshData;
}
namespace gfx {

class PTexMeshShader;

class PTexMeshDrawable : public Drawable {
 public:
  explicit PTexMeshDrawable(
      scene::SceneNode& node,
      PTexMeshShader& shader,
      assets::PTexMeshData& ptexMeshData,
      int submeshID,
      Magnum::SceneGraph::DrawableGroup3D* group = nullptr);

 protected:
  virtual void draw(const Magnum::Matrix4& transformationMatrix,
                    Magnum::SceneGraph::Camera3D& camera) override;

  Magnum::GL::Texture2D& atlasTexture_;
#ifndef CORRADE_TARGET_APPLE
  Magnum::GL::BufferTexture& adjFacesBufferTexture_;
#endif
  uint32_t tileSize_;
  float exposure_;
  float gamma_;
  float saturation_;
};

}  // namespace gfx
}  // namespace esp
