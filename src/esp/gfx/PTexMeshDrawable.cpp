// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PTexMeshDrawable.h"

#include "esp/assets/PTexMeshData.h"

namespace esp {
namespace gfx {

PTexMeshDrawable::PTexMeshDrawable(
    scene::SceneNode& node,
    PTexMeshShader& shader,
    assets::PTexMeshData& ptexMeshData,
    int submeshID,
    Magnum::SceneGraph::DrawableGroup3D* group /* = nullptr */)
    : Drawable{node, shader, ptexMeshData.getRenderingBuffer(submeshID)->mesh,
               group},
      tex_(ptexMeshData.getRenderingBuffer(submeshID)->tex),
      adjTex_(ptexMeshData.getRenderingBuffer(submeshID)->adjTex),
      tileSize_(ptexMeshData.tileSize()),
      exposure_(ptexMeshData.exposure()) {}

void PTexMeshDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                            Magnum::SceneGraph::Camera3D& camera) {
  adjTex_.bind(1);
  PTexMeshShader& ptexMeshShader = static_cast<PTexMeshShader&>(shader_);
  ptexMeshShader.bindTexture(tex_, 0)
      .setPTexUniforms(tex_, tileSize_, exposure_)
      .setMVPMatrix(camera.projectionMatrix() * transformationMatrix);
  mesh_.draw(ptexMeshShader);
}

}  // namespace gfx
}  // namespace esp
