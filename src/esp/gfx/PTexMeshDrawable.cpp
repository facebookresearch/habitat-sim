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
      adjFaces_(ptexMeshData.getRenderingBuffer(submeshID)->adjFaces),
      tileSize_(ptexMeshData.tileSize()),
      exposure_(ptexMeshData.exposure()),
      gamma_(ptexMeshData.gamma()),
      saturation_(ptexMeshData.saturation()) {
  PTexMeshShader& ptexMeshShader = static_cast<PTexMeshShader&>(shader_);
  // clipPlane is const for every ptex mesh in this version.
  // Set it in the constructor, not in the draw()
  ptexMeshShader.setClipPlane(clipPlane_);
}

void PTexMeshDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                            Magnum::SceneGraph::Camera3D& camera) {
  PTexMeshShader& ptexMeshShader = static_cast<PTexMeshShader&>(shader_);
  ptexMeshShader.setExposure(exposure_)
      .setGamma(gamma_)
      .setSaturation(saturation_)
      .setAtlasTextureSize(tex_, tileSize_)
      .bindAtlasTexture(tex_)
      .bindAdjFacesBufferTexture(adjFaces_)
      .setMVPMatrix(camera.projectionMatrix() * transformationMatrix);
  mesh_.draw(ptexMeshShader);
}

}  // namespace gfx
}  // namespace esp
