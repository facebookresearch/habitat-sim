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
      atlasTexture_(ptexMeshData.getRenderingBuffer(submeshID)->atlasTexture),
#ifndef CORRADE_TARGET_APPLE
      adjFacesBufferTexture_(
          ptexMeshData.getRenderingBuffer(submeshID)->adjFacesBufferTexture),
#endif
      tileSize_(ptexMeshData.tileSize()),
      exposure_(ptexMeshData.exposure()),
      gamma_(ptexMeshData.gamma()),
      saturation_(ptexMeshData.saturation()) {
}

void PTexMeshDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                            Magnum::SceneGraph::Camera3D& camera) {
  PTexMeshShader& ptexMeshShader = static_cast<PTexMeshShader&>(shader_);
  ptexMeshShader.setExposure(exposure_)
      .setGamma(gamma_)
      .setSaturation(saturation_)
      .setAtlasTextureSize(atlasTexture_, tileSize_)
      .bindAtlasTexture(atlasTexture_)
#ifndef CORRADE_TARGET_APPLE
      .bindAdjFacesBufferTexture(adjFacesBufferTexture_)
#endif
      .setMVPMatrix(camera.projectionMatrix() * transformationMatrix);
  mesh_.draw(ptexMeshShader);
}

}  // namespace gfx
}  // namespace esp
