// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PTexMeshDrawable.h"

#include "esp/assets/PTexMeshData.h"
#include "esp/gfx/PTexMeshShader.h"

namespace esp {
namespace gfx {

// static constexpr arrays require redundant definitions until C++17
constexpr char PTexMeshDrawable::SHADER_KEY[];

PTexMeshDrawable::PTexMeshDrawable(scene::SceneNode& node,
                                   assets::PTexMeshData& ptexMeshData,
                                   int submeshID,
                                   ShaderManager& shaderManager,
                                   DrawableGroup* group /* = nullptr */)
    : Drawable{node, ptexMeshData.getRenderingBuffer(submeshID)->mesh,
               DrawableType::PTexMesh, group},
      atlasTexture_(ptexMeshData.getRenderingBuffer(submeshID)->atlasTexture),
#ifndef CORRADE_TARGET_APPLE
      adjFacesBufferTexture_(
          ptexMeshData.getRenderingBuffer(submeshID)->adjFacesBufferTexture),
#endif
      tileSize_(ptexMeshData.tileSize()),
      exposure_(ptexMeshData.exposure()),
      gamma_(ptexMeshData.gamma()),
      saturation_(ptexMeshData.saturation()),
      visualizerTriangleMesh_(
          ptexMeshData.getRenderingBuffer(submeshID)->triangleMesh) {
  auto shaderResource =
      shaderManager.get<Magnum::GL::AbstractShaderProgram, PTexMeshShader>(
          SHADER_KEY);

  if (!shaderResource) {
    shaderManager.set<Magnum::GL::AbstractShaderProgram>(shaderResource.key(),
                                                         new PTexMeshShader{});
  }
  shader_ = &(*shaderResource);
}

void PTexMeshDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                            Magnum::SceneGraph::Camera3D& camera) {
  (*shader_)
      .setExposure(exposure_)
      .setGamma(gamma_)
      .setSaturation(saturation_)
      .setAtlasTextureSize(atlasTexture_, tileSize_)
      .bindAtlasTexture(atlasTexture_)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(static_cast<RenderCamera&>(camera).useDrawableIds()
                       ? drawableId_
                       : node_.getSemanticId())
#ifndef CORRADE_TARGET_APPLE
      .bindAdjFacesBufferTexture(adjFacesBufferTexture_)
#endif
      .setMVPMatrix(camera.projectionMatrix() * transformationMatrix)
      .draw(mesh_);
}

}  // namespace gfx
}  // namespace esp
