// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PTexMeshDrawable.h"

#include <Magnum/GL/Renderer.h>

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

  //XXX
  // ASSERT(Magnum::GL::Renderer::Error::NoError == Magnum::GL::Renderer::error());
  switch (Magnum::GL::Renderer::error()) {
    case Magnum::GL::Renderer::Error::NoError:
      LOG(INFO) << "No error";
      break;
    case Magnum::GL::Renderer::Error::InvalidValue:
      LOG(INFO) << "InvalidValue";
      break;
    case Magnum::GL::Renderer::Error::InvalidOperation:
      LOG(INFO) << "InvalidOperation";
      break;
    default:
      LOG(INFO) << "some other errors";
      break;
  }
}

}  // namespace gfx
}  // namespace esp
