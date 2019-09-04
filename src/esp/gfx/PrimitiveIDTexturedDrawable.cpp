// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveIDTexturedDrawable.h"
#include "PrimitiveIDTexturedShader.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

PrimitiveIDTexturedDrawable::PrimitiveIDTexturedDrawable(
    scene::SceneNode& node,
    PrimitiveIDTexturedShader& shader,
    Magnum::GL::Mesh& mesh,
    Magnum::SceneGraph::DrawableGroup3D* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */)
    : Drawable{node, shader, mesh, group}, texture_(texture) {}

void PrimitiveIDTexturedDrawable::draw(
    const Magnum::Matrix4& transformationMatrix,
    Magnum::SceneGraph::Camera3D& camera) {
  PrimitiveIDTexturedShader& shader =
      static_cast<PrimitiveIDTexturedShader&>(shader_);
  shader
      .setTransformationProjectionMatrix(camera.projectionMatrix() *
                                         transformationMatrix)
      .bindTexture(*texture_);

  mesh_.draw(shader_);
}

}  // namespace gfx
}  // namespace esp
