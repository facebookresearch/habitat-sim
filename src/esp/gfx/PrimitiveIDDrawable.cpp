// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveIDDrawable.h"
#include "PrimitiveIDShader.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

PrimitiveIDDrawable::PrimitiveIDDrawable(scene::SceneNode& node,
                                         PrimitiveIDShader& shader,
                                         Magnum::GL::Mesh& mesh,
                                         DrawableGroup* group /* = nullptr */)
    : Drawable{node, shader, mesh, group} {}

void PrimitiveIDDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                               Magnum::SceneGraph::Camera3D& camera) {
  PrimitiveIDShader& shader = static_cast<PrimitiveIDShader&>(shader_);
  shader.setTransformationProjectionMatrix(camera.projectionMatrix() *
                                           transformationMatrix);

  mesh_.draw(shader_);
}

}  // namespace gfx
}  // namespace esp
