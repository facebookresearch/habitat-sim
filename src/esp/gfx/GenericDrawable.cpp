// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Magnum/Shaders/Flat.h>

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(
    scene::SceneNode& node,
    Magnum::Shaders::Flat3D& shader,
    Magnum::GL::Mesh& mesh,
    DrawableGroup* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    const Magnum::Color4& color /* = Magnum::Color4{1} */)
    : Drawable{node, shader, mesh, group},
      texture_(texture),
      objectId_(objectId),
      color_{color} {}

void GenericDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                           Magnum::SceneGraph::Camera3D& camera) {
  Magnum::Shaders::Flat3D& shader =
      static_cast<Magnum::Shaders::Flat3D&>(shader_);
  shader.setTransformationProjectionMatrix(camera.projectionMatrix() *
                                           transformationMatrix);

  if ((shader.flags() & Magnum::Shaders::Flat3D::Flag::Textured) && texture_) {
    shader.bindTexture(*texture_);
  }

  if (!(shader.flags() & Magnum::Shaders::Flat3D::Flag::VertexColor)) {
    shader.setColor(color_);
  }

  shader.setObjectId(node_.getId());
  mesh_.draw(shader_);
}

}  // namespace gfx
}  // namespace esp
