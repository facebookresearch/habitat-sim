// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"
#include "GenericShader.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(
    scene::SceneNode& node,
    GenericShader& shader,
    Magnum::GL::Mesh& mesh,
    Magnum::SceneGraph::DrawableGroup3D* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    const Magnum::Color4& color /* = Magnum::Color4{1} */)
    : Drawable{node, shader, mesh, group},
      texture_(texture),
      objectId_(objectId),
      color_{color} {}

void GenericDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                           Magnum::SceneGraph::Camera3D& camera) {
  GenericShader& shader = static_cast<GenericShader&>(shader_);
  shader
      .setTransformationProjectionMatrix(camera.projectionMatrix() *
                                         transformationMatrix)
      .setProjectionMatrix(transformationMatrix);

  if (((shader.flags() & GenericShader::Flag::Textured) ||
       (shader.flags() & GenericShader::Flag::PrimitiveIDTextured)) &&
      texture_) {
    shader.bindTexture(*texture_);
  }

  if (!(shader.flags() & GenericShader::Flag::VertexColored)) {
    shader.setColor(color_);
  }

  if (!(shader.flags() & GenericShader::Flag::PerVertexIds) &&
      !(shader.flags() & GenericShader::Flag::PrimitiveIDTextured)) {
    shader.setObjectId(node_.getId());
  }
  mesh_.draw(shader_);
}

}  // namespace gfx
}  // namespace esp
