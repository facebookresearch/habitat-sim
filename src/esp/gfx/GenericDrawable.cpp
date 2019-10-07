// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Magnum/Shaders/Phong.h>

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(
    scene::SceneNode& node,
    Magnum::Shaders::Phong& shader,
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
  Magnum::Shaders::Phong& shader =
      static_cast<Magnum::Shaders::Phong&>(shader_);
  shader.setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .setObjectId(node_.getId());

  if ((shader.flags() & Magnum::Shaders::Phong::Flag::DiffuseTexture) &&
      texture_) {
    shader.bindDiffuseTexture(*texture_);
  }

  if (!(shader.flags() & Magnum::Shaders::Phong::Flag::VertexColor)) {
    shader.setDiffuseColor(color_);
  }

  mesh_.draw(shader_);
}

}  // namespace gfx
}  // namespace esp
