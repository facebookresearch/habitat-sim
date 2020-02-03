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
    Magnum::GL::Mesh& mesh,
    DrawableGroup* group /* = nullptr */,
    Magnum::GL::Texture2D* texture /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    const Magnum::Color4& color /* = Magnum::Color4{1} */)
    : Drawable{node, mesh, group},
      texture_(texture),
      objectId_(objectId),
      color_{color} {}

void GenericDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                           Magnum::SceneGraph::Camera3D& camera) {
  Shader::ptr shader = drawables()->getShader();
  ShaderType shaderType = shader->getConfiguration().type;
  Magnum::GL::AbstractShaderProgram& shaderProgram = shader->getShaderProgram();

  // TODO: use polymorphism to do double dispatch here
  if (type == PHONG_SHADER) {
    Magnum::Shaders::Phong& phongShader =
        static_cast<Magnum::Shaders::Phong&>(shaderProgram);
    phongShader.setTransformationMatrix(transformationMatrix)
        .setProjectionMatrix(camera.projectionMatrix())
        .setNormalMatrix(transformationMatrix.rotationScaling())
        .setObjectId(node_.getId());

    if ((phongShader.flags() & Magnum::Shaders::Phong::Flag::DiffuseTexture) &&
        texture_) {
      phongShader.bindDiffuseTexture(*texture_);
    }

    if (!(phongShader.flags() & Magnum::Shaders::Phong::Flag::VertexColor)) {
      phongShader.setDiffuseColor(color_);
    }

  } else {
    Magnum::Shaders::Flat3D& flatShader =
        static_cast<Magnum::Shaders::Flat3D&>(shaderProgram);
    flatShader.setTransformationProjectionMatrix(camera.projectionMatrix() *
                                                 transformationMatrix);

    if ((flatShader.flags() & Magnum::Shaders::Flat3D::Flag::Textured) &&
        texture_) {
      flatShader.bindTexture(*texture_);
    }
    if (!(flatShader.flags() & Magnum::Shaders::Flat3D::Flag::VertexColor)) {
      flatShader.setColor(color_);
    }

    flatShader.setObjectId(node_.getId());
  }
  mesh_.draw(shaderProgram);
}

}  // namespace gfx
}  // namespace esp
