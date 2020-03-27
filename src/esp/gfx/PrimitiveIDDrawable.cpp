// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PrimitiveIDDrawable.h"
#include "PrimitiveIDShader.h"
#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

// static constexpr arrays require redundant definitions until C++17
constexpr char PrimitiveIDDrawable::SHADER_KEY[];

PrimitiveIDDrawable::PrimitiveIDDrawable(scene::SceneNode& node,
                                         Magnum::GL::Mesh& mesh,
                                         ShaderManager& shaderManager,
                                         DrawableGroup* group /* = nullptr */)
    : Drawable{node, mesh, group} {
  auto shaderResource =
      shaderManager.get<Magnum::GL::AbstractShaderProgram, PrimitiveIDShader>(
          SHADER_KEY);

  if (!shaderResource) {
    shaderManager.set<Magnum::GL::AbstractShaderProgram>(
        shaderResource.key(), new PrimitiveIDShader{});
  }
  shader_ = &(*shaderResource);
}

void PrimitiveIDDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                               Magnum::SceneGraph::Camera3D& camera) {
  (*shader_)
      .setTransformationProjectionMatrix(camera.projectionMatrix() *
                                         transformationMatrix)
      .draw(mesh_);
}

}  // namespace gfx
}  // namespace esp
