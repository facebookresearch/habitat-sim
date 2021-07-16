// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

#include "DepthMapDrawable.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

DepthMapDrawable::DepthMapDrawable(scene::SceneNode& node,
                                   Magnum::GL::Mesh* mesh,
                                   ShaderManager& shaderManager,
                                   DrawableGroup* group)
    : DepthMapDrawableBase{node, mesh, shaderManager, DrawableType::DepthMap,
                           group} {}

void DepthMapDrawable::draw(const Mn::Matrix4& transformationMatrix,
                            Mn::SceneGraph::Camera3D& camera) {
  if (!shader_) {
    shader_ = shaderManager_.get<Mn::GL::AbstractShaderProgram, DepthMapShader>(
        "depth-map-shader");
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new DepthMapShader{}, Mn::ResourceDataState::Final,
          Mn::ResourcePolicy::ReferenceCounted);
    }
    CORRADE_INTERNAL_ASSERT(shader_);
  }

  // NOTE:
  // we assume camera is properly set externally, and it has light position and
  // orientation
  shader_->setLightProjectionMatrix(camera.projectionMatrix());
  shader_->setLightModelViewMatrix(transformationMatrix);
  shader_->draw(getMesh());
}

}  // namespace gfx
}  // namespace esp
