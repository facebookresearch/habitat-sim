// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "MeshVisualizerDrawable.h"
#include "Magnum/GL/Renderer.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

MeshVisualizerDrawable::MeshVisualizerDrawable(
    scene::SceneNode& node,
    Magnum::Shaders::MeshVisualizerGL3D& shader,
    Magnum::GL::Mesh& mesh,
    DrawableConfiguration& cfg)
    : Drawable{node, &mesh, DrawableType::MeshVisualizer, cfg,
               Magnum::Resource<LightSetup>()},
      shader_(shader) {}

void MeshVisualizerDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                                  Magnum::SceneGraph::Camera3D& camera) {
  CORRADE_ASSERT(glMeshExists(),
                 "MeshVisualizerDrawable::draw() : GL mesh doesn't exist", );

  Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::PolygonOffsetFill);
  Mn::GL::Renderer::setPolygonOffset(-5.0f, -5.0f);

  shader_.setProjectionMatrix(camera.projectionMatrix())
      .setTransformationMatrix(transformationMatrix);

  shader_.draw(getMesh());

  Mn::GL::Renderer::setPolygonOffset(0.0f, 0.0f);
  Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::PolygonOffsetFill);
}
}  // namespace gfx
}  // namespace esp
