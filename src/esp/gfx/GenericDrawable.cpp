// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Magnum/Shaders/Flat.h>
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

GenericFlatDrawable::GenericFlatDrawable(
    scene::SceneNode& node,
    Magnum::Shaders::Flat3D& shader,
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

  std::vector<Magnum::Vector3> CODALightGlobals = {
      Magnum::Vector3(6.604, 2.78244, -2.33516),
      Magnum::Vector3(6.35092, 2.88244, -4.90123),
      Magnum::Vector3(.35093, 2.88244, -4.90123),
      Magnum::Vector3(1.85093, 2.88244, -4.90123),
      Magnum::Vector3(1.85093, 2.88244, -2.30123),
      Magnum::Vector3(4.45093, 2.88244, -2.30123)};

  std::vector<Magnum::Vector3> CODALightsTransformed;
  for (auto& p : CODALightGlobals) {
    CODALightsTransformed.push_back(camera.cameraMatrix().transformPoint(p));
  }

  shader.setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .setObjectId(node_.getId())
      .setLightPositions(CODALightsTransformed);

  /*
  //Global lights above
  shader.setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .setObjectId(node_.getId())
      .setLightPositions({
    camera.cameraMatrix().transformPoint(Magnum::Vector3{0.0f, 10.0f, 10.0f} *
  100.0f),
                                camera.cameraMatrix().transformPoint(Magnum::Vector3{0.0f,
  10.0f, 10.0f} * 100.0f),
                                camera.cameraMatrix().transformPoint(Magnum::Vector3{0.0f,
  10.0f, 10.0f} * 100.0f)
                                });
                                */

  /**
   * CODA romo lights:
   * 6.604, 2.78244, -2.33516
   * 6.35092, 2.88244, -4.90123
   * 4.35093, 2.88244, -4.90123
   * 1.85093, 2.88244, -4.90123
   * 1.85093, 2.88244, -2.30123
   * 4.45093, 2.88244, -2.30123
   */

  if ((shader.flags() & Magnum::Shaders::Phong::Flag::DiffuseTexture) &&
      texture_) {
    shader.bindDiffuseTexture(*texture_);
  }

  if (!(shader.flags() & Magnum::Shaders::Phong::Flag::VertexColor)) {
    shader.setDiffuseColor(color_);
  }

  mesh_.draw(shader_);
}

void GenericFlatDrawable::draw(const Magnum::Matrix4& transformationMatrix,
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
