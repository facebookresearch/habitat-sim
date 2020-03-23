// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Corrade/Utility/FormatStl.h>

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(scene::SceneNode& node,
                                 Magnum::GL::Mesh& mesh,
                                 ShaderManager& shaderManager,
                                 const Magnum::ResourceKey& lightSetup,
                                 const Magnum::ResourceKey& materialData,
                                 DrawableGroup* group /* = nullptr */,
                                 int objectId /* = ID_UNDEFINED */)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetup)},
      materialData_{
          shaderManager.get<MaterialData, PhongMaterialData>(materialData)},
      objectId_(objectId) {
  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::setLightSetup(const Magnum::ResourceKey& resourceKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(resourceKey);

  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::draw(const Magnum::Matrix4& transformationMatrix,
                           Magnum::SceneGraph::Camera3D& camera) {
  updateShader();

  const Magnum::Matrix4 cameraMatrix = camera.cameraMatrix();

  std::vector<Magnum::Vector3> lightPositions;
  lightPositions.reserve(lightSetup_->size());
  std::vector<Magnum::Color4> lightColors;
  lightColors.reserve(lightSetup_->size());

  for (Magnum::UnsignedInt i = 0; i < lightSetup_->size(); ++i) {
    lightPositions.emplace_back(getLightPositionRelativeToCamera(
        (*lightSetup_)[i], transformationMatrix, cameraMatrix));

    lightColors.emplace_back((*lightSetup_)[i].color);
  }

  (*shader_)
      .setAmbientColor(materialData_->ambientColor)
      .setDiffuseColor(materialData_->diffuseColor)
      .setSpecularColor(materialData_->specularColor)
      .setShininess(materialData_->shininess)
      .setObjectId(node_.getId())
      .setLightPositions(lightPositions)
      .setLightColors(lightColors)
      .setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.rotationScaling());

  if (materialData_->ambientTexture)
    shader_->bindAmbientTexture(*(materialData_->ambientTexture));
  if (materialData_->diffuseTexture)
    shader_->bindDiffuseTexture(*(materialData_->diffuseTexture));
  if (materialData_->specularTexture)
    shader_->bindSpecularTexture(*(materialData_->specularTexture));

  mesh_.draw(*shader_);
}

void GenericDrawable::updateShader() {
  Magnum::UnsignedInt lightCount = lightSetup_->size();
  Magnum::Shaders::Phong::Flags flags = Magnum::Shaders::Phong::Flag::ObjectId;

  if (materialData_->ambientTexture)
    flags |= Magnum::Shaders::Phong::Flag::AmbientTexture;
  if (materialData_->diffuseTexture)
    flags |= Magnum::Shaders::Phong::Flag::DiffuseTexture;
  if (materialData_->specularTexture)
    flags |= Magnum::Shaders::Phong::Flag::SpecularTexture;

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ =
        shaderManager_
            .get<Magnum::GL::AbstractShaderProgram, Magnum::Shaders::Phong>(
                getShaderKey(lightCount, flags));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Magnum::GL::AbstractShaderProgram>(
          shader_.key(), new Magnum::Shaders::Phong{flags, lightCount},
          Magnum::ResourceDataState::Final,
          Magnum::ResourcePolicy::ReferenceCounted);
    }
  }
}

Magnum::ResourceKey GenericDrawable::getShaderKey(
    Magnum::UnsignedInt lightCount,
    Magnum::Shaders::Phong::Flags flags) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<Magnum::Shaders::Phong::Flags::UnderlyingType>(flags));
}

}  // namespace gfx
}  // namespace esp
