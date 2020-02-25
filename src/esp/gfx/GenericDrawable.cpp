// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Corrade/Utility/FormatStl.h>

#include "esp/scene/SceneNode.h"

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(
    scene::SceneNode& node,
    Magnum::GL::Mesh& mesh,
    ShaderManager& shaderManager,
    const Magnum::ResourceKey& lightSetup,
    const Magnum::ResourceKey& materialData,
    DrawableGroup* group /* = nullptr */,
    int objectId /* = ID_UNDEFINED */,
    scene::SceneGraph::ShadowMapRegistry* shadowMapRegistry,
    bool shadeFacesFacingAwayFromLight)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetup)},
      materialData_{
          shaderManager.get<MaterialData, PhongMaterialData>(materialData)},
      objectId_(objectId),
      receivesShadow_{shadowMapRegistry != nullptr},
      shadowMapRegistry_{shadowMapRegistry},
      shadeFacesFacingAwayFromLight_{shadeFacesFacingAwayFromLight} {
  if (receivesShadow_) {
    lightSetupShadowMaps_ =
        shadowMapRegistry_->get<scene::SceneGraph::LightSetupShadowMaps>(
            lightSetup);
  }
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

  if (receivesShadow_ && lightSetupShadowMaps_) {
    // TODO: support more than 1 shadow light
    ShadowLight* shadowLight = (*lightSetupShadowMaps_)[0];

    Corrade::Containers::Array<Magnum::Matrix4> shadowMatrices{
        Corrade::Containers::NoInit, shadowLight->layerCount()};
    for (std::size_t layerIndex = 0; layerIndex != shadowLight->layerCount();
         ++layerIndex) {
      shadowMatrices[layerIndex] = shadowLight->layerMatrix(layerIndex);
    }

    (*shader_)
        // TODO: used cached model matrix computed at same time as
        // transformationMatrix
        .setModelMatrix(object().absoluteTransformationMatrix())
        .setShadowmapMatrices(shadowMatrices)
        .setShadowmapTexture(shadowLight->shadowTexture())
        .setShadeFacesFacingAwayFromLight(shadeFacesFacingAwayFromLight_)
        .setShadowLightDirection(shadowLight->getLightDirection())
        // TODO: make this dynamic
        .setShadowBias(0.0008f);
  }

  mesh_.draw(*shader_);
}

void GenericDrawable::updateShader() {
  Magnum::UnsignedInt lightCount = lightSetup_->size();
  PhongShadowReceiverShader::Flags flags =
      PhongShadowReceiverShader::Flag::ObjectId;
  Magnum::UnsignedInt layerCount = 0;
  if (lightSetupShadowMaps_) {
    layerCount = static_cast<Magnum::UnsignedInt>(
        (*lightSetupShadowMaps_)[0]->layerCount());
  }

  if (materialData_->ambientTexture)
    flags |= PhongShadowReceiverShader::Flag::AmbientTexture;
  if (materialData_->diffuseTexture)
    flags |= PhongShadowReceiverShader::Flag::DiffuseTexture;
  if (materialData_->specularTexture)
    flags |= PhongShadowReceiverShader::Flag::SpecularTexture;

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags || shader_->getNumLayers() != layerCount) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ =
        shaderManager_
            .get<Magnum::GL::AbstractShaderProgram, PhongShadowReceiverShader>(
                getShaderKey(lightCount, flags, layerCount));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Magnum::GL::AbstractShaderProgram>(
          shader_.key(),
          new PhongShadowReceiverShader{flags, lightCount, layerCount},
          Magnum::ResourceDataState::Final,
          Magnum::ResourcePolicy::ReferenceCounted);
    }
  }
}

Magnum::ResourceKey GenericDrawable::getShaderKey(
    Magnum::UnsignedInt lightCount,
    PhongShadowReceiverShader::Flags flags,
    Magnum::UnsignedInt layerCount) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<PhongShadowReceiverShader::Flags::UnderlyingType>(flags),
      layerCount);
}

}  // namespace gfx
}  // namespace esp
