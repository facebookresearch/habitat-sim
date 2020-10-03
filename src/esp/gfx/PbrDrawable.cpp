// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

PbrDrawable::PbrDrawable(scene::SceneNode& node,
                         Mn::GL::Mesh& mesh,
                         ShaderManager& shaderManager,
                         const Mn::ResourceKey& lightSetupKey,
                         const Mn::ResourceKey& materialDataKey,
                         DrawableGroup* group)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetupKey)},
      materialData_{
          shaderManager.get<MaterialData, PbrMaterialData>(materialDataKey)} {
  updateShader().updateShaderLightParameters();
}

void PbrDrawable::setLightSetup(const Mn::ResourceKey& lightSetup) {
  lightSetup_ = shaderManager_.get<LightSetup>(lightSetup);
  updateShader().updateShaderLightParameters();
}

void PbrDrawable::draw(const Mn::Matrix4& transformationMatrix,
                       Mn::SceneGraph::Camera3D& camera) {
  // no need to call updateShaderLightParameters() in the draw loop,
  // only need to update the position or direction of the lights
  updateShader().updateShaderLightDirectionParameters(transformationMatrix,
                                                      camera);

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(
          static_cast<RenderCamera&>(camera).useDrawableIds()
              ? drawableId_
              : (materialData_->perVertexObjectId ? 0 : node_.getSemanticId()))
      .setTransformationMatrix(transformationMatrix)  // modelview matrix
      .setMVPMatrix(camera.projectionMatrix() * transformationMatrix)
      .setNormalMatrix(transformationMatrix.normalMatrix())
      .bindTextures(materialData_->baseColorTexture,
                    materialData_->roughnessTexture,
                    materialData_->metallicTexture,
                    materialData_->noneRoughnessMetallicTexture,
                    materialData_->occlusionRoughnessMetallicTexture,
                    materialData_->normalTexture)
      .setBaseColor(materialData_->baseColor)
      .setRoughness(materialData_->roughness)
      .setMetallic(materialData_->metallic);
  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    shader_->setTextureMatrix(materialData_->textureMatrix);
  }

  shader_->draw(mesh_);
}

Mn::ResourceKey PbrDrawable::getShaderKey(Mn::UnsignedInt lightCount,
                                          PbrShader::Flags flags) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<PbrShader::Flags::UnderlyingType>(
          PbrShader::generateCorrectFlags(flags)));
}

PbrDrawable& PbrDrawable::updateShader() {
  PbrShader::Flags flags = PbrShader::Flag::OBJECT_ID;

  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    flags |= PbrShader::Flag::TEXTURE_TRANSFORMATION;
  }
  if (materialData_->baseColorTexture) {
    flags |= PbrShader::Flag::BASE_COLOR_TEXTURE;
  }
  if (materialData_->occlusionRoughnessMetallicTexture) {
    flags |= PbrShader::Flag::OCCLUSION_ROUGHNESS_METALLIC_TEXTURE;
  }
  if (materialData_->noneRoughnessMetallicTexture) {
    flags |= PbrShader::Flag::NONE_ROUGHNESS_METALLIC_TEXTURE;
  }
  if (materialData_->roughnessTexture)
    flags |= PbrShader::Flag::ROUGHNESS_TEXTURE;
  if (materialData_->metallicTexture)
    flags |= PbrShader::Flag::METALLIC_TEXTURE;
  if (materialData_->normalTexture)
    flags |= PbrShader::Flag::NORMAL_TEXTURE;
  if (materialData_->perVertexObjectId) {
    // TODO: may be supported in the future
  }

  flags = PbrShader::generateCorrectFlags(flags);

  unsigned int lightCount = lightSetup_->size();
  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ = shaderManager_.get<Mn::GL::AbstractShaderProgram, PbrShader>(
        getShaderKey(lightCount, flags));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new PbrShader{flags, lightCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags);
  }

  return *this;
}

// update every light's color, intensity, range etc.
PbrDrawable& PbrDrawable::updateShaderLightParameters() {
  constexpr float dummyRange = Mn::Constants::inf();
  std::vector<float> ranges;
  std::vector<Mn::Color3> colors;
  colors.reserve(lightSetup_->size());
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    ranges.push_back(dummyRange);
    // Note: the light color MUST take the intensity into account
    colors.emplace_back((*lightSetup_)[iLight].color);
  }

  shader_->setLightRanges(ranges);
  shader_->setLightColors(colors);
  return *this;
}

// update light direction (or position) in *camera* space to the shader
PbrDrawable& PbrDrawable::updateShaderLightDirectionParameters(
    const Magnum::Matrix4& transformationMatrix,
    Magnum::SceneGraph::Camera3D& camera) {
  std::vector<Mn::Vector4> lightPositions;
  lightPositions.reserve(lightSetup_->size());

  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    const auto& lightInfo = (*lightSetup_)[iLight];
    lightPositions.emplace_back(Mn::Vector4(getLightPositionRelativeToCamera(
        lightInfo, transformationMatrix, cameraMatrix)));
  }

  shader_->setLightVectors(lightPositions);

  return *this;
}

}  // namespace gfx
}  // namespace esp
