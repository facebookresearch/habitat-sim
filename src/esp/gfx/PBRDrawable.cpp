// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PBRDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

PBRDrawable::PBRDrawable(scene::SceneNode& node,
                         Mn::GL::Mesh& mesh,
                         ShaderManager& shaderManager,
                         const Mn::ResourceKey& lightSetupKey,
                         const Mn::ResourceKey& materialDataKey,
                         DrawableGroup* group)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetupKey)},
      materialData_{
          shaderManager.get<MaterialData, PBRMaterialData>(materialDataKey)} {
  updateShader().updateShaderLightParameters();
}

void PBRDrawable::setLightSetup(const Mn::ResourceKey& lightSetup) {
  lightSetup_ = shaderManager_.get<LightSetup>(lightSetup);
  updateShader().updateShaderLightParameters();
}

void PBRDrawable::draw(const Mn::Matrix4& transformationMatrix,
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
      // because we actually do not have scaling, only rotation, so that the
      // modelview matrix mv = inv(mv^T) where mv stands for the up-left 3x3
      // part of the modelview matrix
      .setNormalMatrix(transformationMatrix.rotationScaling())
      .bindTextures(
          materialData_->albedoTexture, materialData_->roughnessTexture,
          materialData_->metallicTexture, materialData_->normalTexture)
      .setBaseColor(materialData_->baseColor)
      .setRoughness(materialData_->roughness)
      .setMetallic(materialData_->metallic);

  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    // TODO
  }

  shader_->draw(mesh_);
}

Mn::ResourceKey PBRDrawable::getShaderKey(Mn::UnsignedInt lightCount,
                                          PBRShader::Flags flags) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<PBRShader::Flags::UnderlyingType>(flags));
}

PBRDrawable& PBRDrawable::updateShader() {
  PBRShader::Flags flags = PBRShader::Flag::ObjectId;

  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    // TODO: may be supported in the future
  }
  if (materialData_->albedoTexture)
    flags |= PBRShader::Flag::AlbedoTexture;
  if (materialData_->roughnessTexture)
    flags |= PBRShader::Flag::RoughnessTexture;
  if (materialData_->metallicTexture)
    flags |= PBRShader::Flag::MetallicTexture;
  if (materialData_->normalTexture)
    flags |= PBRShader::Flag::NormalTexture;
  if (materialData_->perVertexObjectId) {
    // TODO: may be supported in the future
  }

  unsigned int lightCount = lightSetup_->size();
  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ = shaderManager_.get<Mn::GL::AbstractShaderProgram, PBRShader>(
        getShaderKey(lightCount, flags));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new PBRShader{flags, lightCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags);
  }

  return *this;
}

// update every light's color, intensity, range etc.
PBRDrawable& PBRDrawable::updateShaderLightParameters() {
  constexpr float dummyRange = 10000.0;
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    // Note: the light color already includes the intensity
    (*shader_).setLightColor(iLight, (*lightSetup_)[iLight].color, dummyRange);
  }
  return *this;
}

// update light direction (or position) in *camera* space to the shader
PBRDrawable& PBRDrawable::updateShaderLightDirectionParameters(
    const Magnum::Matrix4& transformationMatrix,
    Magnum::SceneGraph::Camera3D& camera) {
  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    const auto& lightInfo = (*lightSetup_)[iLight];
    (*shader_).setLightVector(
        iLight, Mn::Vector4(getLightPositionRelativeToCamera(
                    lightInfo, transformationMatrix, cameraMatrix)));
  }
  return *this;
}

}  // namespace gfx
}  // namespace esp
