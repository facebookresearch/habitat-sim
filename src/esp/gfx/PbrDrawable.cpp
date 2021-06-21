// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/GL/Renderer.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

PbrDrawable::PbrDrawable(scene::SceneNode& node,
                         Mn::GL::Mesh& mesh,
                         gfx::Drawable::Flags& meshAttributeFlags,
                         ShaderManager& shaderManager,
                         const Mn::ResourceKey& lightSetupKey,
                         const Mn::ResourceKey& materialDataKey,
                         DrawableGroup* group)
    : Drawable{node, mesh, DrawableType::Pbr, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetupKey)},
      materialData_{
          shaderManager.get<MaterialData, PbrMaterialData>(materialDataKey)} {
  if (materialData_->metallicTexture && materialData_->roughnessTexture) {
    CORRADE_ASSERT(
        materialData_->metallicTexture == materialData_->roughnessTexture,
        "PbrDrawable::PbrDrawable(): if both the metallic and roughness "
        "texture exist, they must be packed in the same texture based on glTF "
        "2.0 Spec.", );
  }

  flags_ = PbrShader::Flag::ObjectId;
  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    flags_ |= PbrShader::Flag::TextureTransformation;
  }
  if (materialData_->baseColorTexture) {
    flags_ |= PbrShader::Flag::BaseColorTexture;
  }
  if (materialData_->roughnessTexture) {
    flags_ |= PbrShader::Flag::RoughnessTexture;
  }
  if (materialData_->metallicTexture) {
    flags_ |= PbrShader::Flag::MetallicTexture;
  }
  if (materialData_->normalTexture) {
    flags_ |= PbrShader::Flag::NormalTexture;
    if (meshAttributeFlags & gfx::Drawable::Flag::HasTangent) {
      flags_ |= PbrShader::Flag::PrecomputedTangent;
    }
    if (materialData_->normalTextureScale != 1.0f) {
      flags_ |= PbrShader::Flag::NormalTextureScale;
      CORRADE_ASSERT(materialData_->normalTextureScale > 0.0f,
                     "PbrDrawable::PbrDrawable(): the normal texture scale "
                     "must be positive.", );
    }
  }
  if (materialData_->emissiveTexture) {
    flags_ |= PbrShader::Flag::EmissiveTexture;
  }
  if (materialData_->perVertexObjectId) {
    // TODO: may be supported in the future
  }
  if (materialData_->doubleSided) {
    flags_ |= PbrShader::Flag::DoubleSided;
  }

  // Defer the shader initialization because at this point, the lightSetup may
  // not be done in the Simulator. Simulator itself is currently under
  // construction in this case.
  // updateShader().updateShaderLightParameters();
}

void PbrDrawable::setLightSetup(const Mn::ResourceKey& lightSetupKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(lightSetupKey);
}

void PbrDrawable::draw(const Mn::Matrix4& transformationMatrix,
                       Mn::SceneGraph::Camera3D& camera) {
  updateShader()
      .updateShaderLightParameters()
      .updateShaderLightDirectionParameters(transformationMatrix, camera);

  // Assume that in a model, double-sided meshes are significantly less than
  // single-sided meshes.
  // To reduce the usage of glIsEnabled, once a double-sided mesh is
  // encountered, the FaceCulling is disabled, and will not be enabled again at
  // least in this function.
  // TODO:
  // it should have a global GL state tracker in Magnum to track it.

  if ((flags_ & PbrShader::Flag::DoubleSided) && glIsEnabled(GL_CULL_FACE)) {
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::FaceCulling);
  }

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(
          static_cast<RenderCamera&>(camera).useDrawableIds()
              ? drawableId_
              : (materialData_->perVertexObjectId ? 0 : node_.getSemanticId()))
      .setTransformationMatrix(transformationMatrix)  // modelview matrix
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.normalMatrix())
      .setBaseColor(materialData_->baseColor)
      .setRoughness(materialData_->roughness)
      .setMetallic(materialData_->metallic)
      .setEmissiveColor(materialData_->emissiveColor);

  if ((flags_ & PbrShader::Flag::BaseColorTexture) &&
      (materialData_->baseColorTexture != nullptr)) {
    shader_->bindBaseColorTexture(*materialData_->baseColorTexture);
  }

  if (flags_ &
      (PbrShader::Flag::RoughnessTexture | PbrShader::Flag::MetallicTexture)) {
    Magnum::GL::Texture2D* metallicRoughnessTexture =
        materialData_->roughnessTexture;
    if (!metallicRoughnessTexture) {
      metallicRoughnessTexture = materialData_->metallicTexture;
    }
    CORRADE_ASSERT(metallicRoughnessTexture,
                   "PbrDrawable::draw(): texture pointer cannot be nullptr if "
                   "RoughnessTexture or MetallicTexture is enabled.", );
    shader_->bindMetallicRoughnessTexture(*metallicRoughnessTexture);
  }

  if ((flags_ & PbrShader::Flag::NormalTexture) &&
      (materialData_->normalTexture != nullptr)) {
    shader_->bindNormalTexture(*materialData_->normalTexture);
  }

  if ((flags_ & PbrShader::Flag::EmissiveTexture) &&
      (materialData_->emissiveTexture != nullptr)) {
    shader_->bindEmissiveTexture(*materialData_->emissiveTexture);
  }

  if ((flags_ & PbrShader::Flag::TextureTransformation) &&
      (materialData_->textureMatrix != Mn::Matrix3{})) {
    shader_->setTextureMatrix(materialData_->textureMatrix);
  }

  shader_->draw(mesh_);
}

Mn::ResourceKey PbrDrawable::getShaderKey(Mn::UnsignedInt lightCount,
                                          PbrShader::Flags flags) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<PbrShader::Flags::UnderlyingType>(flags));
}

PbrDrawable& PbrDrawable::updateShader() {
  unsigned int lightCount = lightSetup_->size();
  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags_) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ = shaderManager_.get<Mn::GL::AbstractShaderProgram, PbrShader>(
        getShaderKey(lightCount, flags_));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new PbrShader{flags_, lightCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags_);
  }

  return *this;
}

// update every light's color, intensity, range etc.
PbrDrawable& PbrDrawable::updateShaderLightParameters() {
  // light range has been initialized to Mn::Constants::inf()
  // in the PbrShader's constructor.
  // No need to reset it at this point.
  std::vector<Mn::Color3> colors;
  colors.reserve(lightSetup_->size());
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    // Note: the light color MUST take the intensity into account
    colors.emplace_back((*lightSetup_)[iLight].color);
  }

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
