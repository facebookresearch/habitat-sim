// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>

#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(scene::SceneNode& node,
                                 Mn::GL::Mesh& mesh,
                                 Drawable::Flags& meshAttributeFlags,
                                 ShaderManager& shaderManager,
                                 const Mn::ResourceKey& lightSetupKey,
                                 const Mn::ResourceKey& materialDataKey,
                                 DrawableGroup* group /* = nullptr */)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetupKey)},
      materialData_{
          shaderManager.get<MaterialData, PhongMaterialData>(materialDataKey)} {
  flags_ = Mn::Shaders::Phong::Flag::ObjectId;
  if (materialData_->textureMatrix != Mn::Matrix3{}) {
    flags_ |= Mn::Shaders::Phong::Flag::TextureTransformation;
  }
  if (materialData_->ambientTexture) {
    flags_ |= Mn::Shaders::Phong::Flag::AmbientTexture;
  }
  if (materialData_->diffuseTexture) {
    flags_ |= Mn::Shaders::Phong::Flag::DiffuseTexture;
  }
  if (materialData_->specularTexture) {
    flags_ |= Mn::Shaders::Phong::Flag::SpecularTexture;
  }
  if (materialData_->normalTexture) {
    if (meshAttributeFlags & Drawable::Flag::HasTangent) {
      flags_ |= Mn::Shaders::Phong::Flag::NormalTexture;
      if (meshAttributeFlags & Drawable::Flag::HasSeparateBitangent) {
        flags_ |= Mn::Shaders::Phong::Flag::Bitangent;
      }
    } else {
      LOG(WARNING) << "Mesh does not have tangents and Magnum cannot generate "
                      "them yet, ignoring a normal map";
    }
  }
  if (materialData_->perVertexObjectId) {
    flags_ |= Mn::Shaders::Phong::Flag::InstancedObjectId;
  }
  if (materialData_->vertexColored) {
    flags_ |= Mn::Shaders::Phong::Flag::VertexColor;
  }

  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::setLightSetup(const Mn::ResourceKey& resourceKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(resourceKey);

  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::updateShaderLightingParameters(
    const Mn::Matrix4& transformationMatrix,
    Mn::SceneGraph::Camera3D& camera) {
  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();

  std::vector<Mn::Vector4> lightPositions;
  lightPositions.reserve(lightSetup_->size());
  std::vector<Mn::Color3> lightColors;
  lightColors.reserve(lightSetup_->size());
  std::vector<Mn::Color3> lightSpecularColors;
  lightSpecularColors.reserve(lightSetup_->size());
  constexpr float dummyRange = Mn::Constants::inf();
  std::vector<float> lightRanges(lightSetup_->size(), dummyRange);
  const Mn::Color4 ambientLightColor = getAmbientLightColor(*lightSetup_);

  for (Mn::UnsignedInt i = 0; i < lightSetup_->size(); ++i) {
    const auto& lightInfo = (*lightSetup_)[i];
    lightPositions.emplace_back(Mn::Vector4(getLightPositionRelativeToCamera(
        lightInfo, transformationMatrix, cameraMatrix)));

    const auto& lightColor = (*lightSetup_)[i].color;
    lightColors.emplace_back(lightColor);

    // In general, a light's specular color should match its base color.
    // However, negative lights have zero (black) specular.
    constexpr Mn::Color3 blackColor(0.0, 0.0, 0.0);
    bool isNegativeLight = lightColor.x() < 0;
    lightSpecularColors.emplace_back(isNegativeLight ? blackColor : lightColor);
  }

  // See documentation in src/deps/magnum/src/Magnum/Shaders/Phong.h
  (*shader_)
      .setAmbientColor(materialData_->ambientColor * ambientLightColor)
      .setDiffuseColor(materialData_->diffuseColor)
      .setSpecularColor(materialData_->specularColor)
      .setShininess(materialData_->shininess)
      .setLightPositions(lightPositions)
      .setLightColors(lightColors)
      .setLightRanges(lightRanges);
}

void GenericDrawable::draw(const Mn::Matrix4& transformationMatrix,
                           Mn::SceneGraph::Camera3D& camera) {
  updateShader();

  updateShaderLightingParameters(transformationMatrix, camera);

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(
          static_cast<RenderCamera&>(camera).useDrawableIds()
              ? drawableId_
              : (materialData_->perVertexObjectId ? 0 : node_.getSemanticId()))
      .setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.normalMatrix());

  if ((flags_ & Mn::Shaders::Phong::Flag::TextureTransformation) &&
      materialData_->textureMatrix != Mn::Matrix3{}) {
    shader_->setTextureMatrix(materialData_->textureMatrix);
  }

  if (flags_ & Mn::Shaders::Phong::Flag::AmbientTexture) {
    shader_->bindAmbientTexture(*(materialData_->ambientTexture));
  }
  if (flags_ & Mn::Shaders::Phong::Flag::DiffuseTexture) {
    shader_->bindDiffuseTexture(*(materialData_->diffuseTexture));
  }
  if (flags_ & Mn::Shaders::Phong::Flag::SpecularTexture) {
    shader_->bindSpecularTexture(*(materialData_->specularTexture));
  }
  if (flags_ & Mn::Shaders::Phong::Flag::NormalTexture) {
    shader_->bindNormalTexture(*(materialData_->normalTexture));
  }

  shader_->draw(mesh_);
}

void GenericDrawable::updateShader() {
  Mn::UnsignedInt lightCount = lightSetup_->size();

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags_) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, Mn::Shaders::Phong>(
            getShaderKey(lightCount, flags_));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new Mn::Shaders::Phong{flags_, lightCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags_);
  }
}

Mn::ResourceKey GenericDrawable::getShaderKey(
    Mn::UnsignedInt lightCount,
    Mn::Shaders::Phong::Flags flags) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<Mn::Shaders::Phong::Flags::UnderlyingType>(flags));
}

}  // namespace gfx
}  // namespace esp
