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
                                 ShaderManager& shaderManager,
                                 const Mn::ResourceKey& lightSetup,
                                 const Mn::ResourceKey& materialData,
                                 DrawableGroup* group /* = nullptr */)
    : Drawable{node, mesh, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetup)},
      materialData_{
          shaderManager.get<MaterialData, PhongMaterialData>(materialData)} {
  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::setLightSetup(const Mn::ResourceKey& resourceKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(resourceKey);

  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void GenericDrawable::draw(const Mn::Matrix4& transformationMatrix,
                           Mn::SceneGraph::Camera3D& camera) {
  updateShader();

  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();

  std::vector<Mn::Vector3> lightPositions;
  lightPositions.reserve(lightSetup_->size());
  std::vector<Mn::Color4> lightColors;
  lightColors.reserve(lightSetup_->size());

  for (Mn::UnsignedInt i = 0; i < lightSetup_->size(); ++i) {
    lightPositions.emplace_back(getLightPositionRelativeToCamera(
        (*lightSetup_)[i], transformationMatrix, cameraMatrix));

    lightColors.emplace_back((*lightSetup_)[i].color);
  }

  (*shader_)
      .setAmbientColor(materialData_->ambientColor)
      .setDiffuseColor(materialData_->diffuseColor)
      .setSpecularColor(materialData_->specularColor)
      .setShininess(materialData_->shininess)
      .setLightPositions(lightPositions)
      .setLightColors(lightColors)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(
          static_cast<RenderCamera&>(camera).useDrawableIds()
              ? drawableId_
              : (materialData_->perVertexObjectId ? 0 : node_.getSemanticId()))
      .setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(transformationMatrix.rotationScaling());

  if (materialData_->textureMatrix != Mn::Matrix3{})
    shader_->setTextureMatrix(materialData_->textureMatrix);

  if (materialData_->ambientTexture)
    shader_->bindAmbientTexture(*(materialData_->ambientTexture));
  if (materialData_->diffuseTexture)
    shader_->bindDiffuseTexture(*(materialData_->diffuseTexture));
  if (materialData_->specularTexture)
    shader_->bindSpecularTexture(*(materialData_->specularTexture));
  if (materialData_->normalTexture)
    shader_->bindNormalTexture(*(materialData_->normalTexture));

  shader_->draw(mesh_);
}

void GenericDrawable::updateShader() {
  Mn::UnsignedInt lightCount = lightSetup_->size();
  Mn::Shaders::Phong::Flags flags = Mn::Shaders::Phong::Flag::ObjectId;

  if (materialData_->textureMatrix != Mn::Matrix3{})
    flags |= Mn::Shaders::Phong::Flag::TextureTransformation;
  if (materialData_->ambientTexture)
    flags |= Mn::Shaders::Phong::Flag::AmbientTexture;
  if (materialData_->diffuseTexture)
    flags |= Mn::Shaders::Phong::Flag::DiffuseTexture;
  if (materialData_->specularTexture)
    flags |= Mn::Shaders::Phong::Flag::SpecularTexture;
  if (materialData_->normalTexture)
    flags |= Mn::Shaders::Phong::Flag::NormalTexture;
  if (materialData_->perVertexObjectId)
    flags |= Mn::Shaders::Phong::Flag::InstancedObjectId;
  if (materialData_->vertexColored)
    flags |= Mn::Shaders::Phong::Flag::VertexColor;

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, Mn::Shaders::Phong>(
            getShaderKey(lightCount, flags));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(), new Mn::Shaders::Phong{flags, lightCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags);
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
