// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "GenericDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/GL/Renderer.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/PhongMaterialData.h>

#include "Corrade/Containers/GrowableArray.h"
#include "Magnum/Types.h"
#include "esp/core/Check.h"
#include "esp/gfx/SkinData.h"
#include "esp/scene/SceneNode.h"

namespace Mn = Magnum;

namespace esp {
namespace gfx {

GenericDrawable::GenericDrawable(scene::SceneNode& node,
                                 Mn::GL::Mesh* mesh,
                                 Drawable::Flags& meshAttributeFlags,
                                 ShaderManager& shaderManager,
                                 DrawableConfiguration& cfg)
    : Drawable{node, mesh, DrawableType::Generic, cfg.group_},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(cfg.lightSetupKey_)},
      skinData_(cfg.getSkinData()),
      jointTransformations_(),
      meshAttributeFlags_{meshAttributeFlags} {
  resetMaterialValues(
      shaderManager.get<Mn::Trade::MaterialData, Mn::Trade::MaterialData>(
          cfg.materialDataKey_));

  // update the shader early here to to avoid doing it during the render loop
  if (glMeshExists()) {
    updateShader();
  }
}

void GenericDrawable::setMaterialValuesInternal(
    const Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData>&
        material,
    bool reset) {
  // copy the resource to nonconst so that it can be appropriately accessed
  auto materialData = material;
  // reset == true is intended for changing what actual shader features are
  // supported and will probably require a shader (re)build.

  // reset == false is specifically for changing existing values only, and
  // therefore should never require a shader rebuild. This is specifically
  // intended to support domain randomization in an efficient manner.

  // Use oldFlags to reset the flags_ after the material is repopulated (i.e. if
  // reset == false)
  Mn::Shaders::PhongGL::Flags oldFlags{flags_};

  if (reset) {
    matCache = {};
  }
  flags_ = Mn::Shaders::PhongGL::Flag::ObjectId;

  const auto& tmpMaterial = materialData->as<Mn::Trade::PhongMaterialData>();
  matCache.ambientColor = tmpMaterial.ambientColor();
  matCache.diffuseColor = tmpMaterial.diffuseColor();
  matCache.specularColor = tmpMaterial.specularColor();
  matCache.shininess = tmpMaterial.shininess();

  /* If texture transformation is specified, enable it only if the material is
     actually textured -- it's an error otherwise */
  if (tmpMaterial.commonTextureMatrix() != Mn::Matrix3{} &&
      (materialData->hasAttribute("ambientTexturePointer") ||
       materialData->hasAttribute("baseColorTexturePointer") ||
       materialData->hasAttribute("diffuseTexturePointer") ||
       materialData->hasAttribute("specularTexturePointer") ||
       materialData->hasAttribute("objectIdTexturePointer"))) {
    flags_ |= Mn::Shaders::PhongGL::Flag::TextureTransformation;
    matCache.textureMatrix = tmpMaterial.commonTextureMatrix();
  }
  if (materialData->hasAttribute("ambientTexturePointer")) {
    flags_ |= Mn::Shaders::PhongGL::Flag::AmbientTexture;
    matCache.ambientTexture =
        materialData->attribute<Mn::GL::Texture2D*>("ambientTexturePointer");
  }
  if (materialData->hasAttribute("diffuseTexturePointer")) {
    flags_ |= Mn::Shaders::PhongGL::Flag::DiffuseTexture;
    matCache.diffuseTexture =
        materialData->attribute<Mn::GL::Texture2D*>("diffuseTexturePointer");
  }
  if (materialData->hasAttribute("specularTexturePointer")) {
    flags_ |= Mn::Shaders::PhongGL::Flag::SpecularTexture;
    matCache.specularTexture =
        materialData->attribute<Mn::GL::Texture2D*>("specularTexturePointer");
  }

  if (materialData->hasAttribute("normalTexturePointer")) {
    matCache.normalTexture =
        materialData->attribute<Mn::GL::Texture2D*>("normalTexturePointer");
    if (meshAttributeFlags_ & Drawable::Flag::HasTangent) {
      flags_ |= Mn::Shaders::PhongGL::Flag::NormalTexture;

      if (meshAttributeFlags_ & Drawable::Flag::HasSeparateBitangent) {
        flags_ |= Mn::Shaders::PhongGL::Flag::Bitangent;
      }
    } else {
      ESP_DEBUG() << "Mesh does not have tangents and Magnum cannot generate "
                     "them yet, ignoring a normal map";
    }
  }
  if (materialData->attribute<bool>("hasPerVertexObjectId")) {
    flags_ |= Mn::Shaders::PhongGL::Flag::InstancedObjectId;
  }
  if (materialData->hasAttribute("objectIdTexturePointer")) {
    flags_ |= Mn::Shaders::PhongGL::Flag::ObjectIdTexture;
    matCache.objectIdTexture =
        materialData->attribute<Mn::GL::Texture2D*>("objectIdTexturePointer");
  }
  if (meshAttributeFlags_ & Drawable::Flag::HasVertexColor) {
    flags_ |= Mn::Shaders::PhongGL::Flag::VertexColor;
  }
  // If not reset then make sure the same shader is used
  if (!reset) {
    flags_ = oldFlags;
  }

}  // GenericDrawable::setMaterialValuesInternal

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
    Mn::Vector4 pos = getLightPositionRelativeToCamera(
        lightInfo, transformationMatrix, cameraMatrix);
    // flip directional lights to faciliate faster, non-forking calc in
    // shader.  Leave non-directional lights unchanged
    pos *= (pos[3] * 2) - 1;
    lightPositions.emplace_back(pos);

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
      .setAmbientColor(matCache.ambientColor * ambientLightColor)
      .setDiffuseColor(matCache.diffuseColor)
      .setSpecularColor(matCache.specularColor)
      .setShininess(matCache.shininess)
      .setLightPositions(lightPositions)
      .setLightColors(lightColors)
      .setLightRanges(lightRanges);
}

void GenericDrawable::draw(const Mn::Matrix4& transformationMatrix,
                           Mn::SceneGraph::Camera3D& camera) {
  CORRADE_ASSERT(glMeshExists(),
                 "GenericDrawable::draw() : GL mesh doesn't exist", );

  updateShader();

  updateShaderLightingParameters(transformationMatrix, camera);

  Mn::Matrix3x3 rotScale = transformationMatrix.rotationScaling();
  // Find determinant to calculate backface culling winding dir
  const float normalDet = rotScale.determinant();
  // Normal matrix is calculated as `m.inverted().transposed()`, and
  // `m.inverted()` is the same as `m.comatrix().transposed()/m.determinant()`.
  // We need the determinant to figure out the winding direction as well, thus
  // we calculate it separately and then do
  // `(m.comatrix().transposed()/determinant).transposed()`, which is the same
  // as `m.comatrix()/determinant`.
  Mn::Matrix3x3 normalMatrix = rotScale.comatrix() / normalDet;

  // Flip winding direction to correct handle backface culling
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(Mn::GL::Renderer::FrontFace::ClockWise);
  }

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has been
      // uploaded to GPU so simply pass 0 to the uniform "objectId" in the
      // fragment shader
      .setObjectId(
          static_cast<RenderCamera&>(camera).useDrawableIds() ? drawableId_
          : (((flags_ >= Mn::Shaders::PhongGL::Flag::InstancedObjectId) ||
              (flags_ >= Mn::Shaders::PhongGL::Flag::ObjectIdTexture)))
              ? 0
              : node_.getSemanticId())
      .setTransformationMatrix(transformationMatrix)
      .setProjectionMatrix(camera.projectionMatrix())
      .setNormalMatrix(normalMatrix);

  if (flags_ & Mn::Shaders::PhongGL::Flag::TextureTransformation) {
    shader_->setTextureMatrix(matCache.textureMatrix);
  }

  if (flags_ & Mn::Shaders::PhongGL::Flag::AmbientTexture) {
    shader_->bindAmbientTexture(*(matCache.ambientTexture));
  }
  if (flags_ & Mn::Shaders::PhongGL::Flag::DiffuseTexture) {
    shader_->bindDiffuseTexture(*(matCache.diffuseTexture));
  }
  if (flags_ & Mn::Shaders::PhongGL::Flag::SpecularTexture) {
    shader_->bindSpecularTexture(*(matCache.specularTexture));
  }
  if (flags_ & Mn::Shaders::PhongGL::Flag::NormalTexture) {
    shader_->bindNormalTexture(*(matCache.normalTexture));
  }
  if (flags_ >= Mn::Shaders::PhongGL::Flag::ObjectIdTexture) {
    shader_->bindObjectIdTexture(*(matCache.objectIdTexture));
  }

  if (skinData_) {
    // Gather joint transformations
    const auto& skin = skinData_->skinData->skin;
    const auto& transformNodes = skinData_->jointIdToTransformNode;

    ESP_CHECK(jointTransformations_.size() == skin->joints().size(),
              "Joint transformation count doesn't match bone count.");

    // Undo root node transform so that the model origin matches the root
    // articulated object link.
    const auto invRootTransform =
        skinData_->rootArticulatedObjectNode->absoluteTransformationMatrix()
            .inverted();

    for (std::size_t i = 0; i != jointTransformations_.size(); ++i) {
      const auto jointNodeIt = transformNodes.find(skin->joints()[i]);
      if (jointNodeIt != transformNodes.end()) {
        jointTransformations_[i] =
            invRootTransform *
            jointNodeIt->second->absoluteTransformationMatrix() *
            skin->inverseBindMatrices()[i];
      } else {
        // Joint not found, use placeholder matrix.
        jointTransformations_[i] = Mn::Matrix4{Mn::Math::IdentityInit};
      }
    }
    shader_->setJointMatrices(jointTransformations_);
  }

  shader_->draw(getMesh());

  // Reset winding direction
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(
        Mn::GL::Renderer::FrontFace::CounterClockWise);
  }
}

void GenericDrawable::updateShader() {
  const Mn::UnsignedInt lightCount = lightSetup_->size();
  const Mn::UnsignedInt jointCount =
      skinData_ ? skinData_->skinData->skin->joints().size() : 0;
  const Mn::UnsignedInt perVertexJointCount =
      skinData_ ? skinData_->skinData->perVertexJointCount : 0;

  if (skinData_) {
    Corrade::Containers::arrayResize(
        jointTransformations_, skinData_->skinData->skin->joints().size());
  }

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags_) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ =
        shaderManager_.get<Mn::GL::AbstractShaderProgram, Mn::Shaders::PhongGL>(
            getShaderKey(lightCount, flags_, jointCount));

    // if no shader with desired number of lights and flags exists, create one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(),
          new Mn::Shaders::PhongGL{
              Mn::Shaders::PhongGL::Configuration{}
                  .setFlags(flags_)
                  .setLightCount(lightCount)
                  .setJointCount(jointCount, perVertexJointCount)},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags_);
  }
}

Mn::ResourceKey GenericDrawable::getShaderKey(
    Mn::UnsignedInt lightCount,
    Mn::Shaders::PhongGL::Flags flags,
    Mn::UnsignedInt jointCount) const {
  return Corrade::Utility::formatString(
      SHADER_KEY_TEMPLATE, lightCount,
      static_cast<Mn::Shaders::PhongGL::Flags::UnderlyingType>(flags),
      jointCount);
}

}  // namespace gfx
}  // namespace esp
