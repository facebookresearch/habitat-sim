// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/PbrClearCoatMaterialData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>

#include <Magnum/GL/Renderer.h>

using Magnum::Math::Literals::operator""_radf;
namespace Mn = Magnum;

namespace esp {
namespace gfx {

PbrDrawable::PbrDrawable(scene::SceneNode& node,
                         Mn::GL::Mesh* mesh,
                         gfx::Drawable::Flags& meshAttributeFlags,
                         ShaderManager& shaderManager,
                         DrawableConfiguration& cfg)
    : Drawable{node, mesh, DrawableType::Pbr, cfg,
               shaderManager.get<LightSetup>(cfg.lightSetupKey_)},
      shaderManager_{shaderManager},
      pbrIbl_(std::move(cfg.getPbrIblData())),
      meshAttributeFlags_{meshAttributeFlags} {
  // Build material cache
  resetMaterialValues(
      shaderManager.get<Mn::Trade::MaterialData>(cfg.materialDataKey_));
  // Set shader config flags
  setShaderAttributesValues(cfg.getPbrShaderConfig());
  // update the shader early here to to avoid doing it during the render loop
  if (glMeshExists()) {
    updateShader();
  }
}

void PbrDrawable::setMaterialValuesInternal(
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
  PbrShader::Flags oldFlags(flags_);
  if (reset) {
    matCache = PBRMaterialCache();
  }
  flags_ = PbrShader::Flag::ObjectId;

  const auto& tmpMaterialData =
      materialData->as<Mn::Trade::PbrMetallicRoughnessMaterialData>();
  matCache.baseColor = tmpMaterialData.baseColor();
  matCache.roughness = tmpMaterialData.roughness();
  matCache.metalness = tmpMaterialData.metalness();
  matCache.emissiveColor = tmpMaterialData.emissiveColor();

  if (tmpMaterialData.commonTextureMatrix() != Mn::Matrix3{}) {
    flags_ |= PbrShader::Flag::TextureTransformation;
    matCache.textureMatrix = tmpMaterialData.commonTextureMatrix();
  }
  if (const auto baseColorTexturePtr =
          materialData->findAttribute<Mn::GL::Texture2D*>(
              "baseColorTexturePointer")) {
    flags_ |= PbrShader::Flag::BaseColorTexture;
    matCache.baseColorTexture = *baseColorTexturePtr;
  }

  if (const auto noneRoughMetalTexturePtr =
          materialData->findAttribute<Mn::GL::Texture2D*>(
              "noneRoughnessMetallicTexturePointer")) {
    flags_ |= PbrShader::Flag::NoneRoughnessMetallicTexture;
    matCache.noneRoughnessMetallicTexture = *noneRoughMetalTexturePtr;
  }

  if (const auto normalTexturePtr =
          materialData->findAttribute<Mn::GL::Texture2D*>(
              "normalTexturePointer")) {
    flags_ |= PbrShader::Flag::NormalTexture;
    matCache.normalTexture = *normalTexturePtr;
    if (meshAttributeFlags_ & gfx::Drawable::Flag::HasTangent) {
      flags_ |= PbrShader::Flag::PrecomputedTangent;
    }
    // normal texture scale
    matCache.normalTextureScale = tmpMaterialData.normalTextureScale();
  }

  if (const auto emissiveTexturePtr =
          materialData->findAttribute<Mn::GL::Texture2D*>(
              "emissiveTexturePointer")) {
    flags_ |= PbrShader::Flag::EmissiveTexture;
    matCache.emissiveTexture = *emissiveTexturePtr;
  }
  if (materialData->attribute<bool>("hasPerVertexObjectId")) {
    flags_ |= PbrShader::Flag::InstancedObjectId;
  }
  if (materialData->isDoubleSided()) {
    flags_ |= PbrShader::Flag::DoubleSided;
  }

  ////////////////
  // ClearCoat layer
  if (materialData->hasLayer(Mn::Trade::MaterialLayer::ClearCoat)) {
    const auto& ccLayer =
        materialData->as<Mn::Trade::PbrClearCoatMaterialData>();
    float cc_LayerFactor = ccLayer.layerFactor();
    // As per
    // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_clearcoat
    // if layer is 0 entire layer is disabled/ignored.
    if (cc_LayerFactor > 0.0f) {
      // has non-trivial clearcoat layer
      flags_ |= PbrShader::Flag::ClearCoatLayer;
      //
      matCache.clearCoat.factor = cc_LayerFactor;
      matCache.clearCoat.roughnessFactor = ccLayer.roughness();
      if (const auto layerTexturePtr =
              ccLayer.findAttribute<Mn::GL::Texture2D*>(
                  "layerFactorTexturePointer")) {
        flags_ |= PbrShader::Flag::ClearCoatTexture;
        matCache.clearCoat.texture = *layerTexturePtr;
      }

      if (const auto roughnessTexturePtr =
              ccLayer.findAttribute<Mn::GL::Texture2D*>(
                  "roughnessTexturePointer")) {
        flags_ |= PbrShader::Flag::ClearCoatRoughnessTexture;
        matCache.clearCoat.roughnessTexture = *roughnessTexturePtr;
      }

      if (const auto normalTexturePtr =
              ccLayer.findAttribute<Mn::GL::Texture2D*>(
                  "normalTexturePointer")) {
        flags_ |= PbrShader::Flag::ClearCoatNormalTexture;
        matCache.clearCoat.normalTexture = *normalTexturePtr;
        matCache.clearCoat.normalTextureScale = ccLayer.normalTextureScale();
      }

    }  // non-zero layer factor
  }    // has clearcoat layer

  ////////////////
  // KHR_materials_ior
  if (const auto iorLayerID = materialData->findLayerId("#KHR_materials_ior")) {
    // Read in custom material index of refraction
    if (const auto ior =
            materialData->findAttribute<Mn::Float>(*iorLayerID, "ior")) {
      // ior should be >= 1 or 0 (which gives full weight to specular layer
      // independent of view angle)
      matCache.ior_Index = *ior;
    }

  }  // has KHR_materials_ior layer

  ////////////////
  // KHR_materials_specular layer
  if (const auto specularLayerID =
          materialData->findLayerId("#KHR_materials_specular")) {
    flags_ |= PbrShader::Flag::SpecularLayer;
    /**
     * The strength of the specular reflection. Defaults to 1.0f
     */
    if (const auto specularFactor = materialData->findAttribute<Mn::Float>(
            *specularLayerID, "specularFactor")) {
      matCache.specularLayer.factor =
          Mn::Math::clamp(*specularFactor, 0.0f, 1.0f);
    }

    /**
     * A texture that defines the strength of the specular
     * reflection, stored in the alpha (A) channel. This will be
     * multiplied by specularFactor.
     */
    if (const auto specularFactorTexture =
            materialData->findAttribute<Mn::GL::Texture2D*>(
                *specularLayerID, "specularTexturePointer")) {
      flags_ |= PbrShader::Flag::SpecularLayerTexture;
      matCache.specularLayer.texture = *specularFactorTexture;
    }
    /**
     * The F0 color of the specular reflection (linear RGB).
     */
    if (const auto specularColorFactor =
            materialData->findAttribute<Mn::Color3>(*specularLayerID,
                                                    "specularColorFactor")) {
      matCache.specularLayer.colorFactor = *specularColorFactor;
    }
    /**
     * A texture that defines the F0 color of the specular
     * reflection, stored in the RGB channels and encoded in
     * sRGB. This texture will be multiplied by
     * specularColorFactor.
     */
    if (const auto specularColorTexture =
            materialData->findAttribute<Mn::GL::Texture2D*>(
                *specularLayerID, "specularColorTexturePointer")) {
      flags_ |= PbrShader::Flag::SpecularLayerColorTexture;
      matCache.specularLayer.colorTexture = *specularColorTexture;
    }
  }  // has KHR_materials_specular layer

  ///////////////
  // KHR_materials_anisotropy
  if (const auto anisotropyLayerID =
          materialData->findLayerId("#KHR_materials_anisotropy")) {
    /**
     * The anisotropy strength. When anisotropyTexture is present, this value is
     * multiplied by the blue channel. Default is 0.0f
     */
    if (const auto anisotropyStrength = materialData->findAttribute<Mn::Float>(
            *anisotropyLayerID, "anisotropyStrength")) {
      if (Mn::Math::abs(*anisotropyStrength) > 0.0f) {
        flags_ |= PbrShader::Flag::AnisotropyLayer;
        matCache.anisotropyLayer.factor =
            Mn::Math::clamp(*anisotropyStrength, -1.0f, 1.0f);
      }
      // Early adopters used anisotropy to mean strength
    } else if (const auto anisotropyStrength =
                   materialData->findAttribute<Mn::Float>(*anisotropyLayerID,
                                                          "anisotropy")) {
      if (Mn::Math::abs(*anisotropyStrength) > 0.0f) {
        flags_ |= PbrShader::Flag::AnisotropyLayer;
        matCache.anisotropyLayer.factor =
            Mn::Math::clamp(*anisotropyStrength, -1.0f, 1.0f);
      }
    }
    /**
     * The rotation of the anisotropy in tangent, bitangent space, measured in
     * radians counter-clockwise from the tangent. When anisotropyTexture is
     * present, anisotropyRotation provides additional rotation to the vectors
     * in the texture. Default is 0.0f
     */
    if (const auto anisotropyRotation = materialData->findAttribute<Mn::Float>(
            *anisotropyLayerID, "anisotropyRotation")) {
      if (*anisotropyRotation != 0.0f) {
        flags_ |= PbrShader::Flag::AnisotropyLayer;
        Mn::Rad rotAngle = Mn::Rad{*anisotropyRotation};
        matCache.anisotropyLayer.direction =
            Mn::Vector2{Mn::Complex::rotation(rotAngle)};
      }
      // Early adopters used anisotropyDirection
    } else if (const auto anisotropyRotation =
                   materialData->findAttribute<Mn::Float>(
                       *anisotropyLayerID, "anisotropyDirection")) {
      if (*anisotropyRotation != 0.0f) {
        flags_ |= PbrShader::Flag::AnisotropyLayer;
        Mn::Rad rotAngle = Mn::Rad{*anisotropyRotation};
        matCache.anisotropyLayer.direction =
            Mn::Vector2{Mn::Complex::rotation(rotAngle)};
      }
    }

    /**
     * A texture that defines the anisotropy of the material. Red and green
     * channels represent the anisotropy direction in [-1, 1] tangent,
     * bitangent space, to be rotated by anisotropyRotation. The blue
     * channel contains strength as [0, 1] to be multiplied by
     * anisotropyStrength.
     */
    if (const auto anisotropyLayerTexture =
            materialData->findAttribute<Mn::GL::Texture2D*>(
                *anisotropyLayerID, "anisotropyTexturePointer")) {
      // also covers flags_ |= PbrShader::Flag::AnisotropyLayer;
      flags_ |= PbrShader::Flag::AnisotropyLayerTexture;
      matCache.anisotropyLayer.texture = *anisotropyLayerTexture;
    }
  }  // has KHR_materials_anisotropy

  ////////////////
  // KHR_materials_transmission
  if (const auto transmissionLayerID =
          materialData->findLayerId("#KHR_materials_transmission")) {
    flags_ |= PbrShader::Flag::TransmissionLayer;
    // transmissionFactor
    if (const auto transmissionFactor = materialData->findAttribute<Mn::Float>(
            *transmissionLayerID, "transmissionFactor")) {
      matCache.transmissionLayer.factor = *transmissionFactor;
    }
    // transmissionTexturePointer
    if (const auto transmissionTexturePointer =
            materialData->findAttribute<Mn::GL::Texture2D*>(
                *transmissionLayerID, "transmissionTexturePointer")) {
      flags_ |= PbrShader::Flag::TransmissionLayerTexture;
      matCache.transmissionLayer.texture = *transmissionTexturePointer;
    }
  }  // has KHR_materials_transmission layer

  ////////////////
  // KHR_materials_volume
  if (const auto volumeLayerID =
          materialData->findLayerId("#KHR_materials_volume")) {
    flags_ |= PbrShader::Flag::VolumeLayer;

    if (const auto thicknessFactor = materialData->findAttribute<Mn::Float>(
            *volumeLayerID, "thicknessFactor")) {
      matCache.volumeLayer.thicknessFactor = *thicknessFactor;
    }

    if (const auto thicknessTexturePointer =
            materialData->findAttribute<Mn::GL::Texture2D*>(
                *volumeLayerID, "thicknessTexturePointer")) {
      flags_ |= PbrShader::Flag::VolumeLayerThicknessTexture;
      matCache.volumeLayer.thicknessTexture = *thicknessTexturePointer;
    }

    if (const auto attDist = materialData->findAttribute<Mn::Float>(
            *volumeLayerID, "attenuationDistance")) {
      if (*attDist > 0.0f) {
        // Can't be 0 or inf
        matCache.volumeLayer.attenuationDist = *attDist;
      }
    }

    if (const auto attenuationColor = materialData->findAttribute<Mn::Color3>(
            *volumeLayerID, "attenuationColor")) {
      matCache.volumeLayer.attenuationColor = *attenuationColor;
    }
  }  // has KHR_materials_volume layer
  // If not reset then make sure the same shader is used
  if (!reset) {
    flags_ = oldFlags;
  }

}  // PbrDrawable::setMaterialValuesInternal

void PbrDrawable::setShaderAttributesValues(
    const std::shared_ptr<metadata::attributes::PbrShaderAttributes>&
        pbrShaderConfig) {
  // If direct light is enabled
  pbrShaderConfig->getEnableDirectLighting()
      ? flags_ |= PbrShader::Flag::DirectLighting
      : flags_ &= ~PbrShader::Flag::DirectLighting;

  // If IBL is enabled and the pbrIbl_ helper exists
  (pbrShaderConfig->getEnableIBL() && pbrIbl_)
      ? flags_ |= PbrShader::Flag::ImageBasedLighting
      : flags_ &= ~PbrShader::Flag::ImageBasedLighting;

  // If using Burley/disney diffuse
  pbrShaderConfig->getUseBurleyDiffuse()
      ? flags_ |= PbrShader::Flag::UseBurleyDiffuse
      : flags_ &= ~PbrShader::Flag::UseBurleyDiffuse;

  // If should skip TBN Calculation
  pbrShaderConfig->getSkipCalcMissingTBN()
      ? flags_ |= PbrShader::Flag::SkipMissingTBNCalc
      : flags_ &= ~PbrShader::Flag::SkipMissingTBNCalc;

  // If should use Mikkelsen algorithm for TBN
  pbrShaderConfig->getUseMikkelsenTBN()
      ? flags_ |= PbrShader::Flag::UseMikkelsenTBN
      : flags_ &= ~PbrShader::Flag::UseMikkelsenTBN;

  // If should use tonemapping for direct lighting results
  pbrShaderConfig->getUseDirectLightTonemap()
      ? flags_ |= PbrShader::Flag::UseDirectLightTonemap
      : flags_ &= ~PbrShader::Flag::UseDirectLightTonemap;

  // If should use tonemapping for IBL results
  pbrShaderConfig->getUseIBLTonemap()
      ? flags_ |= PbrShader::Flag::UseIBLTonemap
      : flags_ &= ~PbrShader::Flag::UseIBLTonemap;

  // If clear coat calculations should be skipped
  pbrShaderConfig->getSkipCalcClearcoatLayer()
      ? flags_ |= PbrShader::Flag::SkipClearCoatLayer
      : flags_ &= ~PbrShader::Flag::SkipClearCoatLayer;

  // If specular layer calculations should be skipped
  pbrShaderConfig->getSkipCalcSpecularLayer()
      ? flags_ |= PbrShader::Flag::SkipSpecularLayer
      : flags_ &= ~PbrShader::Flag::SkipSpecularLayer;

  // If anisotropy layer calculations should be skipped
  pbrShaderConfig->getSkipCalcAnisotropyLayer()
      ? flags_ |= PbrShader::Flag::SkipAnisotropyLayer
      : flags_ &= ~PbrShader::Flag::SkipAnisotropyLayer;

  // If should use linear->sRGB remapping on appropriate material textures
  pbrShaderConfig->getMapMatTxtrToLinear()
      ? flags_ |= PbrShader::Flag::MapMatTxtrToLinear
      : flags_ &= ~PbrShader::Flag::MapMatTxtrToLinear;
  // If should use linear->sRGB remapping on IBL environment map
  pbrShaderConfig->getMapIBLTxtrToLinear()
      ? flags_ |= PbrShader::Flag::MapIBLTxtrToLinear
      : flags_ &= ~PbrShader::Flag::MapIBLTxtrToLinear;
  // If should use sRGB -> linear remapping on shader output
  pbrShaderConfig->getMapOutputToSRGB()
      ? flags_ |= PbrShader::Flag::MapOutputToSRGB
      : flags_ &= ~PbrShader::Flag::MapOutputToSRGB;

  // Only set values if actually going to use them
  if (flags_ >= PbrShader::Flag::DirectLighting) {
    // Intensity of direct lighting
    shaderConfig_.directLightingIntensity =
        pbrShaderConfig->getDirectLightIntensity();
    if (flags_ >= PbrShader::Flag::ImageBasedLighting) {
      // Scales contributions but only if both direct and IBL are being
      // processed.
      shaderConfig_.eqScales.directDiffuse =
          pbrShaderConfig->getDirectDiffuseScale();
      shaderConfig_.eqScales.directSpecular =
          pbrShaderConfig->getDirectSpecularScale();
      shaderConfig_.eqScales.iblDiffuse = pbrShaderConfig->getIBLDiffuseScale();
      shaderConfig_.eqScales.iblSpecular =
          pbrShaderConfig->getIBLSpecularScale();
    }
  }
  if (flags_ >= (PbrShader::Flag::UseIBLTonemap) ||
      flags_ >= (PbrShader::Flag::UseDirectLightTonemap)) {
    shaderConfig_.tonemapExposure = pbrShaderConfig->getTonemapExposure();
  }
  if (flags_ >= (PbrShader::Flag::MapMatTxtrToLinear) ||
      flags_ >= (PbrShader::Flag::MapIBLTxtrToLinear) ||
      flags_ >= (PbrShader::Flag::MapOutputToSRGB)) {
    float gamma = pbrShaderConfig->getGamma();
    shaderConfig_.gamma = Mn::Vector3{gamma, gamma, gamma};
  }
}  // PbrDrawable::setShaderAttributesValues

void PbrDrawable::setLightSetup(const Mn::ResourceKey& lightSetupKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(lightSetupKey);
  // update the shader early here to to avoid doing it during the render loop
  updateShader();
}

void PbrDrawable::draw(const Mn::Matrix4& transformationMatrix,
                       Mn::SceneGraph::Camera3D& camera) {
  CORRADE_ASSERT(glMeshExists(),
                 "PbrDrawable::draw() : GL mesh doesn't exist", );

  updateShader();
  // In Drawable.h
  // Pbr uses light position relative to world.
  updateShaderLightingParameters(transformationMatrix, camera, shader_,
                                 [](const LightInfo& lightInfo,
                                    const Magnum::Matrix4& transformationMatrix,
                                    const Magnum::Matrix4& cameraMatrix) {
                                   return getLightPositionRelativeToWorld(
                                       lightInfo, transformationMatrix,
                                       cameraMatrix);
                                 });

  // ABOUT PbrShader::Flag::DoubleSided:
  //
  // "Specifies whether the material is double sided. When this value is
  // false, back-face culling is enabled. When this value is true, back-face
  // culling is disabled and double sided lighting is enabled. The back-face
  // must have its normals reversed before the lighting equation is
  // evaluated." See here:
  // https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/schema/material.schema.json

  if ((flags_ >= PbrShader::Flag::DoubleSided) && glIsEnabled(GL_CULL_FACE)) {
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::FaceCulling);
  }

  Mn::Matrix4 modelMatrix =
      camera.cameraMatrix().inverted() * transformationMatrix;

  Mn::Matrix3x3 rotScale = modelMatrix.rotationScaling();
  // Find determinant to calculate backface culling winding dir
  const float normalDet = rotScale.determinant();
  // Normal matrix is calculated as `m.inverted().transposed()`, and
  // `m.inverted()` is the same as
  // `m.comatrix().transposed()/m.determinant()`. We need the determinant to
  // figure out the winding direction as well, thus we calculate it
  // separately and then do
  // `(m.comatrix().transposed()/determinant).transposed()`, which is the
  // same as `m.comatrix()/determinant`.
  Mn::Matrix3x3 normalMatrix = rotScale.comatrix() / normalDet;

  // Flip winding direction to correctly handle backface culling
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(Mn::GL::Renderer::FrontFace::ClockWise);
  }

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has
      // been uploaded to GPU so simply pass 0 to the uniform "objectId" in
      // the fragment shader
      .setObjectId(static_cast<RenderCamera&>(camera).useDrawableIds()
                       ? drawableId_
                       : (flags_ >= PbrShader::Flag::InstancedObjectId
                              ? 0
                              : node_.getSemanticId()))
      .setProjectionMatrix(camera.projectionMatrix())
      .setViewMatrix(camera.cameraMatrix())
      .setModelMatrix(modelMatrix)  // NOT modelview matrix!
      .setNormalMatrix(normalMatrix)
      .setCameraWorldPosition(
          camera.object().absoluteTransformationMatrix().translation())
      .setBaseColor(matCache.baseColor)
      .setRoughness(matCache.roughness)
      .setMetallic(matCache.metalness)
      .setIndexOfRefraction(matCache.ior_Index)
      .setEmissiveColor(matCache.emissiveColor);

  if (flags_ >= PbrShader::Flag::BaseColorTexture) {
    shader_->bindBaseColorTexture(*matCache.baseColorTexture);
  }

  if (flags_ >= PbrShader::Flag::NoneRoughnessMetallicTexture) {
    shader_->bindMetallicRoughnessTexture(
        *matCache.noneRoughnessMetallicTexture);
  }

  if (flags_ >= PbrShader::Flag::NormalTexture) {
    shader_->bindNormalTexture(*matCache.normalTexture);
    shader_->setNormalTextureScale(matCache.normalTextureScale);
  }

  if (flags_ >= PbrShader::Flag::EmissiveTexture) {
    shader_->bindEmissiveTexture(*matCache.emissiveTexture);
  }

  if (flags_ >= PbrShader::Flag::TextureTransformation) {
    shader_->setTextureMatrix(matCache.textureMatrix);
  }

  // clearcoat data
  if (flags_ >= PbrShader::Flag::ClearCoatLayer) {
    (*shader_)
        .setClearCoatFactor(matCache.clearCoat.factor)
        .setClearCoatRoughness(matCache.clearCoat.roughnessFactor);
    if (flags_ >= PbrShader::Flag::ClearCoatTexture) {
      shader_->bindClearCoatFactorTexture(*matCache.clearCoat.texture);
    }
    if (flags_ >= PbrShader::Flag::ClearCoatRoughnessTexture) {
      shader_->bindClearCoatRoughnessTexture(
          *matCache.clearCoat.roughnessTexture);
    }
    if (flags_ >= PbrShader::Flag::ClearCoatNormalTexture) {
      (*shader_)
          .setClearCoatNormalTextureScale(matCache.clearCoat.normalTextureScale)
          .bindClearCoatNormalTexture(*matCache.clearCoat.normalTexture);
    }
  }

  // specular layer data
  if (flags_ >= PbrShader::Flag::SpecularLayer) {
    (*shader_)
        .setSpecularLayerFactor(matCache.specularLayer.factor)
        .setSpecularLayerColorFactor(matCache.specularLayer.colorFactor);

    if (flags_ >= PbrShader::Flag::SpecularLayerTexture) {
      shader_->bindSpecularLayerTexture(*matCache.specularLayer.texture);
    }
    if (flags_ >= PbrShader::Flag::SpecularLayerColorTexture) {
      shader_->bindSpecularLayerColorTexture(
          *matCache.specularLayer.colorTexture);
    }
  }

  // anisotropy layer data
  if (flags_ >= PbrShader::Flag::AnisotropyLayer) {
    (*shader_)
        .setAnisotropyLayerFactor(matCache.anisotropyLayer.factor)
        .setAnisotropyLayerDirection(matCache.anisotropyLayer.direction);

    if (flags_ >= PbrShader::Flag::AnisotropyLayerTexture) {
      shader_->bindAnisotropyLayerTexture(*matCache.anisotropyLayer.texture);
    }
  }

  // Set gamma value to use for srgb remapping if being used
  // Setter does appropriate checking
  shader_->setGamma(shaderConfig_.gamma);

  // Tonemap exposure
  if (flags_ >= (PbrShader::Flag::UseIBLTonemap) ||
      flags_ >= (PbrShader::Flag::UseDirectLightTonemap)) {
    shader_->setTonemapExposure(shaderConfig_.tonemapExposure);
  }
  if (flags_ >= PbrShader::Flag::DirectLighting) {
    // Intensity of direct lighting
    shader_->setDirectLightIntensity(shaderConfig_.directLightingIntensity);
    if (flags_ >= PbrShader::Flag::ImageBasedLighting) {
      shader_->setPbrEquationScales(shaderConfig_.eqScales);
    }
  }

  // setup image based lighting for the shader
  if (flags_ >= PbrShader::Flag::ImageBasedLighting) {
    CORRADE_INTERNAL_ASSERT(pbrIbl_);
    shader_->bindIrradianceCubeMap(
        pbrIbl_->getIrradianceMap().getTexture(CubeMap::TextureType::Color));
    shader_->bindBrdfLUT(pbrIbl_->getBrdfLookupTable());
    shader_->bindPrefilteredMap(
        pbrIbl_->getPrefilteredMap().getTexture(CubeMap::TextureType::Color));
    shader_->setPrefilteredMapMipLevels(
        pbrIbl_->getPrefilteredMap().getMipmapLevels());
  }

  if (skinData_) {
    buildSkinJointTransforms();
    shader_->setJointMatrices(jointTransformations_);
  }

  shader_->draw(getMesh());

  // Reset winding direction
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(
        Mn::GL::Renderer::FrontFace::CounterClockWise);
  }

  if ((flags_ >= PbrShader::Flag::DoubleSided) && !glIsEnabled(GL_CULL_FACE)) {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  }

}  // PbrDrawable::draw

void PbrDrawable::updateShader() {
  const Mn::UnsignedInt lightCount = lightSetup_->size();
  const Mn::UnsignedInt jointCount =
      skinData_ ? skinData_->skinData->skin->joints().size() : 0;
  const Mn::UnsignedInt perVertexJointCount =
      skinData_ ? skinData_->skinData->perVertexJointCount : 0;

  if (skinData_) {
    resizeJointTransformArray(jointCount);
  }

  if (!shader_ || shader_->lightCount() != lightCount ||
      shader_->flags() != flags_) {
    // if the number of lights or flags have changed, we need to fetch a
    // compatible shader
    shader_ = shaderManager_.get<Mn::GL::AbstractShaderProgram, PbrShader>(
        getShaderKey("PBR", lightCount,
                     static_cast<PbrShader::Flags::UnderlyingType>(flags_),
                     jointCount));

    // if no shader with desired number of lights and flags exists, create
    // one
    if (!shader_) {
      shaderManager_.set<Mn::GL::AbstractShaderProgram>(
          shader_.key(),
          new PbrShader{flags_, lightCount, jointCount, perVertexJointCount},
          Mn::ResourceDataState::Final, Mn::ResourcePolicy::ReferenceCounted);
    }

    CORRADE_INTERNAL_ASSERT(shader_ && shader_->lightCount() == lightCount &&
                            shader_->flags() == flags_);
  }
}  // PbrDrawable::updateShader

}  // namespace gfx
}  // namespace esp
