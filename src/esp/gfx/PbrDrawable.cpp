// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrDrawable.h"

#include <Corrade/Containers/ArrayViewStl.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Trade/MaterialData.h>
#include <Magnum/Trade/PbrClearCoatMaterialData.h>
#include <Magnum/Trade/PbrMetallicRoughnessMaterialData.h>

#include <Magnum/GL/Renderer.h>

namespace Mn = Magnum;

namespace esp {
namespace gfx {

PbrDrawable::PbrDrawable(scene::SceneNode& node,
                         Mn::GL::Mesh* mesh,
                         gfx::Drawable::Flags& meshAttributeFlags,
                         ShaderManager& shaderManager,
                         const Mn::ResourceKey& lightSetupKey,
                         const Mn::ResourceKey& materialDataKey,
                         DrawableGroup* group,
                         PbrImageBasedLighting* pbrIbl)
    : Drawable{node, mesh, DrawableType::Pbr, group},
      shaderManager_{shaderManager},
      lightSetup_{shaderManager.get<LightSetup>(lightSetupKey)},
      meshAttributeFlags_{meshAttributeFlags},
      pbrIbl_(pbrIbl) {
  setMaterialValuesInternal(
      shaderManager.get<Mn::Trade::MaterialData>(materialDataKey));

  if (pbrIbl_) {
    flags_ |= PbrShader::Flag::ImageBasedLighting;
  }

  // Defer the shader initialization because at this point, the lightSetup may
  // not be done in the Simulator. Simulator itself is currently under
  // construction in this case.
  // updateShader().updateShaderLightParameters();
}

void PbrDrawable::setMaterialValuesInternal(
    const Mn::Resource<Mn::Trade::MaterialData, Mn::Trade::MaterialData>&
        material) {
  materialData_ = material;

  const auto& tmpMaterialData =
      materialData_->as<Mn::Trade::PbrMetallicRoughnessMaterialData>();
  flags_ = PbrShader::Flag::ObjectId;

  matCache.baseColor = tmpMaterialData.baseColor();
  matCache.roughness = tmpMaterialData.roughness();
  matCache.metalness = tmpMaterialData.metalness();
  matCache.emissiveColor = tmpMaterialData.emissiveColor();

  if (materialData_->hasAttribute("metallicTexturePointer") &&
      materialData_->hasAttribute("roughnessTexturePointer")) {
    CORRADE_ASSERT(
        materialData_->attribute<Mn::GL::Texture2D*>(
            "metallicTexturePointer") ==
            materialData_->attribute<Mn::GL::Texture2D*>(
                "roughnessTexturePointer"),
        "PbrDrawable::setMaterialValuesInternal(): if both the metallic and "
        "roughness "
        "texture exist, they must be packed in the same texture based on glTF "
        "2.0 Spec.", );
  }
  if (tmpMaterialData.commonTextureMatrix() != Mn::Matrix3{}) {
    flags_ |= PbrShader::Flag::TextureTransformation;
    matCache.textureMatrix = tmpMaterialData.commonTextureMatrix();
  }
  if (materialData_->hasAttribute("baseColorTexturePointer")) {
    flags_ |= PbrShader::Flag::BaseColorTexture;
    matCache.baseColorTexture =
        materialData_->attribute<Mn::GL::Texture2D*>("baseColorTexturePointer");
  }

  matCache.hasAnyMetallicRoughnessTexture = false;
  matCache.useMetallicRoughnessTexture = nullptr;
  // noneRoughnessMetallic takes precedence, but currently all are
  // treated the same way
  if (materialData_->hasAttribute("noneRoughnessMetallicTexturePointer")) {
    flags_ |= PbrShader::Flag::NoneRoughnessMetallicTexture;
    matCache.noneRoughnessMetallicTexture =
        materialData_->attribute<Mn::GL::Texture2D*>(
            "noneRoughnessMetallicTexturePointer");
    matCache.hasAnyMetallicRoughnessTexture = true;
    matCache.useMetallicRoughnessTexture =
        matCache.noneRoughnessMetallicTexture;
  }
  if (materialData_->hasAttribute("roughnessTexturePointer")) {
    flags_ |= PbrShader::Flag::RoughnessTexture;
    matCache.roughnessTexture =
        materialData_->attribute<Mn::GL::Texture2D*>("roughnessTexturePointer");
    if (!matCache.hasAnyMetallicRoughnessTexture) {
      matCache.useMetallicRoughnessTexture = matCache.roughnessTexture;
    }
    matCache.hasAnyMetallicRoughnessTexture = true;
  }
  if (materialData_->hasAttribute("metallicTexturePointer")) {
    flags_ |= PbrShader::Flag::MetallicTexture;
    matCache.metallicTexture =
        materialData_->attribute<Mn::GL::Texture2D*>("metallicTexturePointer");

    if (!matCache.hasAnyMetallicRoughnessTexture) {
      matCache.useMetallicRoughnessTexture = matCache.metallicTexture;
    }
    matCache.hasAnyMetallicRoughnessTexture = true;
  }

  CORRADE_ASSERT(((matCache.useMetallicRoughnessTexture != nullptr) ==
                  matCache.hasAnyMetallicRoughnessTexture),
                 "PbrDrawable::setMaterialValuesInternal(): Error assigning "
                 "proper Metallic/Roughness texture pointers - either a "
                 "texture is expected but not present or vice versa.", );

  if (materialData_->hasAttribute("normalTexturePointer")) {
    flags_ |= PbrShader::Flag::NormalTexture;
    matCache.normalTexture =
        materialData_->attribute<Mn::GL::Texture2D*>("normalTexturePointer");
    if (meshAttributeFlags_ & gfx::Drawable::Flag::HasTangent) {
      flags_ |= PbrShader::Flag::PrecomputedTangent;
    }
    if (tmpMaterialData.normalTextureScale() != 1.0f) {
      flags_ |= PbrShader::Flag::NormalTextureScale;
      matCache.normalTextureScale = tmpMaterialData.normalTextureScale();
      CORRADE_ASSERT(tmpMaterialData.normalTextureScale() > 0.0f,
                     "PbrDrawable::PbrDrawable(): the normal texture scale "
                     "must be positive.", );
    }
  }
  if (materialData_->hasAttribute("emissiveTexturePointer")) {
    flags_ |= PbrShader::Flag::EmissiveTexture;
    matCache.emissiveTexture =
        materialData_->attribute<Mn::GL::Texture2D*>("emissiveTexturePointer");
  }
  if (materialData_->attribute<bool>("hasPerVertexObjectId")) {
    flags_ |= PbrShader::Flag::InstancedObjectId;
  }
  if (materialData_->isDoubleSided()) {
    flags_ |= PbrShader::Flag::DoubleSided;
  }

  std::string debugStr = "";
  ////////////////
  // ClearCoat layer
  if (materialData_->hasLayer(Mn::Trade::MaterialLayer::ClearCoat)) {
    const auto& ccLayer =
        materialData_->as<Mn::Trade::PbrClearCoatMaterialData>();
    float cc_LayerFactor = ccLayer.layerFactor();
    if (cc_LayerFactor > 0.0f) {
      // has non-trivial clearcoat layer
      flags_ |= PbrShader::Flag::ClearCoatLayer;
      //
      matCache.cc_ClearCoatFactor = cc_LayerFactor;
      matCache.cc_Roughness = ccLayer.roughness();

      if (ccLayer.hasAttribute("layerFactorTexturePointer")) {
        flags_ |= PbrShader::Flag::CCLayer_CCTexture;
        matCache.cc_ClearCoatTexture =
            ccLayer.attribute<Mn::GL::Texture2D*>("layerFactorTexturePointer");
      }

      if (ccLayer.hasAttribute("roughnessTexturePointer")) {
        flags_ |= PbrShader::Flag::CCLayer_RoughnessTexture;
        matCache.cc_RoughnessTexture =
            ccLayer.attribute<Mn::GL::Texture2D*>("roughnessTexturePointer");
        matCache.cc_Roughness_Texture_Swizzle =
            ccLayer.roughnessTextureSwizzle();
      }

      if (ccLayer.hasAttribute("normalTexturePointer")) {
        flags_ |= PbrShader::Flag::CCLayer_NormalTexture;
        matCache.cc_NormalTexture =
            ccLayer.attribute<Mn::GL::Texture2D*>("normalTexturePointer");
        // TODO : do we really need to verify if scale
        matCache.cc_NormalTextureScale = ccLayer.normalTextureScale();
      }
      Cr::Utility::formatInto(
          debugStr, debugStr.size(),
          " ClearCoat layer with factor : {} Roughness : {}", cc_LayerFactor,
          matCache.cc_Roughness);

    }  // non-zero layer factor
  }    // has clearcoat layer

  ////////////////
  // KHR_materials_ior
  if (materialData_->hasLayer("#KHR_materials_ior")) {
    // Read in custom material index of refraction
    if (materialData_->hasAttribute("#KHR_materials_ior", "ior")) {
      matCache.ior_Index =
          materialData_->attribute<float>("#KHR_materials_ior", "ior");
      Cr::Utility::formatInto(debugStr, debugStr.size(),
                              " | IOR layer w/IOR = {}", matCache.ior_Index);
    }

  }  // has KHR_materials_ior layer

  ////////////////
  // KHR_materials_specular layer
  if (materialData_->hasLayer("#KHR_materials_specular")) {
    flags_ |= PbrShader::Flag::SpecularLayer;
    /**
     * The strength of the specular reflection. Defaults to 1.0f
     */
    if (materialData_->hasAttribute("#KHR_materials_specular",
                                    "specularFactor")) {
      matCache.spec_SpecularFactor = materialData_->attribute<float>(
          "#KHR_materials_specular", "specularFactor");
    }

    /**
     * A texture that defines the strength of the specular reflection, stored in
     * the alpha (A) channel. This will be multiplied by specularFactor.
     */
    if (materialData_->hasAttribute("#KHR_materials_specular",
                                    "specularTexturePointer")) {
      flags_ |= PbrShader::Flag::SpecLayer_SpecTexture;
      matCache.spec_SpecularTexture =
          materialData_->attribute<Mn::GL::Texture2D*>(
              "#KHR_materials_specular", "specularTexturePointer");
    }
    /**
     * The F0 color of the specular reflection (linear RGB).
     */
    if (materialData_->hasAttribute("#KHR_materials_specular",
                                    "specularColorFactor")) {
      matCache.spec_SpecularColorFactor = materialData_->attribute<Mn::Color3>(
          "#KHR_materials_specular", "specularColorFactor");
    }
    /**
     * A texture that defines the F0 color of the specular reflection,
     * stored in the RGB channels and encoded in sRGB. This texture will be
     * multiplied by specularColorFactor.
     */
    if (materialData_->hasAttribute("#KHR_materials_specular",
                                    "specularColorTexturePointer")) {
      flags_ |= PbrShader::Flag::SpecLayer_SpecColorTexture;
      matCache.spec_SpecularColorTexture =
          materialData_->attribute<Mn::GL::Texture2D*>(
              "#KHR_materials_specular", "specularColorTexturePointer");
    }

    Cr::Utility::formatInto(
        debugStr, debugStr.size(),
        " | Specular layer with factor : {} spec clr : [{},{}.{}]",
        matCache.spec_SpecularFactor, matCache.spec_SpecularColorFactor.r(),
        matCache.spec_SpecularColorFactor.g(),
        matCache.spec_SpecularColorFactor.b());
  }  // has KHR_materials_specular layer

  ////////////////
  // KHR_materials_transmission
  if (materialData_->hasLayer("#KHR_materials_transmission")) {
    flags_ |= PbrShader::Flag::TransmissionLayer;
    // transmissionFactor
    if (materialData_->hasAttribute("#KHR_materials_transmission",
                                    "transmissionFactor")) {
      matCache.trns_TransmissionFactor = materialData_->attribute<float>(
          "#KHR_materials_transmission", "transmissionFactor");
    }
    // transmissionTexturePointer

    if (materialData_->hasAttribute("#KHR_materials_transmission",
                                    "transmissionTexturePointer")) {
      flags_ |= PbrShader::Flag::TransLayer_TransmissionTexture;
      matCache.trns_TransmissionTexture =
          materialData_->attribute<Mn::GL::Texture2D*>(
              "#KHR_materials_transmission", "transmissionTexturePointer");
    }

    Cr::Utility::formatInto(debugStr, debugStr.size(), " | Transmission layer");
  }  // has KHR_materials_transmission layer
  ////////////////
  // KHR_materials_volume
  if (materialData_->hasLayer("#KHR_materials_volume")) {
    Cr::Utility::formatInto(debugStr, debugStr.size(), " | Volume layer");
    flags_ |= PbrShader::Flag::VolumeLayer;

    if (materialData_->hasAttribute("#KHR_materials_volume",
                                    "thicknessFactor")) {
      matCache.vol_ThicknessFactor = materialData_->attribute<float>(
          "#KHR_materials_volume", "thicknessFactor");
    }

    if (materialData_->hasAttribute("#KHR_materials_volume",
                                    "thicknessTexturePointer")) {
      flags_ |= PbrShader::Flag::VolLayer_ThicknessTexture;
      matCache.vol_ThicknessTexture =
          materialData_->attribute<Mn::GL::Texture2D*>(
              "#KHR_materials_volume", "thicknessTexturePointer");
    }

    if (materialData_->hasAttribute("#KHR_materials_volume",
                                    "attenuationDistance")) {
      float attDist = materialData_->attribute<float>("#KHR_materials_volume",
                                                      "attenuationDistance");
      if (attDist > 0.0f) {
        // Can't be 0 or inf
        matCache.vol_AttenuationDist = attDist;
      }
    }

    if (materialData_->hasAttribute("#KHR_materials_volume",
                                    "attenuationColor")) {
      matCache.vol_AttenuationColor = materialData_->attribute<Mn::Color3>(
          "#KHR_materials_volume", "attenuationColor");
    }
  }  // has KHR_materials_volume layer
  if (debugStr.length() > 0) {
    ESP_WARNING() << "PBR Material:" << debugStr;
  }

}  // PbrDrawable::setMaterialValuesInternal

void PbrDrawable::setLightSetup(const Mn::ResourceKey& lightSetupKey) {
  lightSetup_ = shaderManager_.get<LightSetup>(lightSetupKey);
}

void PbrDrawable::draw(const Mn::Matrix4& transformationMatrix,
                       Mn::SceneGraph::Camera3D& camera) {
  CORRADE_ASSERT(glMeshExists(),
                 "PbrDrawable::draw() : GL mesh doesn't exist", );

  updateShader()
      .updateShaderLightParameters()
      .updateShaderLightDirectionParameters(transformationMatrix, camera);

  // ABOUT PbrShader::Flag::DoubleSided:
  //
  // "Specifies whether the material is double sided. When this value is
  // false, back-face culling is enabled. When this value is true, back-face
  // culling is disabled and double sided lighting is enabled. The back-face
  // must have its normals reversed before the lighting equation is
  // evaluated." See here:
  // https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/schema/material.schema.json

  // HOWEVER, WE CANNOT DISABLE BACK FACE CULLING (that is why the following
  // code is commented out) since it causes lighting artifacts ("dashed
  // lines") on hard edges. (maybe due to potential numerical issues? we do
  // not know yet.)
  /*
  if ((flags_ & PbrShader::Flag::DoubleSided) && glIsEnabled(GL_CULL_FACE)) {
    Mn::GL::Renderer::disable(Mn::GL::Renderer::Feature::FaceCulling);
  }
  */
  Mn::Matrix4 modelMatrix =
      camera.cameraMatrix().inverted() * transformationMatrix;

  Mn::Matrix3x3 rotScale = modelMatrix.rotationScaling();
  // Find determinant to calculate backface culling winding dir
  const float normalDet = rotScale.determinant();
  // Normal matrix is calculated as `m.inverted().transposed()`, and
  // `m.inverted()` is the same as
  // `m.comatrix().transposed()/m.determinant()`. We need the determinant to
  // figure out the winding direction as well, thus we calculate it separately
  // and then do
  // `(m.comatrix().transposed()/determinant).transposed()`, which is the same
  // as `m.comatrix()/determinant`.
  Mn::Matrix3x3 normalMatrix = rotScale.comatrix() / normalDet;

  // Flip winding direction to correct handle backface culling
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(Mn::GL::Renderer::FrontFace::ClockWise);
  }

  (*shader_)
      // e.g., semantic mesh has its own per vertex annotation, which has
      // been uploaded to GPU so simply pass 0 to the uniform "objectId" in
      // the fragment shader
      .setObjectId(static_cast<RenderCamera&>(camera).useDrawableIds()
                       ? drawableId_
                       : ((flags_ & PbrShader::Flag::InstancedObjectId) ==
                                  PbrShader::Flag::InstancedObjectId
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
      .setEmissiveColor(matCache.emissiveColor);

  // TODO:
  // IN PbrShader class, we set the resonable defaults for the
  // PbrShader::PbrEquationScales. Here we need a smart way to reset it
  // just in case user would like to do so during the run-time.

  if (flags_ & PbrShader::Flag::BaseColorTexture) {
    shader_->bindBaseColorTexture(*matCache.baseColorTexture);
  }

  if (matCache.hasAnyMetallicRoughnessTexture) {
    shader_->bindMetallicRoughnessTexture(
        *matCache.useMetallicRoughnessTexture);
  }

  if (flags_ & PbrShader::Flag::NormalTexture) {
    shader_->bindNormalTexture(*matCache.normalTexture);
  }

  if (flags_ & PbrShader::Flag::EmissiveTexture) {
    shader_->bindEmissiveTexture(*matCache.emissiveTexture);
  }

  if (flags_ & PbrShader::Flag::TextureTransformation) {
    shader_->setTextureMatrix(matCache.textureMatrix);
  }

  // setup image based lighting for the shader
  if (flags_ & PbrShader::Flag::ImageBasedLighting) {
    CORRADE_INTERNAL_ASSERT(pbrIbl_);
    shader_->bindIrradianceCubeMap(  // TODO: HDR Color
        pbrIbl_->getIrradianceMap().getTexture(CubeMap::TextureType::Color));
    shader_->bindBrdfLUT(pbrIbl_->getBrdfLookupTable());
    shader_->bindPrefilteredMap(
        // TODO: HDR Color
        pbrIbl_->getPrefilteredMap().getTexture(CubeMap::TextureType::Color));
    shader_->setPrefilteredMapMipLevels(
        pbrIbl_->getPrefilteredMap().getMipmapLevels());
  }

  if (flags_ & PbrShader::Flag::ShadowsVSM) {
    CORRADE_INTERNAL_ASSERT(shadowMapManger_ && shadowMapKeys_);
    CORRADE_ASSERT(shadowMapKeys_->size() <= 3,
                   "PbrDrawable::draw: the number of shadow maps exceeds the "
                   "maximum (current it is 3).", );
    for (int iShadow = 0; iShadow < shadowMapKeys_->size(); ++iShadow) {
      Mn::Resource<CubeMap> shadowMap =
          (*shadowMapManger_).get<CubeMap>((*shadowMapKeys_)[iShadow]);

      CORRADE_INTERNAL_ASSERT(shadowMap);

      if (flags_ & PbrShader::Flag::ShadowsVSM) {
        shader_->bindPointShadowMap(
            iShadow,
            shadowMap->getTexture(CubeMap::TextureType::VarianceShadowMap));
      }
    }
  }

  shader_->draw(getMesh());

  // Reset winding direction
  if (normalDet < 0) {
    Mn::GL::Renderer::setFrontFace(
        Mn::GL::Renderer::FrontFace::CounterClockWise);
  }

  // WE stopped supporting doubleSided material due to lighting artifacts on
  // hard edges. See comments at the beginning of this function.
  /*
  if ((flags_ & PbrShader::Flag::DoubleSided) && !glIsEnabled(GL_CULL_FACE)) {
    Mn::GL::Renderer::enable(Mn::GL::Renderer::Feature::FaceCulling);
  }
  */
}  // namespace gfx

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

// update light direction (or position) in *world* space to the shader
PbrDrawable& PbrDrawable::updateShaderLightDirectionParameters(
    const Mn::Matrix4& transformationMatrix,
    Mn::SceneGraph::Camera3D& camera) {
  std::vector<Mn::Vector4> lightPositions;
  lightPositions.reserve(lightSetup_->size());

  const Mn::Matrix4 cameraMatrix = camera.cameraMatrix();
  for (unsigned int iLight = 0; iLight < lightSetup_->size(); ++iLight) {
    const auto& lightInfo = (*lightSetup_)[iLight];
    Mn::Vector4 pos = getLightPositionRelativeToWorld(
        lightInfo, transformationMatrix, cameraMatrix);
    // flip directional lights to faciliate faster, non-forking calc in
    // shader.  Leave non-directional lights unchanged
    pos *= (pos[3] * 2) - 1;
    lightPositions.emplace_back(pos);
  }

  shader_->setLightVectors(lightPositions);

  return *this;
}

void PbrDrawable::setShadowData(ShadowMapManager& manager,
                                ShadowMapKeys& keys,
                                PbrShader::Flag shadowFlag) {
  // sanity check first
  CORRADE_ASSERT(shadowFlag == PbrShader::Flag::ShadowsVSM,
                 "PbrDrawable::setShadowData(): the shadow flag can only be "
                 "ShadowsVSM.", );

  shadowMapManger_ = &manager;
  shadowMapKeys_ = &keys;
  flags_ |= shadowFlag;
}

}  // namespace gfx
}  // namespace esp
