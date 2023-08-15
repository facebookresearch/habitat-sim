// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrShaderAttributesManager.h"
#include "AttributesManagerBase.h"

#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace esp {

namespace metadata {

using attributes::PbrShaderAttributes;
namespace managers {

PbrShaderAttributes::ptr PbrShaderAttributesManager::createObject(
    const std::string& pbrConfigFilename,
    bool registerTemplate) {
  std::string msg;
  PbrShaderAttributes::ptr attrs = this->createFromJsonOrDefaultInternal(
      pbrConfigFilename, msg, registerTemplate);

  if (nullptr != attrs) {
    ESP_DEBUG() << msg << "pbr shader configuration created"
                << (registerTemplate ? "and registered." : ".");
  }
  return attrs;
}  // PbrShaderAttributesManager::createObject

void PbrShaderAttributesManager::setValsFromJSONDoc(
    PbrShaderAttributes::ptr pbrShaderAttribs,
    const io::JsonGenericValue& jsonConfig) {
  ////////////////////////////
  // Direct lighting calculation settings
  // whether direct lighting should be enabled
  io::jsonIntoSetter<bool>(
      jsonConfig, "enable_direct_lights",
      [pbrShaderAttribs](bool enableLights) {
        pbrShaderAttribs->setEnableDirectLighting(enableLights);
      });
  // direct light intensity scale
  io::jsonIntoSetter<double>(
      jsonConfig, "direct_light_intensity",
      [pbrShaderAttribs](double lightIntensity) {
        pbrShaderAttribs->setDirectLightIntensity(lightIntensity);
      });
  // whether we use the burley/disney diffuse calculation or lambertian diffuse
  // calculation for direct lighting.
  io::jsonIntoSetter<bool>(
      jsonConfig, "use_burley_diffuse", [pbrShaderAttribs](bool useLambertian) {
        pbrShaderAttribs->setUseBurleyDiffuse(useLambertian);
      });

  // if TBN frame should be calculated if no precomputed tangent is present
  io::jsonIntoSetter<bool>(
      jsonConfig, "skip_missing_tbn_calc",
      [pbrShaderAttribs](bool calcMissingTBN) {
        pbrShaderAttribs->setSkipCalcMissingTBN(calcMissingTBN);
      });

  // whether the Mikkelsen method should be used for the TBN calculation, or
  // if a simplified calculation that seems to give equivalent results should be
  // used.
  io::jsonIntoSetter<bool>(
      jsonConfig, "use_mikkelsen_tbn",
      [pbrShaderAttribs](bool useMikkelsenTBN) {
        pbrShaderAttribs->setUseMikkelsenTBN(useMikkelsenTBN);
      });

  // whether tonemapping should be used for direct lighting
  io::jsonIntoSetter<bool>(
      jsonConfig, "use_direct_tonemap",
      [pbrShaderAttribs](bool useDirectTonemap) {
        pbrShaderAttribs->setUseDirectLightTonemap(useDirectTonemap);
      });

  // direct light diffuse contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "direct_diffuse_scale",
                             [pbrShaderAttribs](double scale) {
                               pbrShaderAttribs->setDirectDiffuseScale(scale);
                             });

  // direct light specular contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "direct_specular_scale",
                             [pbrShaderAttribs](double scale) {
                               pbrShaderAttribs->setDirectSpecularScale(scale);
                             });

  ////////////////////////////
  // Material Layer calculation settings

  // whether clearcoat layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoSetter<bool>(
      jsonConfig, "skip_clearcoat_calc",
      [pbrShaderAttribs](bool skipCalcClearCoat) {
        pbrShaderAttribs->setSkipCalcClearcoatLayer(skipCalcClearCoat);
      });

  // whether speecular layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoSetter<bool>(
      jsonConfig, "skip_specular_layer_calc",
      [pbrShaderAttribs](bool skipCalcSpecular) {
        pbrShaderAttribs->setSkipCalcSpecularLayer(skipCalcSpecular);
      });

  // whether anisotropy layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoSetter<bool>(
      jsonConfig, "skip_anisotropy_layer_calc",
      [pbrShaderAttribs](bool skipCalcAnisotropy) {
        pbrShaderAttribs->setSkipCalcAnisotropyLayer(skipCalcAnisotropy);
      });

  ////////////////////
  // IBL-specific quantities
  // whether image-based lighting should be used in PBR shader
  io::jsonIntoSetter<bool>(jsonConfig, "enable_ibl",
                           [pbrShaderAttribs](bool enableIBL) {
                             pbrShaderAttribs->setEnableIBL(enableIBL);
                           });

  // the filename for the brdf lookup table used by the IBL calculations. If
  // empty retain default, which should always be in resource file.
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "ibl_blut_filename",
      [pbrShaderAttribs](const std::string& brdfLUTAsset) {
        // If empty retain default filename
        if (!brdfLUTAsset.empty()) {
          pbrShaderAttribs->setIBLBrdfLUTAssetHandle(brdfLUTAsset);
        }
      });

  // the filename for the equirectangular environment map used by the IBL
  // calculations. If empty retain default, which should always be in resource
  // file.
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "ibl_envmap_filename",
      [pbrShaderAttribs](const std::string& envMapAsset) {
        // If empty retain default filename
        if (!envMapAsset.empty()) {
          pbrShaderAttribs->setIBLEnvMapAssetHandle(envMapAsset);
        }
      });

  // whether tonemapping should be used for IBL lighting calculations
  io::jsonIntoSetter<bool>(jsonConfig, "use_ibl_tonemap",
                           [pbrShaderAttribs](bool useIBLTonemap) {
                             pbrShaderAttribs->setUseIBLTonemap(useIBLTonemap);
                           });

  // IBL diffuse contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "ibl_diffuse_scale",
                             [pbrShaderAttribs](double scale) {
                               pbrShaderAttribs->setIBLDiffuseScale(scale);
                             });

  // IBL specular contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "ibl_specular_scale",
                             [pbrShaderAttribs](double scale) {
                               pbrShaderAttribs->setIBLSpecularScale(scale);
                             });

  ///////////////////
  // Both Direct and IBL-related quantities

  // tonemap exposure setting. This value scales the linear color before
  // tonemapping.
  io::jsonIntoSetter<double>(jsonConfig, "tonemap_exposure",
                             [pbrShaderAttribs](double exposure) {
                               pbrShaderAttribs->setTonemapExposure(exposure);
                             });

  // whether the approximation sRGB->linear mapping should be used in the
  // shader on the appropriate textures (basecolor, emissivecolor,
  // specularlayercolor) as described by GLTF standard. This will not be needed
  // once the textures in question are converted on load.
  io::jsonIntoSetter<bool>(
      jsonConfig, "map_mat_txtr_to_linear",
      [pbrShaderAttribs](bool useSRGBRemapping) {
        pbrShaderAttribs->setMapMatTxtrToLinear(useSRGBRemapping);
      });

  // whether the approximation sRGB->linear mapping should be used in the
  // shader on the environment map textures. This will not be needed once the
  // textures in question are converted on load.
  io::jsonIntoSetter<bool>(
      jsonConfig, "map_ibl_txtr_to_linear",
      [pbrShaderAttribs](bool useSRGBRemapping) {
        pbrShaderAttribs->setMapIBLTxtrToLinear(useSRGBRemapping);
      });

  // whether the approximation linear->sRGB mapping should be used in the
  // shader on the output. This will not be needed when we are using the
  // appropriate framebuffer.
  io::jsonIntoSetter<bool>(
      jsonConfig, "map_output_to_srgb",
      [pbrShaderAttribs](bool useSRGBRemapping) {
        pbrShaderAttribs->setMapOutputToSRGB(useSRGBRemapping);
      });

  // the gamma value for the pbr shader. This value is used for the
  // approximation mapping from sRGB to linear and back.
  io::jsonIntoSetter<double>(
      jsonConfig, "gamma",
      [pbrShaderAttribs](double gamma) { pbrShaderAttribs->setGamma(gamma); });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(pbrShaderAttribs, jsonConfig);

}  // PbrShaderAttributesManager::createFileBasedAttributesTemplate

PbrShaderAttributes::ptr PbrShaderAttributesManager::initNewObjectInternal(
    const std::string& handleName,
    bool) {
  attributes::PbrShaderAttributes::ptr newAttributes =
      this->constructFromDefault(handleName);
  if (nullptr == newAttributes) {
    newAttributes = attributes::PbrShaderAttributes::create(handleName);
  }
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);

  return newAttributes;
}  // PbrShaderAttributesManager::initNewObjectInternal

}  // namespace managers
}  // namespace metadata
}  // namespace esp
