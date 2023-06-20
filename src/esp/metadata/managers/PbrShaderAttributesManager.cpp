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
    PbrShaderAttributes::ptr PbrShaderAttributes,
    const io::JsonGenericValue& jsonConfig) {
  ////////////////////////////
  // Direct lighting calculation settings
  // whether direct lighting should be enabled
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "enable_direct_lights",
      [PbrShaderAttributes](bool enableLights) {
        PbrShaderAttributes->setEnableDirectLighting(enableLights);
      });
  // direct light intensity scale
  io::jsonIntoSetter<double>(
      jsonConfig, "direct_light_intensity",
      [PbrShaderAttributes](double lightIntensity) {
        PbrShaderAttributes->setDirectLightIntensity(lightIntensity);
      });

  // if TBN frame should be calculated if no precomputed tangent is present
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "skip_missing_tbn_calc",
      [PbrShaderAttributes](bool calcMissingTBN) {
        PbrShaderAttributes->setSkipCalcMissingTBN(calcMissingTBN);
      });

  // whether the Mikkelsen method should be used for the TBN calculation, or
  // if a simplified calculation that seems to give equivalent results should be
  // used.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "use_mikkelsen_tbn",
      [PbrShaderAttributes](bool useMikkelsenTBN) {
        PbrShaderAttributes->setUseMikkelsenTBN(useMikkelsenTBN);
      });

  // whether the approximation sRGB<->linear mapping should be used in the
  // shader. This will not be needed once the textures in question are converted
  // on load.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "use_srgb_remapping",
      [PbrShaderAttributes](bool useSRGBRemapping) {
        PbrShaderAttributes->setUseSRGBRemapping(useSRGBRemapping);
      });

  // whether tonemapping should be used for direct lighting
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "use_direct_tonemap",
      [PbrShaderAttributes](bool useDirectTonemap) {
        PbrShaderAttributes->setUseDirectLightTonemap(useDirectTonemap);
      });

  ////////////////////////////
  // Material Layer calculation settings
  // whether we use the lambertian diffuse calculation. If false we use the
  // burley/disney mapping.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "use_lambertian", [PbrShaderAttributes](bool useLambertian) {
        PbrShaderAttributes->setUseDirectLightTonemap(useLambertian);
      });

  // whether clearcoat layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "skip_clearcoat_calc",
      [PbrShaderAttributes](bool skipCalcClearCoat) {
        PbrShaderAttributes->setSkipCalcCleacoatLayer(skipCalcClearCoat);
      });

  // whether speecular layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "skip_specular_layer_calc",
      [PbrShaderAttributes](bool skipCalcSpecular) {
        PbrShaderAttributes->setSkipCalcSpecularLayer(skipCalcSpecular);
      });

  // whether anisotropy layer contributions should be calculated where they are
  // specified by the material.  Note this will not require a rebuild of the
  // shader since only the calculations are disabled.
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "skip_anisotropy_layer_calc",
      [PbrShaderAttributes](bool skipCalcAnisotropy) {
        PbrShaderAttributes->setSkipCalcAnisotropyLayer(skipCalcAnisotropy);
      });

  // direct light diffuse contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(
      jsonConfig, "direct_diffuse_scale", [PbrShaderAttributes](double scale) {
        PbrShaderAttributes->setDirectDiffuseScale(scale);
      });

  // direct light specular contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(
      jsonConfig, "direct_specular_scale", [PbrShaderAttributes](double scale) {
        PbrShaderAttributes->setDirectSpecularScale(scale);
      });

  ////////////////////
  // IBL-specific quantities
  // whether image-based lighting should be used in PBR shader
  io::jsonIntoConstSetter<bool>(jsonConfig, "enable_ibl",
                                [PbrShaderAttributes](bool enableIBL) {
                                  PbrShaderAttributes->setEnableIBL(enableIBL);
                                });

  // the filename for the brdf lookup table used by the IBL calculations
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "ibl_blut_filename",
      [PbrShaderAttributes](const std::string& brdfLUTAsset) {
        PbrShaderAttributes->setIBLBrdfLUTAssetHandle(brdfLUTAsset);
      });

  // the filename for the equirectangular environment map used by the IBL
  // calculations
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "ibl_envmap_filename",
      [PbrShaderAttributes](const std::string& envMapAsset) {
        PbrShaderAttributes->setIBLEnvMapAssetHandle(envMapAsset);
      });

  // whether tonemapping should be used for IBL lighting calculations
  io::jsonIntoConstSetter<bool>(
      jsonConfig, "use_ibl_tonemap", [PbrShaderAttributes](bool useIBLTonemap) {
        PbrShaderAttributes->setUseIBLTonemap(useIBLTonemap);
      });

  // IBL diffuse contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "ibl_diffuse_scale",
                             [PbrShaderAttributes](double scale) {
                               PbrShaderAttributes->setIBLDiffuseScale(scale);
                             });

  // IBL specular contirbution scaling.  Only used if both direct and
  // indirect (IBL) lighting is enabled.
  io::jsonIntoSetter<double>(jsonConfig, "ibl_specular_scale",
                             [PbrShaderAttributes](double scale) {
                               PbrShaderAttributes->setIBLSpecularScale(scale);
                             });

  ///////////////////
  // Both Direct and IBL-related quantities

  // tonemap exposure setting. This value scales the linear color before
  // tonemapping.
  io::jsonIntoSetter<double>(
      jsonConfig, "tonemap_exposure", [PbrShaderAttributes](double exposure) {
        PbrShaderAttributes->setTonemapExposure(exposure);
      });

  // the gamma value for the pbr shader. This value is used for the
  // approximation mapping from sRGB to linear and back.
  io::jsonIntoSetter<double>(jsonConfig, "gamma",
                             [PbrShaderAttributes](double gamma) {
                               PbrShaderAttributes->setGamma(gamma);
                             });

  // check for user defined attributes
  this->parseUserDefinedJsonVals(PbrShaderAttributes, jsonConfig);

}  // PbrShaderAttributesManager::createFileBasedAttributesTemplate

}  // namespace managers
}  // namespace metadata
}  // namespace esp
