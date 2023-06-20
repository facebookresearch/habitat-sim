// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "PbrShaderAttributes.h"

namespace esp {
namespace metadata {
namespace attributes {

PbrShaderAttributes::PbrShaderAttributes(const std::string& handle)
    : AbstractAttributes("PbrShaderAttributes", handle) {
  setEnableDirectLighting(true);

  // Should IBL default to enabled?
  setEnableIBL(false);
  setDirectLightIntensity(1.0f);
  setSkipCalcMissingTBN(false);
  setUseMikkelsenTBN(false);
  setUseSRGBRemapping(false);
  setUseDirectLightTonemap(false);
  setUseLambertianDiffuse(false);
  // Layer calcs
  setSkipCalcCleacoatLayer(false);
  setSkipCalcSpecularLayer(false);
  setSkipCalcAnisotropyLayer(false);

  // These asset files are fallbacks/defaults incase such files are not included
  // in a dataset, and must be found in /data/pbr and specified in
  // data/pbr/PbrImages.conf

  // Default brdf lookup table is the brdflut from here:
  // https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/screenshots/tex_brdflut.png
  setIBLBrdfLUTAssetHandle("brdflut_ldr_512x512.png");

  // Default equirectangular environment cube map
  setIBLEnvMapAssetHandle("lythwood_room_1k.hdr");

  setTonemapExposure(4.5f);
  setUseIBLTonemap(true);
  // Direct and IBL output scaling
  // Set balance between direct and IBL to be equal for diffuse and specular
  setDirectToIBLDiffuseBalance(.5);
  setDirectToIBLSpecularBalance(.5);

  // For remapping
  setGamma(2.2f);
}  // PbrShaderAttributes ctor

void PbrShaderAttributes::writeValuesToJson(
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  writeValueToJson("enable_direct_lights", jsonObj, allocator);
  writeValueToJson("enable_ibl", jsonObj, allocator);
  writeValueToJson("light_intensity", jsonObj, allocator);
  writeValueToJson("skip_missing_tbn_calc", jsonObj, allocator);
  writeValueToJson("use_mikkelsen_tbn", jsonObj, allocator);
  writeValueToJson("use_srgb_remapping", jsonObj, allocator);
  writeValueToJson("use_direct_tonemap", jsonObj, allocator);
  writeValueToJson("use_lambertian", jsonObj, allocator);
  writeValueToJson("skip_clearcoat_calc", jsonObj, allocator);
  writeValueToJson("skip_specular_layer_calc", jsonObj, allocator);
  writeValueToJson("skip_anisotropy_layer_calc", jsonObj, allocator);
  writeValueToJson("direct_diffuse_scale", jsonObj, allocator);
  writeValueToJson("direct_specular_scale", jsonObj, allocator);
  writeValueToJson("ibl_blut_filename", jsonObj, allocator);
  writeValueToJson("ibl_envmap_filename", jsonObj, allocator);
  writeValueToJson("use_ibl_tonemap", jsonObj, allocator);
  writeValueToJson("ibl_diffuse_scale", jsonObj, allocator);
  writeValueToJson("ibl_specular_scale", jsonObj, allocator);
  writeValueToJson("tonemap_exposure", jsonObj, allocator);
  writeValueToJson("gamma", jsonObj, allocator);

}  // PbrShaderAttributes::writeValuesToJson

std::string PbrShaderAttributes::getObjectInfoHeaderInternal() const {
  return "Direct Lights On,IBL On,Global Direct Light Intensity,Calc "
         "Missing Tangent Frame,Use Mikkelsen TBN Calc,Use sRGB Color "
         "Remapping,Use Lambertian Diffuse,Calc Clearcoat,Calc Spec "
         "Layer,Calc Anisotropy,BRDF LUT Filename,Environment Map "
         "Filename,Scaling [Dir Diffuse|Dir Spec|IBL Diffuse|IBL "
         "Spec],Tonemap Exposure, Global Gamma";
}

/**
 * @brief Retrieve a comma-separated informational string about the contents
 * of this managed object.
 */
std::string PbrShaderAttributes::getObjectInfoInternal() const {
  std::string contribScales = Cr::Utility::formatString(
      "[{}|{}|{}|{}]", getAsString("direct_diffuse_scale"),
      getAsString("direct_specular_scale"), getAsString("ibl_diffuse_scale"),
      getAsString("ibl_specular_scale"));
  return Cr::Utility::formatString(
      "{},{},{},{},{},{},{},{},{},{},{},{},{},",
      getAsString("enable_direct_lights"), getAsString("enable_ibl"),
      getAsString("light_intensity"), getAsString("skip_missing_tbn_calc"),
      getAsString("use_mikkelsen_tbn"), getAsString("use_srgb_remapping"),
      getAsString("use_lambertian"), getAsString("skip_clearcoat_calc"),
      getAsString("skip_specular_layer_calc"),
      getAsString("skip_anisotropy_layer_calc"),
      getAsString("ibl_blut_filename"), getAsString("ibl_envmap_filename"),
      contribScales, getAsString("tonemap_exposure"), getAsString("gamma"));
}

}  // namespace attributes
}  // namespace metadata
}  // namespace esp
