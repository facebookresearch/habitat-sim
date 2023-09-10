// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ATTRIBUTES_PBRSHADERATTRIBUTES_H_
#define ESP_METADATA_ATTRIBUTES_PBRSHADERATTRIBUTES_H_

#include "AttributesBase.h"

namespace esp {
namespace metadata {
namespace attributes {

/**
 * @brief attributes class describing essential and default quantities and
 * settings for configuring PBR shader calculations.
 */
class PbrShaderAttributes : public AbstractAttributes {
 public:
  explicit PbrShaderAttributes(const std::string& handle = "");

  ////////////////////////////
  // Direct lighting compiler directive settings
  // Changing these values will require recompiling the shader

  /**
   * @brief Set whether direct lighting should be used in PBR shader.
   */
  void setEnableDirectLighting(bool enableLights) {
    set("enable_direct_lights", enableLights);
  }

  /**
   * @brief Get whether direct lighting should be used in PBR shader.
   */
  bool getEnableDirectLighting() const {
    return get<bool>("enable_direct_lights");
  }

  /**
   * @brief Set the scene-wide direct lighting intensity for pbr shader. Applied
   * to all direct lighting contributions equally.
   */
  void setDirectLightIntensity(double intensity) {
    set("direct_light_intensity", intensity);
  }

  /**
   * @brief Get the scene-wide lighting intensity for pbr shader. Applied to all
   * direct lighting contributions equally.
   */
  float getDirectLightIntensity() const {
    return static_cast<float>(get<double>("direct_light_intensity"));
  }

  /**
   * @brief Set if we should skip the calculation of a TBN frame in the fragment
   * shader if no precalculated TBN is provided. Without this frame normal
   * textures cannot be used, and anisotropy, if present, will not look
   * appropriate.
   */
  void setSkipCalcMissingTBN(bool skipCalcMissingTBN) {
    set("skip_missing_tbn_calc", skipCalcMissingTBN);
  }

  /**
   * @brief Get if we should skip the calculation of a TBN frame in the fragment
   * shader if no precalculated TBN is provided. Without this frame normal
   * textures cannot be used, and anisotropy, if present, will not look
   * appropriate.
   */
  bool getSkipCalcMissingTBN() const {
    return get<bool>("skip_missing_tbn_calc");
  }

  /**
   * @brief Set if we should use the more expensive formulation for calculating
   * the TBN frame in the fragment shader using partial derivatives given by
   * Mikkelsen, as per https://jcgt.org/published/0009/03/04/paper.pdf .
   * Otherwise a simplified version is used that seems to give equivalent
   * results, based on
   * https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag
   * These calculations are only performed if a precalculated tangent is not
   * provided and if normal textures or anisotropy are specified.
   */
  void setUseMikkelsenTBN(bool useMikkelsenTBN) {
    set("use_mikkelsen_tbn", useMikkelsenTBN);
  }

  /**
   * @brief Get if we should use the more expensive formulation for calculating
   * the TBN frame in the fragment shader using partial derivatives given by
   * Mikkelsen, as per https://jcgt.org/published/0009/03/04/paper.pdf .
   * Otherwise a simplified version is used that seems to give equivalent
   * results, based on
   * https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag
   * These calculations are only performed if a precalculated tangent is not
   * provided and if normal textures or anisotropy are specified.
   */
  bool getUseMikkelsenTBN() const { return get<bool>("use_mikkelsen_tbn"); }

  /**
   * @brief Set if we should use shader-based srgb->linear approx remapping of
   * applicable material color textures in PBR rendering for direct lighting and
   * IBL. This field should be removed/ignored when Magnum fully supports sRGB
   * texture conversion on load.
   */
  void setMapMatTxtrToLinear(bool mapMatTxtrToLinear) {
    set("map_mat_txtr_to_linear", mapMatTxtrToLinear);
  }

  /**
   * @brief Get if we should use shader-based srgb->linear approx remapping of
   * applicable material color textures in PBR rendering for direct lighting and
   * IBL. This field should be removed/ignored when Magnum fully supports sRGB
   * texture conversion on load.
   */
  bool getMapMatTxtrToLinear() const {
    return get<bool>("map_mat_txtr_to_linear");
  }

  /**
   * @brief Set if we should use shader-based srgb->linear approx remapping of
   * applicable IBL environment textures in PBR rendering for IBL calculations.
   * This field should be removed/ignored when Magnum fully supports sRGB
   * texture conversion on load.
   */
  void setMapIBLTxtrToLinear(bool mapIBLTxtrToLinear) {
    set("map_ibl_txtr_to_linear", mapIBLTxtrToLinear);
  }

  /**
   * @brief Get if we should use shader-based srgb->linear approx remapping of
   * applicable IBL environment textures in PBR rendering for IBL calculations.
   * This field should be removed/ignored when Magnum fully supports sRGB
   * texture conversion on load.
   */
  bool getMapIBLTxtrToLinear() const {
    return get<bool>("map_ibl_txtr_to_linear");
  }

  /**
   * @brief Set if we should use shader-based linear->srgb approx remapping of
   * color output in PBR rendering for direct lighting and IBL results. This
   * field should be removed/ignored when an appropriate framebuffer is used for
   * output to handle this conversion.
   */
  void setMapOutputToSRGB(bool mapOutToSRGB) {
    set("map_output_to_srgb", mapOutToSRGB);
  }

  /**
   * @brief Get if we should use shader-based linear->srgb approx remapping of
   * color output in PBR rendering for direct lighting and IBL results. This
   * field should be removed/ignored when an appropriate framebuffer is used for
   * output to handle this conversion.
   */
  bool getMapOutputToSRGB() const { return get<bool>("map_output_to_srgb"); }

  /**
   * @brief Set if we should use tonemapping for direct lighting.
   * TODO : specify multiple tonemappings? Each would be defined in the shader,
   * and we could specify which to use by an enum.
   */
  void setUseDirectLightTonemap(bool useDirectTonemap) {
    set("use_direct_tonemap", useDirectTonemap);
  }

  /**
   * @brief Get if we should use tonemapping for direct lighting.
   * TODO : specify multiple tonemappings? Each would be defined in the shader,
   * and we could specify which to use by an enum.
   */
  bool getUseDirectLightTonemap() const {
    return get<bool>("use_direct_tonemap");
  }

  ////////////////////////////
  // Material Layer calculation compiler directive settings
  // Changing these values will require recompiling the shader

  /**
   * @brief Set if we should use a diffuse calculation based on Burley, modified
   * to be more energy conserving :
   * https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
   * instead of the default Lambertian calculation for direct lighting diffuse
   * color under direct light. By default our PBR shader uses Lambertian, which
   * is simpler and quicker to calculate but may not look as 'nice'
   */
  void setUseBurleyDiffuse(bool useBurleyDiffuse) {
    set("use_burley_diffuse", useBurleyDiffuse);
  }

  /**
   * @brief Get if we should use a diffuse calculation based on Burley, modified
   * to be more energy conserving :
   * https://media.disneyanimation.com/uploads/production/publication_asset/48/asset/s2012_pbs_disney_brdf_notes_v3.pdf
   * instead of the default Lambertian calculation for direct lighting diffuse
   * color under direct light. By default our PBR shader uses Lambertian, which
   * is simpler and quicker to calculate but may not look as 'nice'
   */
  bool getUseBurleyDiffuse() const { return get<bool>("use_burley_diffuse"); }

  /**
   * @brief Set whether the clearcoat layer calculations should be skipped. If
   * true, disable calcs regardless of material setting. Note this will not
   * require a rebuild of the shader since only the calculations are disabled.
   */
  void setSkipCalcClearcoatLayer(bool skipCalcClearCoat) {
    set("skip_clearcoat_calc", skipCalcClearCoat);
  }

  /**
   * @brief Set whether the clearcoat layer calculations should be skipped. If
   * true, disable calcs regardless of material setting. Note this will not
   * require a rebuild of the shader since only the calculations are disabled.
   */
  bool getSkipCalcClearcoatLayer() const {
    return get<bool>("skip_clearcoat_calc");
  }

  /**
   * @brief Set whether the specular layer calculations should be
   * skipped. If true, disable calcs regardless of material setting. Note
   * this will not require a rebuild of the shader since only the calculations
   * are disabled.
   */
  void setSkipCalcSpecularLayer(bool skipCalcSpecular) {
    set("skip_specular_layer_calc", skipCalcSpecular);
  }

  /**
   * @brief Set whether the specular layer calculations should be
   * skipped. If true, disable calcs regardless of material setting. Note
   * this will not require a rebuild of the shader since only the calculations
   * are disabled.
   */
  bool getSkipCalcSpecularLayer() const {
    return get<bool>("skip_specular_layer_calc");
  }

  /**
   * @brief Set whether the anisotropy layer calculations should be
   * skipped. If true, disable calcs regardless of material setting. Note
   * this will not require a rebuild of the shader since only the calculations
   * are disabled.
   */
  void setSkipCalcAnisotropyLayer(bool skipCalcAnisotropy) {
    set("skip_anisotropy_layer_calc", skipCalcAnisotropy);
  }

  /**
   * @brief Set whether the anisotropy layer calculations should be
   * skipped. If true, disable calcs regardless of material setting. Note
   * this will not require a rebuild of the shader since only the calculations
   * are disabled.
   */
  bool getSkipCalcAnisotropyLayer() const {
    return get<bool>("skip_anisotropy_layer_calc");
  }

  ////////////////////
  // IBL-specific compiler directive quantities
  // Changing these values will require recompiling the shader

  /**
   * @brief Set whether Image-based lighting should be used in PBR shader.
   */
  void setEnableIBL(bool enableIBL) { set("enable_ibl", enableIBL); }

  /**
   * @brief Get whether Image-based lighting should be used in PBR shader.
   */
  bool getEnableIBL() { return get<bool>("enable_ibl"); }

  /**
   * @brief Set the filename for the brdf lookup table used by the IBL
   * calculations.Also builds the PbrIBLHelper key to check/retrive helpers
   * in map in ResourceManager.
   */
  void setIBLBrdfLUTAssetHandle(const std::string& brdfLUTAsset) {
    set("ibl_blut_filename", brdfLUTAsset);
    set("pbr_ibl_helper_key",
        Cr::Utility::formatString("{}_{}", brdfLUTAsset,
                                  get<std::string>("ibl_envmap_filename")));
  }
  /**
   * @brief Get the filename for the brdf lookup table used by the IBL
   * calculations.
   */
  std::string getIBLBrdfLUTAssetHandle() const {
    return get<std::string>("ibl_blut_filename");
  }

  /**
   * @brief Set the filename for the equirectangular environment map used by the
   * IBL calculations. Also builds the PbrIBLHelper key to check/retrive helpers
   * in map in ResourceManager.
   */
  void setIBLEnvMapAssetHandle(const std::string& envMapAsset) {
    set("ibl_envmap_filename", envMapAsset);
    set("pbr_ibl_helper_key",
        Cr::Utility::formatString(
            "{}_{}", get<std::string>("ibl_blut_filename"), envMapAsset));
  }

  /**
   * @brief Get the filename for the equirectangular environment map used by the
   * calculations.
   */
  std::string getIBLEnvMapAssetHandle() const {
    return get<std::string>("ibl_envmap_filename");
  }

  /**
   * @brief Retrieve the handle for the PbrIBLHelper to be used for objects
   * using this attributes. This value is set internally when either bLUT or
   * EnvMap asset names are set. Format is '<bLUT asset handle>_<envMap asset
   * handle>'.
   */
  std::string getPbrShaderHelperKey() const {
    return get<std::string>("pbr_ibl_helper_key");
  }

  /**
   * @brief set if we should use tonemapping for IBL lighting.
   * TODO : Eventually provide mechanism for specifying multiple tonemappings?
   * Each would be defined in the shader, and we could specify which to use by
   * an enum.
   */
  void setUseIBLTonemap(bool useIBLTonemap) {
    set("use_ibl_tonemap", useIBLTonemap);
  }

  /**
   * @brief Get if we should use tonemapping for IBL lighting.
   * TODO : Eventually provide mechanism for specifying multiple tonemappings?
   * Each would be defined in the shader, and we could specify which to use by
   * an enum.
   */
  bool getUseIBLTonemap() const { return get<bool>("use_ibl_tonemap"); }

  ///////////////////
  // Both Direct and IBL-related uniform quantities
  // Changing these values would not require shader rebuild

  /**
   * @brief Set the exposure value for tonemapping in the pbr shader. This value
   * scales the color before the tonemapping is applied.
   */
  void setTonemapExposure(double exposure) {
    set("tonemap_exposure", exposure);
  }

  /**
   * @brief Get the exposure value for tonemapping in the pbr shader. This value
   * scales the color before the tonemapping is applied.
   */
  float getTonemapExposure() const {
    return static_cast<float>(get<double>("tonemap_exposure"));
  }

  /**
   * @brief Set the gamma value for the pbr shader. This value is used
   * for the approximation mapping from sRGB to linear and back. Cannot be <= 0.
   */
  void setGamma(double gamma) { set("gamma", Mn::Math::max(gamma, 0.000001)); }

  /**
   * @brief Get the gamma value for the pbr shader. This value is used
   * for the approximation mapping from sRGB to linear and back.
   */
  float getGamma() const { return static_cast<float>(get<double>("gamma")); }

  /**
   * @brief Convenience accessor to address balance between direct
   * diffuse and indirect diffuse. Setting this to 0 or less
   * will have full direct lighting result and no indirect/IBL, setting to 1 or
   * more will have full IBL lighting result and no direct lighting
   * contributions. Only used if both direct and image-based lighting is
   * enabled.
   */
  void setIBLToDirectDiffuseBalance(double balance) {
    balance = Mn::Math::clamp(balance, 0.0, 1.0);
    set("direct_diffuse_scale", 1.0 - balance);
    set("ibl_diffuse_scale", balance);
  }

  /**
   * @brief Convenience accessor to address balance between direct
   * specular and indirect specular. Setting this to 0 or less
   * will have full direct lighting result and no indirect/IBL, setting to 1 or
   * more will have full IBL lighting result and no direct lighting
   * contributions. Only used if both direct and image-based lighting is
   * enabled.
   */
  void setIBLToDirectSpecularBalance(double balance) {
    balance = Mn::Math::clamp(balance, 0.0, 1.0);
    set("direct_specular_scale", 1.0 - balance);
    set("ibl_specular_scale", balance);
  }

  /**
   * @brief Convenience accessor to balance between direct diffuse and indirect
   * diffuse. Retrieves a value from [0.1], with 0 meaning only direct lighting
   * diffuse results, and 1 meaning only image-based lighting diffuse results.
   * Only used if both direct and image-based lighting is enabled.
   */
  double getIBLToDirectDiffuseBalance() const {
    auto direct = get<double>("direct_diffuse_scale");
    auto ibl = get<double>("ibl_diffuse_scale");
    auto sum = direct + ibl;
    return ibl / sum;
  }

  /**
   * @brief Convenience accessor to balance between direct specular and indirect
   * specular. Retrieves a value from [0.1], with 0 meaning only direct
   * lighting specular results, and 1 meaning only image-based lighting specular
   * results. Only used if both direct and image-based lighting is enabled.
   */
  double getIBLToDirectSpecularBalance() const {
    auto direct = get<double>("direct_specular_scale");
    auto ibl = get<double>("ibl_specular_scale");
    auto sum = direct + ibl;
    return ibl / sum;
  }

  /**
   * @brief Set value to scale the direct lighting diffuse contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  void setDirectDiffuseScale(double scale) {
    set("direct_diffuse_scale", scale);
  }
  /**
   * @brief Get value to scale the direct lighting diffuse contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  float getDirectDiffuseScale() const {
    return static_cast<float>(get<double>("direct_diffuse_scale"));
  }

  /**
   * @brief Set value to scale the direct lighting specular contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  void setDirectSpecularScale(double scale) {
    set("direct_specular_scale", scale);
  }
  /**
   * @brief Get value to scale the direct lighting specular contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  float getDirectSpecularScale() const {
    return static_cast<float>(get<double>("direct_specular_scale"));
  }

  /**
   * @brief Set value to scale the IBL lighting diffuse contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  void setIBLDiffuseScale(double scale) { set("ibl_diffuse_scale", scale); }

  /**
   * @brief Get value to scale the IBL lighting diffuse contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  float getIBLDiffuseScale() const {
    return static_cast<float>(get<double>("ibl_diffuse_scale"));
  }

  /**
   * @brief Set value to scale the IBL lighting specular contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  void setIBLSpecularScale(double scale) { set("ibl_specular_scale", scale); }
  /**
   * @brief Get value to scale the IBL lighting specular contribution if both
   * Direct lighting and IBL are specified, Ignored otherwise. Value is not
   * checked, so can create energy if direct + indirect contributions scaled by
   * more than 1.0.
   */
  float getIBLSpecularScale() const {
    return static_cast<float>(get<double>("ibl_specular_scale"));
  }

  /**
   * @brief Populate a json object with all the first-level values held in
   * this configuration.  Default is overridden to handle special cases for
   * PbrShaderAttributes.
   */
  void writeValuesToJson(io::JsonGenericValue& jsonObj,
                         io::JsonAllocator& allocator) const override;

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object, type-specific.
   */

  std::string getObjectInfoHeaderInternal() const override;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfoInternal() const override;

 public:
  ESP_SMART_POINTERS(PbrShaderAttributes)
};  // class PbrShaderAttributes

}  // namespace attributes
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ATTRIBUTES_PBRSHADERATTRIBUTES_H_
