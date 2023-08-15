// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This PBR implementation draws from many sources, including
// Karis, Brian. “Real Shading in Unreal Engine 4.” (2013).

precision highp float;

// -------------- output -------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out vec4 fragmentColor;
#if defined(OBJECT_ID)
layout(location = OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID) out highp uint
    fragmentObjectId;
#endif

void main() {
  // Build structure to hold uniform-provided and subsequently derived values
  // for lighting calcs
  PBRData pbrInfo = buildPBRData();

  // Build a structure to hold the results of the direct and indirect (IBL)
  // lighting calcs
  PBRResultData colorVals = buildBaseColorResults();

  /////////////////
  // direct lights

/////////////////
// Direct lighting
#if defined(DIRECT_LIGHTING)

  // compute contribution of each light using the microfacet model
  // the following part of the code is inspired by the Phong.frag in Magnum
  // library (https://magnum.graphics/)
  for (int iLight = 0; iLight < LIGHT_COUNT; ++iLight) {
    // Build a light info for this light
    LightInfo l;
    if (!buildLightInfoFromLightIdx(iLight, pbrInfo, l)) {
      continue;
    }

    // Calculate the Schlick approximation of the Fresnel coefficient
    // Fresnel Specular color at view angle == Schlick approx using view
    //     angle
    vec3 fresnel = fresnelSchlick(pbrInfo.specularColor_f0,
                                  pbrInfo.specularColor_f90, l.v_dot_h);

#if defined(USE_BURLEY_DIFFUSE)
    // Burley/Disney diffuse contribution
    vec3 currentDiffuseContrib =
        l.projLightIrradiance *
        BRDF_BurleyDiffuseRenorm(pbrInfo.diffuseColor, l,
                                 pbrInfo.alphaRoughness);
#else
    // Lambertian diffuse contribution (simpler calc)
    vec3 currentDiffuseContrib = l.projLightIrradiance * pbrInfo.diffuseColor;

#endif  // USE_BURLEY_DIFFUSE else use lambertian diffuse

#if defined(ANISOTROPY_LAYER) && !defined(SKIP_CALC_ANISOTROPY_LAYER)
    // Specular microfacet for anisotropic layer
    // calculate light-specific anisotropic layer cosines
    AnisotropyDirectLight anisoLightInfo;
    configureAnisotropyLightInfo(l, pbrInfo, anisoLightInfo);

    // Anisotropic specular contribution
    vec3 currentSpecularContrib =
        l.projLightIrradiance *
        BRDF_specularAnisotropicGGX(fresnel, l, pbrInfo, anisoLightInfo);
#else
    // Specular microfacet
    vec3 currentSpecularContrib =
        l.projLightIrradiance *
        BRDF_Specular(fresnel, l, pbrInfo.alphaRoughness);
#endif  // ANISOTROPY_LAYER else isotropy

#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
    LightInfo cc_l;
    // build a clearcoat normal-based light info
    configureLightInfo(l.light, l.lightIrradiance, pbrInfo.clearCoatNormal,
                       pbrInfo.view, pbrInfo.cc_n_dot_v, cc_l);
    // scalar clearcoat fresnel, scaled by strength
    vec3 ccFresnel =
        fresnelSchlick(pbrInfo.clearCoatCoating_f0,
                       pbrInfo.clearCoatCoating_f90, cc_l.v_dot_h) *
        pbrInfo.clearCoatStrength;
    // get clearcoat contribution
    vec3 currentClearCoatContrib =
        cc_l.projLightIrradiance *
        BRDF_ClearCoatSpecular(ccFresnel, cc_l, pbrInfo.clearCoatRoughness);
    // Scale substrate specular by 1-ccfresnel to account for coating
    currentSpecularContrib *= (1 - ccFresnel);
#endif  // CLEAR_COAT

    // TODO Transmission here

    // aggregate contributions for each light
    colorVals.diffuseContrib += currentDiffuseContrib;
    colorVals.specularContrib += currentSpecularContrib;
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
    colorVals.clearCoatContrib += currentClearCoatContrib;
#endif  // CLEAR_COAT

  }  // for each light

  // Set intensity and scale by inv_pi for surface area
  float colorContribScale = uDirectLightIntensity * INV_PI;

  // Scale each direct light color
  colorVals.diffuseContrib *= colorContribScale;
  colorVals.specularContrib *= colorContribScale;
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  colorVals.clearCoatContrib *= colorContribScale;
#endif  // CLEAR_COAT

// If using direct lighting tone map
#if defined(DIRECT_TONE_MAP)
  colorVals.diffuseContrib = toneMap(colorVals.diffuseContrib);
  colorVals.specularContrib = toneMap(colorVals.specularContrib);
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  colorVals.clearCoatContrib = toneMap(colorVals.clearCoatContrib);
#endif  // CLEAR_COAT
#endif  // DIRECT_TONE_MAP

#endif  // if (DIRECT_LIGHTING)

#if defined(IMAGE_BASED_LIGHTING)

  colorVals.iblDiffuseContrib =
      computeIBLDiffuse(pbrInfo.diffuseColor, pbrInfo.n);

#if defined(ANISOTROPY_LAYER) && !defined(SKIP_CALC_ANISOTROPY_LAYER)
  vec3 reflection = normalize(reflect(-pbrInfo.view, pbrInfo.bentNormal));
#else
  vec3 reflection = normalize(reflect(-pbrInfo.view, pbrInfo.n));
#endif  // if ANISOTROPY else

  colorVals.iblSpecularContrib =
      computeIBLSpecular(pbrInfo.alphaRoughness, pbrInfo.n_dot_v,
                         pbrInfo.specularColor_f0, reflection);

#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
// Clear coat reflection
#if defined(ANISOTROPY_LAYER) && !defined(SKIP_CALC_ANISOTROPY_LAYER)
  vec3 cc_reflection = normalize(reflect(-pbrInfo.view, pbrInfo.cc_BentNormal));
#else
  vec3 cc_reflection =
      normalize(reflect(-pbrInfo.view, pbrInfo.clearCoatNormal));
#endif
  // Clear coat reflection contribution
  colorVals.iblClearCoatContrib =
      computeIBLSpecular(pbrInfo.clearCoatRoughness, pbrInfo.cc_n_dot_v,
                         pbrInfo.clearCoatCoating_f0, cc_reflection) *
      pbrInfo.clearCoatStrength;
  // Scale substrate specular by 1-cc_fresnel to account for coating
  colorVals.iblSpecularContrib *= (pbrInfo.OneM_ccFresnelGlbl);
#endif  // CLEAR_COAT

#endif  // IMAGE_BASED_LIGHTING

////////////////////
// Scale if both direct lighting and ibl are enabled
#if defined(IMAGE_BASED_LIGHTING) && defined(DIRECT_LIGHTING)

  // Only scale direct lighting contribution if also using IBL
  colorVals.diffuseContrib *= uComponentScales[DirectDiffuse];
  colorVals.specularContrib *= uComponentScales[DirectSpecular];

  // Only scale IBL if also using direct lighting
  colorVals.iblDiffuseContrib *= uComponentScales[IblDiffuse];
  colorVals.iblSpecularContrib *= uComponentScales[IblSpecular];
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  // TODO provide custom scaling factor for Direct lit clear coat?
  colorVals.clearCoatContrib *= uComponentScales[DirectSpecular];
  colorVals.iblClearCoatContrib *= uComponentScales[IblSpecular];
#endif  // CLEAR_COAT

#endif  // Component scaling if both types of lighting exist

  ///////////
  // Total contributions

  // Aggregate total contributions from direct and indirect lighting
  vec3 ttlDiffuseContrib =
      colorVals.diffuseContrib + colorVals.iblDiffuseContrib;
  vec3 ttlSpecularContrib =
      colorVals.specularContrib + colorVals.iblSpecularContrib;

  // Aggregate direct and indirect diffuse and specular with emissive color
  // TODO expand emissiveColor handling
  vec3 finalColor =
      vec3(ttlDiffuseContrib + ttlSpecularContrib + pbrInfo.emissiveColor);

#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  // Total clearcoat contribution
  vec3 ttlClearCoatContrib =
      (colorVals.clearCoatContrib + colorVals.iblClearCoatContrib);
  // Scale entire contribution from substrate by 1-clearCoatFresnel
  // to account for energy loss in base layer due to coating and add coating
  // contribution
  // https://google.github.io/filament/Filament.md.html#figure_clearcoat
  finalColor =
      (finalColor * (pbrInfo.OneM_ccFresnelGlbl)) + ttlClearCoatContrib;

#endif  // CLEAR_COAT

// final aggregation
// TODO alpha masking?

// Whether to remap the output to sRGB or not
#if defined(MAP_OUTPUT_TO_SRGB)
  fragmentColor = vec4(linearToSRGB(finalColor), pbrInfo.baseColor.a);
#else
  fragmentColor = vec4(finalColor, pbrInfo.baseColor.a);
#endif  // MAP_OUTPUT_TO_SRGB

#if defined(OBJECT_ID)
  fragmentObjectId = uObjectId;
#endif

// PBR equation debug
// "none", "Diff (l,n)", "F (l,h)", "G (l,v,h)", "D (h)", "Specular"
#if defined(PBR_DEBUG_DISPLAY)
  if (uPbrDebugDisplay > 0) {
    switch (uPbrDebugDisplay) {
      case 1:
        fragmentColor.rgb = colorVals.diffuseContrib;  // direct diffuse
        break;
      case 2:
        fragmentColor.rgb = colorVals.specularContrib;  // direct specular
        break;
      case 3:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = colorVals.iblDiffuseContrib;  // ibl diffuse
#endif
        break;
      case 4:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = colorVals.iblSpecularContrib;  // ibl specular
#endif
        break;
      case 5:
        fragmentColor.rgb = pbrInfo.n;  // normal
        break;
      default:
        break;
    }  // switch
  }
#endif
}  // main
