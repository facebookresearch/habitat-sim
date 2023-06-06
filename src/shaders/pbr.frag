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

  /////////////////
  // direct lights

/////////////////
// Direct lighting
#if (LIGHT_COUNT > 0)

  // compute contribution of each light using the microfacet model
  // the following part of the code is inspired by the Phong.frag in Magnum
  // library (https://magnum.graphics/)
  for (int iLight = 0; iLight < LIGHT_COUNT; ++iLight) {
    // Directional lights have the .w component set to 0
    // Non-directional lights (i.e. point) have w == 1

    // Incident light vector - directions have been flipped for directional
    // lights before being fed to uniform so we can use the same function for
    // both kinds of lights without a condition check
    vec3 toLightVec =
        uLightDirections[iLight].xyz - (position * uLightDirections[iLight].w);
    // either the length of the toLightVec vector or 0 (for directional
    // lights, to enable attenuation calc to stay 1)
    highp float dist = length(toLightVec) * uLightDirections[iLight].w;
    // either the squared length of the toLightVec vector or 1 (for
    // directional lights, to prevent divide by 0)
    highp float sqDist = (((dist * dist) - 1) * uLightDirections[iLight].w) + 1;

    // If uLightRanges is 0 for whatever reason, clamp it to a small value to
    // avoid a NaN when dist is 0 as well (which is the case for directional
    // lights)
    // https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_lights_punctual/README.md#range-property
    // Attenuation is 1 for directional lights, governed by inverse square law
    // otherwise if no range is given
    highp float attenuation =
        clamp(1 - pow4(dist / (uLightRanges[iLight] + epsilon)), 0.0, 1.0) /
        sqDist;

    // if color is not visible, skip contribution
    if (attenuation == 0) {
      continue;
    }

    // Build a light info for this light
    LightInfo l;
    configureLightInfo(normalize(toLightVec),
                       uLightColors[iLight] * attenuation, pbrInfo.n,
                       pbrInfo.view, pbrInfo.n_dot_v, l);

    // Calculate the Schlick approximation of the Fresnel coefficient
    // Fresnel Specular color at view angle == Schlick approx using view angle
    vec3 fresnel = fresnelSchlick(pbrInfo.specularColor_f0,
                                  pbrInfo.specularColor_f90, l.v_dot_h);

    // NOTE : REMOVED INV_PI scaling of direct lights so we don't have to
    // increase the intensity for PBR lights in c++ . This way phong and pbr are
    // reasonably equivalent brightness with the same light setup.

    // Lambertian diffuse contribution
    // currentDiffuseContrib =
    //     l.projLightIrradiance * //INV_PI *
    //      pbrInfo.diffuseColor;

    // Burley/Disney diffuse contribution
    vec3 currentDiffuseContrib =
        l.projLightIrradiance *  // INV_PI *
        BRDF_BurleyDiffuseRenorm(pbrInfo.diffuseColor, l,
                                 pbrInfo.alphaRoughness);

// TODO get rid of skip when implementation is ready
#if defined(ANISOTROPY_LAYER) && !defined(SKIP_CALC_ANISOTROPY_LAYER)
    // Specular microfacet for anisotropic layer
    // calculate light-specific anisotropic layer cosines
    AnistropyDirectLight anisoLightInfo;
    configureAnisotropyLightInfo(l, pbrInfo, anisoLightInfo);

    // Anisotropic specular contribution
    vec3 currentSpecularContrib =
        l.projLightIrradiance *  // INV_PI *
        BRDF_specularAnisotropicGGX(fresnel, l, pbrInfo, anisoLightInfo);

#else
    // Specular microfacet - 1/pi from specular D normal dist function
    vec3 currentSpecularContrib =
        l.projLightIrradiance *  // INV_PI *
        BRDF_Specular(fresnel, l, pbrInfo.alphaRoughness);
#endif  // Anisotropy else isotropy
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
    LightInfo cc_l;
    // build a clearcoat normal-based light info
    configureLightInfo(l.light, l.lightIrradiance, pbrInfo.clearCoatNormal,
                       pbrInfo.view, pbrInfo.cc_n_dot_v, cc_l);
    // scalar clearcoat contribution
    // RECALCULATE LIGHT for clearcoat normal
    vec3 ccFresnel =
        fresnelSchlick(pbrInfo.clearCoatCoating_f0,
                       pbrInfo.clearCoatCoating_f90, cc_l.v_dot_h) *
        pbrInfo.clearCoatStrength;
    // get clearcoat contribution
    vec3 currentClearCoatContrib =
        cc_l.projLightIrradiance *  // INV_PI *
        BRDF_ClearCoatSpecular(ccFresnel, cc_l, pbrInfo.clearCoatRoughness);
    // Scale substrate specular by 1-ccfresnel to account for coating
    currentSpecularContrib *= (1 - ccFresnel);
#endif  // CLEAR_COAT

#if defined(SHADOWS_VSM)
    float shadow =
        (iLight < maxShadowNum
             ? computeShadowVSM(iLight, position, uLightDirections[iLight].xyz)
             : 1.0f);
    currentDiffuseContrib *= shadow;
    currentSpecularContrib *= shadow;
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
    currentClearCoatContrib *= shadow;
#endif  // CLEAR_COAT
#endif

    // TODO Transmission here

    // aggregate contributions for each light
    pbrInfo.diffuseContrib += currentDiffuseContrib;
    pbrInfo.specularContrib += currentSpecularContrib;
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
    pbrInfo.clearCoatContrib += currentClearCoatContrib;
#endif  // CLEAR_COAT
  }     // for each light

#endif  // if (LIGHT_COUNT > 0)

#if defined(IMAGE_BASED_LIGHTING)

  pbrInfo.iblDiffuseContrib =
      computeIBLDiffuse(pbrInfo.diffuseColor, pbrInfo.n);

#if defined(ANISOTROPY_LAYER) && !defined(SKIP_CALC_ANISOTROPY_LAYER)
  // Derive bent normal for reflection
  vec3 bentNormal = cross(pbrInfo.anisotropicB, pbrInfo.view);
  bentNormal = normalize(cross(bentNormal, pbrInfo.anisotropicB));
  // This heuristic can probably be improved upon
  float a = pow4(1.0 - pbrInfo.anisotropy * (1.0 - pbrInfo.perceivedRoughness));
  bentNormal = normalize(mix(bentNormal, pbrInfo.n, a));
  // float ibl_n_dot_v = abs(dot(bentNormal, pbrInfo.view));
  vec3 reflection = normalize(reflect(-pbrInfo.view, bentNormal));
#else
  // float ibl_n_dot_v =  n_dot_v;
  vec3 reflection = normalize(reflect(-pbrInfo.view, pbrInfo.n));
#endif  // if ANISOTROPY else

  pbrInfo.iblSpecularContrib =
      computeIBLSpecular(pbrInfo.perceivedRoughness, pbrInfo.n_dot_v,
                         pbrInfo.specularColor_f0, reflection);

#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  // Clear coat reflection
  vec3 cc_reflection =
      normalize(reflect(-pbrInfo.view, pbrInfo.clearCoatNormal));
  // Clear coat reflection contribution
  pbrInfo.iblClearCoatContrib =
      computeIBLSpecular(pbrInfo.clearCoatRoughness, pbrInfo.cc_n_dot_v,
                         pbrInfo.clearCoatCoating_f0, cc_reflection);
  // Scale substrate specular by 1-cc_fresnel to account for coating
  pbrInfo.iblSpecularContrib *= (pbrInfo.OneM_ccFresnelGlbl);
#endif  // CLEAR_COAT

#endif  // IMAGE_BASED_LIGHTING

////////////////////
// Scale if both direct lighting and ibl are enabled
#if defined(IMAGE_BASED_LIGHTING) && (LIGHT_COUNT > 0)

  // Only scale direct lighting contribution if also using IBL
  pbrInfo.diffuseContrib *= uComponentScales[DirectDiffuse];
  pbrInfo.specularContrib *= uComponentScales[DirectSpecular];

  // Only scale IBL if also using direct lighting
  pbrInfo.iblDiffuseContrib *= uComponentScales[IblDiffuse];
  pbrInfo.iblSpecularContrib *= uComponentScales[IblSpecular];
#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  // TODO provide custom scaling factor for Direct lit clear coat?
  pbrInfo.clearCoatContrib *= uComponentScales[DirectSpecular];
  pbrInfo.iblClearCoatContrib *= uComponentScales[IblSpecular];
#endif  // CLEAR_COAT

#endif  // Component scaling if both types of lighting exist

  ///////////
  // Total contributions

  // TODO expand emissiveColor handling
  fragmentColor = vec4(pbrInfo.emissiveColor, pbrInfo.baseColor.a);

  // Aggregate total contributions from direct and indirect lighting
  vec3 ttlDiffuseContrib = pbrInfo.diffuseContrib + pbrInfo.iblDiffuseContrib;
  vec3 ttlSpecularContrib =
      pbrInfo.specularContrib + pbrInfo.iblSpecularContrib;

  fragmentColor.rgb += vec3(ttlDiffuseContrib + ttlSpecularContrib);

#if defined(CLEAR_COAT) && !defined(SKIP_CALC_CLEAR_COAT)
  // scale by clearcoat strength
  vec3 ttlClearCoatContrib =
      (pbrInfo.clearCoatContrib + pbrInfo.iblClearCoatContrib) *
      pbrInfo.clearCoatStrength;
  // Scale entire contribution from substrate by 1-clearCoatFresnel
  // to account for energy loss in base layer due to coating and add coating
  // contribution
  // https://google.github.io/filament/Filament.md.html#figure_clearcoat
  fragmentColor.rgb =
      (fragmentColor.rgb * (pbrInfo.OneM_ccFresnelGlbl)) + ttlClearCoatContrib;

#endif  // CLEAR_COAT

#if defined(OBJECT_ID)
  fragmentObjectId = uObjectId;
#endif

// PBR equation debug
// "none", "Diff (l,n)", "F (l,h)", "G (l,v,h)", "D (h)", "Specular"
#if defined(PBR_DEBUG_DISPLAY)
  if (uPbrDebugDisplay > 0) {
    switch (uPbrDebugDisplay) {
      case 1:
        fragmentColor.rgb = pbrInfo.diffuseContrib;  // direct diffuse
        break;
      case 2:
        fragmentColor.rgb = pbrInfo.specularContrib;  // direct specular
        break;
      case 3:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = pbrInfo.iblDiffuseContrib;  // ibl diffuse
#endif
        break;
      case 4:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = pbrInfo.iblSpecularContrib;  // ibl specular
#endif
        break;
      case 5:
        fragmentColor.rgb = pbrInfo.n;  // normal
        break;
      case 6:
        // TODO: Shadows
        /*
  #if defined(SHADOWS_VSM)
        fragmentColor.rgb =
            visualizePointShadowMap(1, position, uLightDirections[1].xyz);
  #endif
  */
        break;
    }  // switch
  }
#endif
}  // main
