// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This PBR implementation draws from many sources, including
// Karis, Brian. “Real Shading in Unreal Engine 4.” (2013).

precision highp float;

// -------------- input ---------------------
// position, normal, tangent, biTangent in world space,
// NOT camera space
in highp vec3 position;
in highp vec3 normal;
#if defined(TEXTURED)
in highp vec2 texCoord;
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
in highp vec3 tangent;
in highp vec3 biTangent;
#endif

// -------------- output -------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out vec4 fragmentColor;
#if defined(OBJECT_ID)
layout(location = OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID) out highp uint
    fragmentObjectId;
#endif


// -------------- uniforms ----------------
#if defined(OBJECT_ID)
uniform highp uint ObjectId;
#endif

#if defined(NORMAL_TEXTURE)
uniform mediump float NormalTextureScale
#ifndef GL_ES
    = 1.0
#endif
    ;
#endif

// camera position in world space
uniform highp vec3 CameraWorldPos;


uniform int PbrDebugDisplay;

/////////////////////
// define this macro to disable anisotropy calc/display
#if defined(ANISOTROPY_LAYER)
//#define SKIP_ANISOTROPY_DISPLAY
#endif


// REMOVED INV_PI so we don't have to increase the luminance by PI for lights in c++
//const float INV_PI = 1.0 / PI;

const int maxShadowNum = 3;


// Build TBN matrix
// Using local gradient of position and UV coords to derive tangent if not provided
// See https://jcgt.org/published/0009/03/04/paper.pdf Section 3.3
#if (defined(NORMAL_TEXTURE)|| defined(CLEAR_COAT_NORMAL_TEXTURE) || defined(ANISOTROPY_LAYER))
mat3 buildTBN(){
  vec3 N = normalize(normal);
#if defined(PRECOMPUTED_TANGENT)
  vec3 T = normalize(tangent);
  vec3 B = normalize(biTangent);
#else
  vec3 T;
  vec3 B;
  vec3 posDx = dFdx(position);
  vec3 posDy = dFdy(position);
  vec2 uvDx2 = dFdx(texCoord);
  vec2 uvDy2 = dFdy(texCoord);
  if(length(uvDx2) + length(uvDy2) < epsilon) {
    uvDx2 = vec2(1.0, 0.0);
    uvDy2 = vec2(0.0, 1.0);
  }
// {
//   // From paper See https://jcgt.org/published/0009/03/04/paper.pdf Section 3.3
//   if (gl_FrontFacing == false) {
//     N *= -1.0;
//   }
//   vec3 sigmaX = posDx - dot(posDx, N) * N;
//   vec3 sigmaY = posDy - dot(posDy, N) * N;
//   float flip_sign = dot(posDy, cross(N, posDx)) < 0 ? -1.0 : 1.0;

//   vec2 uvDy2Inv = vec2(uvDy2.t, -uvDy2.s);
//   float det = dot(uvDx2, uvDy2Inv);
//   float sign_det = det < 0.0 ? -1.0 : 1.0;

//   vec2 invC0 = sign_det * uvDy2Inv;
//   T = normalize(sigmaX * invC0.x + sigmaY * invC0.y);

//   B = (sign_det * flip_sign) * normalize(cross(N, T));
// }
  // Alternate method, may be cheaper, better performing
  // from https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag
{
  T = (uvDy2.t * posDx - uvDx2.t * posDy) / (uvDx2.s * uvDy2.t - uvDy2.s * uvDx2.t);
  //Gramm-Schmidt renorm
    T = normalize(T - N * dot(N, T));
    B = cross(N, T);
  if (gl_FrontFacing == false) {
    N *= -1.0;
  }
}
#endif // if defined(PRECOMPUTED_TANGENT) else
  // negate the TBN matrix for back-facing primitives
  if (gl_FrontFacing == false) {
    T *= -1.0;
    B *= -1.0;
  }
  return mat3(T, B, N);
}

// Derive normal from normal map sample
// normTextureSample : Normal texture sample at texCoords
// normTextureScale : scale amount for normal texture sample
// TBN : Tangent/Bitangent/Normal frame, maps tangentspace to world
vec3 getNormalFromNormalMap(vec3 normTextureSample, float normalTextureScale, mat3 TBN) {
  vec3 tangentNormal = normalize((normTextureSample * 2.0 - 1.0) *
                vec3(normalTextureScale, normalTextureScale, 1.0));
  // TBN transforms tangentNormal from tangent space to world space
  return normalize(TBN * tangentNormal);
}
#endif  // if (defined(NORMAL_TEXTURE) || defined(CLEAR_COAT_NORMAL_TEXTURE))


void main() {

///////////////////////
// If normal texture, clearcoat normal texture or anisotropy is provided/specified but no precomputed tangent is provided,
// the TBN frame will be built using local gradients of position and uv coordinates based on following paper
// See https://jcgt.org/published/0009/03/04/paper.pdf Section 3.3

// TODO verify this is acceptable performance for synthesizing TBN frame

#if (defined(NORMAL_TEXTURE) || defined(CLEAR_COAT_NORMAL_TEXTURE) || defined(ANISOTROPY_LAYER))
  mat3 TBN = buildTBN();
#endif

#if defined(NORMAL_TEXTURE)
  // normal is now in the camera space
  vec3 n = getNormalFromNormalMap(texture(NormalTexture, texCoord).xyz, NormalTextureScale, TBN);
#else
  vec3 n = normalize(normal);
  // This means backface culling is disabled,
  // which implies it is rendering with the "double sided" material.
  // Based on glTF 2.0 Spec, the normal must be reversed for back faces
  if (gl_FrontFacing == false) {
    n *= -1.0;
  }
#endif

/////////////////
// Initialization
// Scalars
  // Index of refraction 1.5 yields 0.04 dielectric fresnel reflectance at normal incidence
  float ior = Material.ior;
  // Index of refraction of adjacent material (air unless clearcoat is present)
  float ior_adj = 1.0;

/////////////////
// vectors
  // View is the normalized vector from the shading location to the camera
  // in *world space*
  vec3 view = normalize(CameraWorldPos - position);

  // view projected on normal
  float n_dot_v = abs(dot(n, view));


//////////////////////
// colors
  vec4 baseColor = Material.baseColor;
#if defined(BASECOLOR_TEXTURE)
  baseColor *= texture(BaseColorTexture, texCoord);
#endif

  vec3 emissiveColor = Material.emissiveColor;
#if defined(EMISSIVE_TEXTURE)
  emissiveColor *= texture(EmissiveTexture, texCoord).rgb;
#endif

/////////////////
//Metalness and Roughness calc

  float perceivedRoughness = Material.roughness;
  float metallic = Material.metallic;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
  vec3 RoughnessMetallicSample = texture(MetallicRoughnessTexture, texCoord).rgb;

  perceivedRoughness *= RoughnessMetallicSample.g;
  metallic *= RoughnessMetallicSample.b;
#endif
  // clamp roughness to prevent denormals in distribution function calc
  perceivedRoughness = clamp(perceivedRoughness, 0.045, 1.0);
  // Roughness is authored as perceptual roughness by convention,
  // convert to more linear roughness mapping by squaring the perceptual roughness.
  float alphaRoughness = perceivedRoughness * perceivedRoughness;


/////////////////
// Diffuse color calc

  // diffuse color remapping based on metallic value (aka c_diff)
  // https://google.github.io/filament/Filament.md.html#listing_basecolortodiffuse
  vec3 diffuseColor =  baseColor.rgb * (1 - metallic);

/////////////////
//Clearcoat layer support

#if defined(CLEAR_COAT)
  //TODO if clearCoatStrength is 0 then ignore clearcoat
  float clearCoatStrength = ClearCoat.factor;
#if defined(CLEAR_COAT_TEXTURE)
  clearCoatStrength *= texture(ClearCoatTexture, texCoord).r;
#endif
  clearCoatStrength = clamp(clearCoatStrength, 0.0, 1.0);

  float clearCoatPerceivedRoughness = ClearCoat.roughness;
#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
  clearCoatPerceivedRoughness *= texture(ClearCoatRoughnessTexture, texCoord).g;
#endif
  // clamp clearcoat roughness to prevent denormals in distribution function calc
  clearCoatPerceivedRoughness = clamp(clearCoatPerceivedRoughness, 0.045, 1.0);
  float clearCoatRoughness = clearCoatPerceivedRoughness * clearCoatPerceivedRoughness;

  //
  // If clearcoatNormalTexture is not given, no normal mapping is applied to the clear
  // coat layer, even if normal mapping is applied to the base material. Otherwise,
  // clearcoatNormalTexture may be a reference to the same normal map used by the
  // base material, or any other compatible normal map.

  vec3 cc_Normal = n;
#if defined(CLEAR_COAT_NORMAL_TEXTURE)
  // TODO Need to verify TBH generation is reasonable if precomputed tangent not provided
  cc_Normal = getNormalFromNormalMap(texture(ClearCoatNormalTexture, texCoord).xyz, ClearCoat.normalTextureScale, TBN);
#endif
  float cc_n_dot_v = dot(cc_Normal, view);
  // Assuming dielectric reflectance of 0.4, which corresponds to
  // polyurethane coating with IOR == 1.5
  vec3 clearCoatCoating_f0 = vec3(0.4);
  vec3 clearCoatCoating_f90 = vec3(1.0);
  // We need to use this to modify the base dielectric reflectance,
  // since the clearcoat, adjacent to the material, is not air,
  // and it darkens the reflectance of the underlying material
  ior_adj = 1.5;

  //Get glbl fresnel contribution for IBL and for clear-coat contrib
  vec3 OneM_ccFresnelGlbl = 1-(fresnelSchlick(clearCoatCoating_f0, clearCoatCoating_f90, cc_n_dot_v) * clearCoatStrength);
#endif // CLEAR_COAT


  // DielectricReflectance == 0.04 <--> ior == 1.5
  // If clearcoat is present, ior_adj is not 1.0 of air but 1.5 of polyurethane coating
  float DielectricReflectance = pow2((ior - ior_adj) / (ior + ior_adj));

  // Achromatic dielectric material f0 : fresnel reflectance at normal incidence
  // based on given IOR
  vec3 dielectric_f0 = vec3(DielectricReflectance);

  // glancing incidence dielectric specular reflectance
  vec3 specularColor_f90 = vec3(1.0);

////////////////////
// Anisotropy Layer support

// init from here
// https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#individual-lights
#if defined(ANISOTROPY_LAYER)
  float anisotropy = AnisotropyLayer.factor;
  vec2 anisotropyDir = AnisotropyLayer.direction;

#if defined(ANISOTROPY_LAYER_TEXTURE)
  vec3 anisotropyTex = texture(AnisotropyLayerTexture, texCoord).rgb;
  anisotropyDir = anisotropyTex.rg * 2.0 - vec2(1.0);
  anisotropyDir = mat2(AnisotropyLayer.direction.x, AnisotropyLayer.direction.y, -AnisotropyLayer.direction.y, AnisotropyLayer.direction.x) * normalize(anisotropyDir);
  anisotropy *= anisotropyTex.b;
#endif // ANISOTROPY_LAYER_TEXTURE

  //Tangent and bitangent
  vec3 anisotropicT = normalize(TBN * vec3(anisotropyDir, 0.0));
  vec3 anisotropicB = normalize(cross(n, anisotropicT));


  AnisotropyInfo anisoInfo;
  //Populate AnisotropyInfo object for this shader
  configureAnisotropyInfo(anisotropyDir, anisotropy, anisotropicT, anisotropicB, view, alphaRoughness, anisoInfo);


#endif // ANISOTROPY_LAYER


/////////////////
//Specular layer support
  //Modifies fresnel terms
#if defined(SPECULAR_LAYER)

  float specularWeight = SpecularLayer.factor;
#if defined(SPECULAR_LAYER_TEXTURE)
  specularWeight *= texture(SpecularLayerTexture, texCoord).a;
#endif
  // The F0 color of the specular reflection (linear RGB)
  vec3 specularLayerColor = SpecularLayer.colorFactor;

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)
  //TODO SpecularLayerColorTexture is in sRGB
  specularLayerColor *= texture(SpecularLayerColorTexture, texCoord).rgb;
#endif
  // Recalculate dielectric_f0 and specularColor_f90 based on passed specular layer quantities
  // see https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_specular
  dielectric_f0 = min(dielectric_f0 *specularLayerColor, vec3(1.0)) * specularWeight;
  // specularColor_f90 set to vec3(1)
  specularColor_f90 = mix(vec3(specularWeight), specularColor_f90, metallic);

#endif // SPECULAR_LAYER
  // compute specular reflectance (fresnel) at normal incidence
  // for dielectric or metallic, using IOR-derived fresnel reflectance
  // accounting for specular layers
  vec3 specularColor_f0 = mix(dielectric_f0, baseColor.rgb, metallic);


/////////////////
// lights

  // Contributions for diffuse, specular, clearcoat
  // for direct and image-based lighting
  vec3 diffuseContrib = vec3(0.0, 0.0, 0.0);
  vec3 specularContrib = vec3(0.0, 0.0, 0.0);
#if defined(CLEAR_COAT)
  vec3 clearCoatContrib = vec3(0.0, 0.0, 0.0);
#endif // Clearcoat

  vec3 iblDiffuseContrib = vec3(0.0, 0.0, 0.0);
  vec3 iblSpecularContrib  = vec3(0.0, 0.0, 0.0);
#if defined(CLEAR_COAT)
  vec3 iblClearCoatContrib = vec3(0.0, 0.0, 0.0);
#endif // Clearcoat

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
    // lights before being fed to uniform so we can use the same function for both
    // kinds of lights without a condition check
    vec3 toLightVec =
          LightDirections[iLight].xyz - (position * LightDirections[iLight].w);
    // either the length of the toLightVec vector or 0 (for directional
    // lights, to enable attenuation calc to stay 1)
    highp float dist = length(toLightVec) * LightDirections[iLight].w;
    // either the squared length of the toLightVec vector or 1 (for
    // directional lights, to prevent divide by 0)
    highp float sqDist = (((dist * dist) - 1) * LightDirections[iLight].w) + 1;

    // If LightRanges is 0 for whatever reason, clamp it to a small value to
    // avoid a NaN when dist is 0 as well (which is the case for directional lights)
    // https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_lights_punctual/README.md#range-property
    // Attenuation is 1 for directional lights, governed by inverse square law
    // otherwise if no range is given
    highp float attenuation =
        clamp(1 - pow4(dist / (LightRanges[iLight] + epsilon)), 0.0, 1.0) /
        sqDist;

    //if color is not visible, skip contribution
    if (attenuation == 0){
      continue;
    }

    //Build a light info for this light
    LightInfo l;
    configureLightInfo(normalize(toLightVec), LightColors[iLight] * attenuation, n, view, n_dot_v, l);


    // Calculate the Schlick approximation of the Fresnel coefficient
    // Fresnel Specular color at view angle == Schlick approx using view angle
    vec3 fresnel = fresnelSchlick(specularColor_f0, specularColor_f90, l.v_dot_h);

    // Lambertian diffuse contribution
    // currentDiffuseContrib =
    //     l.projLightRadiance * //INV_PI *
    //      diffuseColor;

    // Burley/Disney diffuse contribution
    vec3 currentDiffuseContrib =
        l.projLightRadiance * //INV_PI *
        BRDF_BurleyDiffuseRenorm(diffuseColor, l,
                           alphaRoughness);

// TODO get rid of skip when implementation is ready
#if defined(ANISOTROPY_LAYER) && !defined(SKIP_ANISOTROPY_DISPLAY)
    // Specular microfacet for anisotropic layer
    // calculate light-specific anisotropic layer cosines
    AnistropyDirectLight anisoLightInfo;
    configureAnisotropyLightInfo(l, anisoInfo, anisoLightInfo);

    // Anisotropic specular contribution
    vec3 currentSpecularContrib = l.projLightRadiance * //INV_PI *
                BRDF_specularAnisotropicGGX(fresnel, l, anisoInfo, anisoLightInfo);

#else
    // Specular microfacet - 1/pi from specular D normal dist function
    vec3 currentSpecularContrib = l.projLightRadiance * //INV_PI *
                             BRDF_Specular(fresnel, l, alphaRoughness);
#endif // Anisotropy else isotropy
#if defined(CLEAR_COAT)
    LightInfo cc_l;
    //build a clearcoat normal-based light info
    configureLightInfo(l.light, l.lightRadiance, cc_Normal, view, cc_n_dot_v, cc_l);
    // scalar clearcoat contribution
    // RECALCULATE LIGHT for clearcoat normal
    vec3 ccFresnel = fresnelSchlick(clearCoatCoating_f0, clearCoatCoating_f90, cc_l.v_dot_h) * clearCoatStrength;
    // get clearcoat contribution
    vec3 currentClearCoatContrib = cc_l.projLightRadiance * //INV_PI *
            BRDF_ClearCoatSpecular(ccFresnel, cc_l, clearCoatRoughness);
    // Scale substrate specular by 1-ccfresnel to account for coating
    currentSpecularContrib *= (1-ccFresnel);
#endif // CLEAR_COAT

#if defined(SHADOWS_VSM)
    float shadow =
        (iLight < maxShadowNum
             ? computeShadowVSM(iLight, position, LightDirections[iLight].xyz)
             : 1.0f);
    currentDiffuseContrib *= shadow;
    currentSpecularContrib *= shadow;
#if defined(CLEAR_COAT)
    currentClearCoatContrib *= shadow;
#endif // CLEAR_COAT
#endif

    // TODO Transmission here
    diffuseContrib += currentDiffuseContrib;
    specularContrib += currentSpecularContrib;
#if defined(CLEAR_COAT)
    clearCoatContrib += currentClearCoatContrib;
#endif// CLEAR_COAT
  }  // for each light

#endif  // if (LIGHT_COUNT > 0)

#if defined(IMAGE_BASED_LIGHTING)

  iblDiffuseContrib = computeIBLDiffuse(diffuseColor, n);

#if defined(ANISOTROPY_LAYER)  && !defined(SKIP_ANISOTROPY_DISPLAY)
  //Derive bent normal for reflection
  vec3 bentNormal = cross(anisoInfo.anisotropicB, view);
  bentNormal = normalize(cross(bentNormal, anisoInfo.anisotropicB));
  // This heuristic can probably be improved upon
  float a = pow4(1.0 - anisoInfo.anisotropy * (1.0 - perceivedRoughness));
  bentNormal = normalize(mix(bentNormal, n, a));
  //float ibl_n_dot_v = abs(dot(bentNormal, view));
  vec3 reflection = normalize(reflect(-view, bentNormal));
#else
  //float ibl_n_dot_v =  n_dot_v;
  vec3 reflection = normalize(reflect(-view, n));
#endif //if ANISOTROPY else

  iblSpecularContrib =
      computeIBLSpecular(perceivedRoughness, n_dot_v, specularColor_f0, reflection);

#if defined(CLEAR_COAT)
  //Clear coat reflection
  vec3 cc_reflection = normalize(reflect(-view, cc_Normal));
  //Clear coat reflection contribution
  iblClearCoatContrib =
        computeIBLSpecular(clearCoatRoughness, cc_n_dot_v, clearCoatCoating_f0, cc_reflection);
  // Scale substrate specular by 1-cc_fresnel to account for coating
  iblSpecularContrib *= (OneM_ccFresnelGlbl);
#endif // CLEAR_COAT

#endif  // IMAGE_BASED_LIGHTING

////////////////////
// Scale if both direct lighting and ibl are enabled
#if defined(IMAGE_BASED_LIGHTING) && (LIGHT_COUNT > 0)

  // Only scale direct lighting contribution if also using IBL
  diffuseContrib *= ComponentScales[DirectDiffuse];
  specularContrib *= ComponentScales[DirectSpecular];

  //Only scale IBL if also using direct lighting
  iblDiffuseContrib *= ComponentScales[IblDiffuse];
  iblSpecularContrib *= ComponentScales[IblSpecular];
#if defined(CLEAR_COAT)
  //TODO provide custom scaling factor for Direct lit clear coat?
  clearCoatContrib *= ComponentScales[DirectSpecular];
  iblClearCoatContrib *= ComponentScales[IblSpecular];
#endif// CLEAR_COAT

#endif// Component scaling if both types of lighting exist


///////////
// Total contributions

//TODO verify if this is the best way to handle emissive color
fragmentColor = vec4(emissiveColor, baseColor.a);


vec3 ttlDiffuseContrib = diffuseContrib + iblDiffuseContrib;
vec3 ttlSpecularContrib = specularContrib +  iblSpecularContrib;

fragmentColor.rgb += vec3(ttlDiffuseContrib + ttlSpecularContrib);

#if defined(CLEAR_COAT)
  // scale by clearcoat strength
  vec3 ttlClearCoatContrib = (clearCoatContrib + iblClearCoatContrib) * clearCoatStrength;
  // Scale entire contribution from substrate -again- by 1-clearCoatFresnel
  // https://google.github.io/filament/Filament.md.html#figure_clearcoat
  fragmentColor.rgb = (fragmentColor.rgb * (OneM_ccFresnelGlbl)) + ttlClearCoatContrib;

#endif // CLEAR_COAT

#if defined(OBJECT_ID)
  fragmentObjectId = ObjectId;
#endif

// #if defined(ANISOTROPY_LAYER)
//   fragmentColor = vec4(1,0,0,1);
// #endif

// PBR equation debug
// "none", "Diff (l,n)", "F (l,h)", "G (l,v,h)", "D (h)", "Specular"
#if defined(PBR_DEBUG_DISPLAY)
  if (PbrDebugDisplay > 0) {
    switch (PbrDebugDisplay) {
      case 1:
        fragmentColor.rgb = diffuseContrib;  // direct diffuse
        break;
      case 2:
        fragmentColor.rgb = specularContrib;  // direct specular
        break;
      case 3:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = iblDiffuseContrib;  // ibl diffuse
#endif
        break;
      case 4:
#if defined(IMAGE_BASED_LIGHTING)
        fragmentColor.rgb = iblSpecularContrib;  // ibl specular
#endif
        break;
      case 5:
        fragmentColor.rgb = n;  // normal
        break;
      case 6:
        // TODO: Shadows
        /*
  #if defined(SHADOWS_VSM)
        fragmentColor.rgb =
            visualizePointShadowMap(1, position, LightDirections[1].xyz);
  #endif
  */
        break;
    }  // switch
  }
#endif
}  // main
