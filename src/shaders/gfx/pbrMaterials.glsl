// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// -------------- process uniforms and derive data ------------------

// #define USE_MIKKELSEN_TBN
// Specifying this macro uses a more expensive calculation to derive the TBN
// frame From paper See https://jcgt.org/published/0009/03/04/paper.pdf
// section 3.3 but it might be more accurate
//
// Otherwise a simpler method is used that seems to yield equivalent results,
// from
// https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag

// Build TBN matrix
// Using local gradient of position and UV coords to derive tangent if
// precomputed tangent is not provided See
// https://jcgt.org/published/0009/03/04/paper.pdf Section 3.3
#if (defined(NORMAL_TEXTURE) || defined(CLEAR_COAT_NORMAL_TEXTURE) || \
     defined(ANISOTROPY_LAYER))
mat3 buildTBN() {
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
  if (length(uvDx2) + length(uvDy2) < EPSILON) {
    uvDx2 = vec2(1.0, 0.0);
    uvDy2 = vec2(0.0, 1.0);
  }

#if defined(USE_MIKKELSEN_TBN)
  // From paper See https://jcgt.org/published/0009/03/04/paper.pdf
  // Section 3.3
  if (gl_FrontFacing == false) {
    N *= -1.0;
  }
  vec3 sigmaX = posDx - dot(posDx, N) * N;
  vec3 sigmaY = posDy - dot(posDy, N) * N;
  float flip_sign = dot(posDy, cross(N, posDx)) < 0 ? -1.0 : 1.0;

  vec2 uvDy2Inv = vec2(uvDy2.t, -uvDy2.s);
  float det = dot(uvDx2, uvDy2Inv);
  float sign_det = det < 0.0 ? -1.0 : 1.0;

  vec2 invC0 = sign_det * vec2(uvDy2.t, -uvDx2.t);
  T = normalize(sigmaX * invC0.x + sigmaY * invC0.y);

  B = (sign_det * flip_sign) * normalize(cross(N, T));
#else
  // Simplified method, may be cheaper/better performing, from
  // https://github.com/KhronosGroup/Vulkan-Samples/blob/main/shaders/pbr.frag

  T = (uvDy2.t * posDx - uvDx2.t * posDy) /
      (uvDx2.s * uvDy2.t - uvDy2.s * uvDx2.t);
  // Gramm-Schmidt orthonormalization step
  T = normalize(T - N * dot(N, T));
  B = cross(N, T);
  if (gl_FrontFacing == false) {
    N *= -1.0;
  }
#endif  // if defined(USE_MIKKELSEN_TBN)
#endif  // if defined(PRECOMPUTED_TANGENT) else
  // negate the TBN matrix for back-facing primitives
  if (gl_FrontFacing == false) {
    T *= -1.0;
    B *= -1.0;
  }
  return mat3(T, B, N);
}  // buildTBN()

// Derive normal from normal map sample
// normTextureSample : Normal texture sample at texCoords
// normTextureScale : scale amount for normal texture sample
// TBN : Tangent/Bitangent/Normal frame, maps tangentspace to world
vec3 getNormalFromNormalMap(vec3 normTextureSample,
                            float normalTextureScale,
                            mat3 TBN) {
  vec3 tangentNormal =
      normalize((normTextureSample * 2.0 - 1.0) *
                vec3(normalTextureScale, normalTextureScale, 1.0));
  // TBN transforms tangentNormal from tangent space to world space
  return normalize(TBN * tangentNormal);
}
#endif  // if (defined(NORMAL_TEXTURE)|| defined(CLEAR_COAT_NORMAL_TEXTURE) ||
        // defined(ANISOTROPY_LAYER))

// Build normal and TBN frame, if appropriate,
// and set PBRData values for normal, view vector
// material-derived IOR and adjacent material IOR,
// baseColor, emissiveColor, roughness and metallicness
PBRData buildPBRData() {
  PBRData pbrInfo;
  ///////////////////////
  // If normal texture, clearcoat normal texture or anisotropy is
  // provided/specified but no precomputed tangent is provided, the TBN frame
  // will be built using local gradients of position and uv coordinates based
  // on following paper See https://jcgt.org/published/0009/03/04/paper.pdf
  // Section 3.3

#if (defined(NORMAL_TEXTURE) || defined(CLEAR_COAT_NORMAL_TEXTURE) || \
     defined(ANISOTROPY_LAYER))
  pbrInfo.TBN = buildTBN();
#endif

#if defined(NORMAL_TEXTURE) && !defined(SKIP_CALC_NORMAL_TEXTURE)
  // normal is now in the camera space
  pbrInfo.n = getNormalFromNormalMap(texture(uNormalTexture, texCoord).xyz,
                                     uNormalTextureScale, pbrInfo.TBN);
#else
  pbrInfo.n = normalize(normal);
  // This means backface culling is disabled,
  // which implies it is rendering with the "double sided" material.
  // Based on glTF 2.0 Spec, the normal must be reversed for back faces
  if (gl_FrontFacing == false) {
    pbrInfo.n *= -1.0;
  }
#endif  // defined(NORMAL_TEXTURE) && !defined(SKIP_CALC_NORMAL_TEXTURE)

  // Index of refraction 1.5 yields 0.04 dielectric fresnel reflectance at
  // normal incidence
  pbrInfo.ior = uMaterial.ior;
  // IOR of adjacent material
  pbrInfo.ior_adj =
#if defined(CLEAR_COAT)
      // We need to use this to modify the base dielectric reflectance,
      // since the clearcoat, adjacent to the material, is not air,
      // and it darkens the reflectance of the underlying material
      1.5f;
#else   // ifndef CLEAR_COAT (use air value)
      1.0f;
#endif  // if defined(CLEAR_COAT) else

  // View is the normalized vector from the shading location to the camera
  // in *world space*
  pbrInfo.view = normalize(uCameraWorldPos - position);

  // cos angle between view and normal
  // clamp to ensure range adherence, abs instead of epsilon to avoid errors at
  // clancing angles
  pbrInfo.n_dot_v = clamp(dot(pbrInfo.n, pbrInfo.view), 0.0, 1.0);
  //////////////////////
  // colors
  pbrInfo.baseColor = uMaterial.baseColor;
#if defined(BASECOLOR_TEXTURE)

#if defined(MAP_MAT_TXTRS_TO_LINEAR)
  pbrInfo.baseColor *= sRGBToLinear(texture(uBaseColorTexture, texCoord));
#else
  pbrInfo.baseColor *= texture(uBaseColorTexture, texCoord);
#endif  // MAP_MAT_TXTRS_TO_LINEAR
#endif  // BASECOLOR_TEXTURE

  pbrInfo.emissiveColor = uMaterial.emissiveColor;
#if defined(EMISSIVE_TEXTURE)

#if defined(MAP_MAT_TXTRS_TO_LINEAR)
  pbrInfo.emissiveColor *=
      sRGBToLinear(texture(uEmissiveTexture, texCoord).rgb);
#else
  pbrInfo.emissiveColor *= texture(uEmissiveTexture, texCoord).rgb;

#endif  // MAP_MAT_TXTRS_TO_LINEAR
#endif  // EMISSIVE_TEXTURE

  /////////////////
  // Metalness and Roughness calc
  pbrInfo.perceivedRoughness = uMaterial.roughness;
  pbrInfo.metallic = uMaterial.metallic;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
  vec3 RoughnessMetallicSample =
      texture(uMetallicRoughnessTexture, texCoord).rgb;

  pbrInfo.perceivedRoughness *= RoughnessMetallicSample.g;
  pbrInfo.metallic *= RoughnessMetallicSample.b;
#endif  // NONE_ROUGHNESS_METALLIC_TEXTURE
  // clamp roughness to prevent denormals in distribution function calc
  pbrInfo.perceivedRoughness = clamp(pbrInfo.perceivedRoughness, 0.045, 1.0);
  // Roughness is authored as perceptual roughness by convention,
  // convert to more linear roughness mapping by squaring the perceptual
  // roughness.
  pbrInfo.alphaRoughness =
      pbrInfo.perceivedRoughness * pbrInfo.perceivedRoughness;

  /////////////////
  // Diffuse color calc

  // diffuse color remapping based on metallic value (aka c_diff)
  // https://google.github.io/filament/Filament.md.html#listing_basecolortodiffuse
  pbrInfo.diffuseColor = pbrInfo.baseColor.rgb * (1 - pbrInfo.metallic);

  // ior_adj is changed based on whether clearcoat or not

  // dielectricReflectance == 0.04 <--> ior == 1.5
  // If clearcoat is present, ior_adj is not 1.0 of air but 1.5 of
  // polyurethane coating, as defined in PBRInfoStruct
  float dielectricReflectance =
      pow2((pbrInfo.ior - pbrInfo.ior_adj) / (pbrInfo.ior + pbrInfo.ior_adj));

  // Achromatic dielectric material f0 : fresnel reflectance at normal
  // incidence based on given IOR
  // This may be modified by specular layer quantities
  pbrInfo.dielectric_f0 = vec3(dielectricReflectance);
  // glancing incidence dielectric/metallic reflectance. Specular layer data
  // modfies this
  pbrInfo.specularColor_f90 = vec3(1.0);

  /////////////////
  // Clearcoat layer support
#if defined(CLEAR_COAT)
  // Assuming dielectric reflectance of 0.04, which corresponds to
  // polyurethane coating with IOR == 1.5
  pbrInfo.clearCoatCoating_f0 = vec3(0.04);
  // Glancing angle reflectance
  pbrInfo.clearCoatCoating_f90 = vec3(1.0);

  pbrInfo.clearCoatStrength = uClearCoat.factor;
#if defined(CLEAR_COAT_TEXTURE)
  pbrInfo.clearCoatStrength *= texture(uClearCoatTexture, texCoord).r;
#endif  // CLEAR_COAT_TEXTURE
  pbrInfo.clearCoatStrength = clamp(pbrInfo.clearCoatStrength, 0.0, 1.0);

  pbrInfo.clearCoatPerceivedRoughness = uClearCoat.roughness;
#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
  pbrInfo.clearCoatPerceivedRoughness *=
      texture(uClearCoatRoughnessTexture, texCoord).g;
#endif  // CLEAR_COAT_ROUGHNESS_TEXTURE
  // clamp clearcoat roughness to prevent denormals in distribution function
  // calc
  pbrInfo.clearCoatPerceivedRoughness =
      clamp(pbrInfo.clearCoatPerceivedRoughness, 0.045, 1.0);
  pbrInfo.clearCoatRoughness =
      pbrInfo.clearCoatPerceivedRoughness * pbrInfo.clearCoatPerceivedRoughness;
  //
  // If clearcoatNormalTexture is not given, no normal mapping is applied to
  // the clear coat layer, even if normal mapping is applied to the base
  // material. Otherwise, clearcoatNormalTexture may be a reference to the
  // same normal map used by the base material, or any other compatible normal
  // map.
  pbrInfo.clearCoatNormal = pbrInfo.n;
#if defined(CLEAR_COAT_NORMAL_TEXTURE)
  // TBH generation appears reasonable when precomputed tangent not provided
  pbrInfo.clearCoatNormal =
      getNormalFromNormalMap(texture(uClearCoatNormalTexture, texCoord).xyz,
                             uClearCoat.normalTextureScale, pbrInfo.TBN);
#endif  // CLEAR_COAT_NORMAL_TEXTURE
  // Clearcoat cos angle between clearcoat normal and view
  pbrInfo.cc_n_dot_v =
      clamp(dot(pbrInfo.clearCoatNormal, pbrInfo.view), 0.0, 1.0);

  // Get glbl normal-based clearcoat fresnel contribution for IBL
  // and direct lit clear-coat contrib calc
  // https://google.github.io/filament/Filament.md.html#toc5.3.5
  pbrInfo.OneM_ccFresnelGlbl =
      1 - (fresnelSchlick(pbrInfo.clearCoatCoating_f0,
                          pbrInfo.clearCoatCoating_f90, pbrInfo.cc_n_dot_v) *
           pbrInfo.clearCoatStrength);

  // Clearcoat is on top of emissive layer, so emissive layer is darkened by
  // clearcoat
  pbrInfo.emissiveColor *= pbrInfo.OneM_ccFresnelGlbl;

#endif  // CLEAR_COAT

////////////////////
// Anisotropy Layer support

// init from here
// https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#individual-lights
#if defined(ANISOTROPY_LAYER)
  // Strength of effect - negative directs in bitangent direction
  pbrInfo.anisotropy = uAnisotropyLayer.factor;
  // Direction of anisotropy, as a 2d rotation in Tangentspace plane, measured
  // from tangent
  vec2 anisotropyDir = uAnisotropyLayer.direction;

#if defined(ANISOTROPY_LAYER_TEXTURE)
  vec3 anisotropyTex = texture(uAnisotropyLayerTexture, texCoord).rgb;
  anisotropyDir = anisotropyTex.rg * 2.0 - vec2(1.0);
  anisotropyDir =
      mat2(uAnisotropyLayer.direction.x, uAnisotropyLayer.direction.y,
           -uAnisotropyLayer.direction.y, uAnisotropyLayer.direction.x) *
      normalize(anisotropyDir);
  pbrInfo.anisotropy *= anisotropyTex.b;
#endif  // ANISOTROPY_LAYER_TEXTURE

  // Tangent and bitangent
  // aniso tangent and bitangent are TBN rotated by anistropy dir
  pbrInfo.anisotropicT = normalize(pbrInfo.TBN * vec3(anisotropyDir, 0.0));
  pbrInfo.anisotropicB = normalize(cross(pbrInfo.n, pbrInfo.anisotropicT));

#if defined(IMAGE_BASED_LIGHTING)
  // Precalc bent normal for reflection
  // Allow for negative anisotropy
  // From Filament
  // https://github.com/google/filament/blob/5ffb52a17d1c9d710140f263ca82b8ab23cae59e/shaders/src/common_lighting.fs#L90

  vec3 TBNAnisoDir =
      (pbrInfo.anisotropy >= 0.0 ? pbrInfo.anisotropicB : pbrInfo.anisotropicT);
  vec3 anisoTan = cross(TBNAnisoDir, pbrInfo.view);
  vec3 bentNormal = normalize(cross(anisoTan, TBNAnisoDir));
  // From KHR standard description
  // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#implementation
  float a = (pow4(1.0 - abs(pbrInfo.anisotropy) *
                            (1.0 - pbrInfo.perceivedRoughness)));
  pbrInfo.bentNormal = normalize(mix(bentNormal, pbrInfo.n, a));
#if defined(CLEAR_COAT)
  pbrInfo.cc_BentNormal =
      normalize(mix(bentNormal, pbrInfo.clearCoatNormal, a));
#endif  // ANISOTROPY_LAYER
#endif  // IMAGE_BASED_LIGHTING

  // Derive roughness in each direction
  // Below from standard
  // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#implementation
  // pbrInfo.aT = mix( pbrInfo.alphaRoughness, 1.0,  pbrInfo.anisotropy*
  // pbrInfo.anisotropy);
  // pbrInfo.aB =  pbrInfo.alphaRoughness;

  // Below from
  // https://google.github.io/filament/Filament.md.html#materialsystem/anisotropicmodel/anisotropicspecularbrdf
  // If anisotropy == 0 then just alpharoughness in each direction
  pbrInfo.aT =
      max(pbrInfo.alphaRoughness * (1.0 + pbrInfo.anisotropy), EPSILON);
  pbrInfo.aB =
      max(pbrInfo.alphaRoughness * (1.0 - pbrInfo.anisotropy), EPSILON);
  pbrInfo.aSqr = pbrInfo.aT * pbrInfo.aB;
  // precompute known cos thetas
  pbrInfo.t_dot_v = dot(pbrInfo.anisotropicT, pbrInfo.view);
  pbrInfo.b_dot_v = dot(pbrInfo.anisotropicB, pbrInfo.view);

#endif  // ANISOTROPY_LAYER

/////////////////
// Specular layer support
// Modifies fresnel terms
#if defined(SPECULAR_LAYER)
  pbrInfo.specularWeight = uSpecularLayer.factor;
#if defined(SPECULAR_LAYER_TEXTURE)
  pbrInfo.specularWeight *= texture(uSpecularLayerTexture, texCoord).a;
#endif  // SPECULAR_LAYER_TEXTURE
  // The F0 color of the specular reflection (linear RGB)
  pbrInfo.specularLayerColor = uSpecularLayer.colorFactor;

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)

#if defined(MAP_MAT_TXTRS_TO_LINEAR)
  pbrInfo.specularLayerColor *=
      sRGBToLinear(texture(uSpecularLayerColorTexture, texCoord).rgb);
#else
  pbrInfo.specularLayerColor *=
      texture(uSpecularLayerColorTexture, texCoord).rgb;
#endif  // MAP_MAT_TXTRS_TO_LINEAR

#endif  // SPECULAR_LAYER_COLOR_TEXTURE
#ifndef SKIP_CALC_SPECULAR_LAYER
  // Recalculate dielectric_f0 and specularColor_f90 based on passed specular
  // layer quantities see
  // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_specular
  pbrInfo.dielectric_f0 =
      min(pbrInfo.dielectric_f0 * pbrInfo.specularLayerColor, vec3(1.0)) *
      pbrInfo.specularWeight;
  // specularColor_f90 previously set to vec3(1)
  pbrInfo.specularColor_f90 = mix(vec3(pbrInfo.specularWeight),
                                  pbrInfo.specularColor_f90, pbrInfo.metallic);

#endif  // ifndef SKIP_CALC_SPECULAR_LAYER
#endif  // SPECULAR_LAYER

  // compute specular reflectance (fresnel) at normal incidence
  // for dielectric or metallic, using IOR-derived fresnel reflectance and
  // accounting for specular layer contribution if any is present
  pbrInfo.specularColor_f0 =
      mix(pbrInfo.dielectric_f0, pbrInfo.baseColor.rgb, pbrInfo.metallic);

  return pbrInfo;
}  // buildBasePBRData
