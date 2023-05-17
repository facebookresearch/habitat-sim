// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;


#if (LIGHT_COUNT > 0)
/////////////////////
// Structure holding light information to facilitate passing as
// function arguments
struct LightInfo{
  // normalize vector to light from illumination point
  vec3 light;
  // distance-attenuated light color vector
  vec3 lightRadiance;
  // cos angle between normal and view
  float n_dot_v;
  // cos angle between normal and light
  float n_dot_l;
  // normalized vector halfway between the light and view vector
  vec3 halfVector;
  // cos angle between view and halfway vector
  float v_dot_h;
  // cos angle between normal and halfway vector
  float n_dot_h;
  // Radiance scaled by incident angle cosine
  vec3 projLightRadiance;
};

// Configure a LightInfo object
// light : normalized point to light vector
// lightRadiance : distance-attenuated light radiance color
// n : normal
// view : normalized view vector (point to camera)
// n_dot_v : cos angle between n and view
// (out) l : LightInfo structure being popualted
void configureLightInfo(
  vec3 light,
  vec3 lightRadiance,
  vec3 n,
  vec3 view,
  float n_dot_v,
  out LightInfo l){
    l.light = light;
    l.lightRadiance = lightRadiance;
    l.n_dot_v = n_dot_v;
    l.n_dot_l = clamp(dot(n, light), 0.0, 1.0);
    l.halfVector = normalize(light + view);
    l.v_dot_h = clamp(dot(view, l.halfVector), 0.0, 1.0),
    l.n_dot_h = clamp(dot(n, l.halfVector), 0.0, 1.0),
    l.projLightRadiance = lightRadiance * l.n_dot_l;
} // configureLightInfo
#endif // (LIGHT_COUNT > 0)

// Fresnel specular coefficient at view angle using Schlick approx
// http://graphicrants.blogspot.com/2013/08/specular-brdf-reference.html
// https://github.com/wdas/brdf/tree/master/src/brdfs
// https://google.github.io/filament/Filament.md.html
//
// The following equation models the Fresnel reflectance term of the spec
// equation (aka F())
// f0 : specular color reflectance at normal incidence
// v_dot_h: <view, halfVector> view projected on half vector
//          view: camera direction, aka light outgoing direction
//          halfVector: half vector of light and view
vec3 fresnelSchlick(vec3 f0, vec3 f90, float v_dot_h) {
  return f0 + (f90 - f0) * pow5(1.0 - v_dot_h);
}
float fresnelSchlick(float f0, float f90s, float v_dot_h) {
  return f0 + (f90s - f0) * pow5(1.0 - v_dot_h);
}
float fresnelSchlick(float f0, float v_dot_h) {
  return fresnelSchlick(f0, 1.0, v_dot_h);
}


#if defined(ANISOTROPY_LAYER)
//////////////////////////
// Structure holding anisotropy info to faciliate passing as function arguments
struct AnisotropyInfo{
  // Tangent-space direction of anisotropy
  vec2 dir;
  // Strength of anisotropy
  float anisotropy;
  // anisotropic roughness value along tangent direction
  float aT;
  // anisotropic roughness value along bitangent direction
  float aB;
  // Tangent vector in world
  vec3 anisotropicT;
  // Bitangent vector in world
  vec3 anisotropicB;
  // cos angle between tangent and view
  float t_dot_v;
  // cos angle between bitangent and view
  float b_dot_v;

};
// Configure an AnisotropyInfo object
// dir : the tangent-space direction of the anisotropy
// anisotropy : strength of anisotropy
// anisotropicT : world space tangent
// anisotropicB : world space bitangent
// view : view vector
// alphaRoughness : linearized material roughness (perceived roughness squared)
// info : AnisotropyInfo structure to be populated
void configureAnisotropyInfo(
  vec2 dir,
  float anisotropy,
  vec3 anisotropicT,
  vec3 anisotropicB,
  vec3 view,
  float alphaRoughness,
  out AnisotropyInfo info){
    info.dir = dir;
    info.anisotropy = anisotropy;
    // Derive roughness in each direction
    // From standard https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#implementation
    // info.aT = mix(alphaRoughness, 1.0, anisotropy*anisotropy);
    // info.aB = alphaRoughness;
    // From https://google.github.io/filament/Filament.md.html#materialsystem/anisotropicmodel/anisotropicspecularbrdf
    // If anisotropy == 0 then just alpharoughness in each direction
    info.aT = max(alphaRoughness * (1.0 + anisotropy), epsilon);
    info.aB = max(alphaRoughness * (1.0 - anisotropy), epsilon);

    info.anisotropicT = anisotropicT;
    info.anisotropicB = anisotropicB;
    info.t_dot_v = dot(anisotropicT, view);
    info.b_dot_v = dot(anisotropicB, view);

} // configureAnisotropyInfo

#endif


#if (LIGHT_COUNT > 0)

// Smith Joint GGX visibility function
// Note: Vis = G / (4 * n_dot_l * n_dot_v)
// see Eric Heitz. 2014. Understanding the Masking-Shadowing Function in
// Microfacet-Based BRDFs.
// https://jcgt.org/published/0003/02/03/paper.pdf
// See also
// https://google.github.io/filament/Filament.md.html#materialsystem/specularbrdf/geometricshadowing(specularg)
// l : LightInfo structure describing current light
// arSq : alphaRoughness squared
float V_GGX(LightInfo l, float arSq) {
  float oneMArSq = (1.0 - arSq);

  float GGXL = l.n_dot_v * sqrt(l.n_dot_l * l.n_dot_l * oneMArSq + arSq);
  float GGXV = l.n_dot_l * sqrt(l.n_dot_v * l.n_dot_v * oneMArSq + arSq);

  return 0.5 / max(GGXV + GGXL, epsilon);
}

// Approximation - mathematically wrong but faster without sqrt
// all the sqrt terms are <= 1
//https://google.github.io/filament/Filament.md.html#listing_approximatedspecularv
// NOTE the argument here is alphaRoughness not alphaRoughness squared
// l : LightInfo structure describing current light
float V_GGXFast(LightInfo l, float ar) {
    float oneMAr = (1.0 - ar);
    float GGXL = l.n_dot_v * (l.n_dot_l * oneMAr + ar);
    float GGXV = l.n_dot_l * (l.n_dot_v * oneMAr + ar);
    return 0.5 / max(GGXV + GGXL, epsilon);
}

// Approximation for clearcoat visibility from Kelemen et al
// http://www.hungrycat.hu/microfacet.pdf
float V_Kelemen(float v_dot_h){
  return .25 / (v_dot_h * v_dot_h);
}


#if defined(ANISOTROPY_LAYER)
//////////////////////////
// Structure holding anisotropy-related lighting info to faciliate passing as function args
struct AnistropyDirectLight{
  // cos angle between tangent and light
  float t_dot_l;
  // cos angle between tangent and halfvector
  float t_dot_h;
  // cos angle between bitangent and light
  float b_dot_l;
  // cos angle between bitangent and halfvector
  float b_dot_h;
};

// Configure a light-dependent AnistropyDirectLight object
// aInfo : AnisotropyInfo object for current material
// l : LightInfo structure for current light
// (out) info : AnisotropyInfo structure to be populated
void configureAnisotropyLightInfo(
  LightInfo l,
  AnisotropyInfo aInfo,
  out AnistropyDirectLight info){
    info.t_dot_l = dot(aInfo.anisotropicT, l.light);
    info.b_dot_l = dot(aInfo.anisotropicB, l.light);
    info.t_dot_h = dot(aInfo.anisotropicT, l.halfVector);
    info.b_dot_h = dot(aInfo.anisotropicB, l.halfVector);

} // configureAnisotropyLightInfo

// Anisotropic Visibility function
// l : LightInfo structure describing current light
// anisoInfo : AnisotropyInfo structure for material
// anisoLightInfo : Light-specific anisotropic tangent and bitangent cosines
float V_GGX_anisotropic(
      LightInfo l,
      AnisotropyInfo anisoInfo,
      AnistropyDirectLight anisoLightInfo){
    float GGXL = l.n_dot_v * length(vec3(anisoInfo.aT * anisoLightInfo.t_dot_l, anisoInfo.aB * anisoLightInfo.b_dot_l, l.n_dot_l));
    float GGXV = l.n_dot_l * length(vec3(anisoInfo.aT * anisoInfo.t_dot_v, anisoInfo.aB * anisoInfo.b_dot_v, l.n_dot_v));
    return 0.5 / max(GGXV + GGXL, epsilon);
}
// Anisotropic microfacet distribution model
// l : LightInfo structure describing current light
// anisoInfo : AnisotropyInfo structure for material
// anisoLightInfo : Light-specific anisotropic tangent and bitangent cosines
float D_GGX_anisotropic(
      LightInfo l,
      AnisotropyInfo anisoInfo,
      AnistropyDirectLight anisoLightInfo){
    float a2 = anisoInfo.aT * anisoInfo.aB;
    vec3 f = vec3(anisoInfo.aB * anisoLightInfo.t_dot_h, anisoInfo.aT * anisoLightInfo.b_dot_h, a2 * l.n_dot_h);
    float w2 = a2 / dot(f, f);
    //division by Pi performed later
    return a2 * w2 * w2;
}

#endif //ANISOTROPY_LAYER

#endif //(LIGHT_COUNT > 0)

// The following equation models the distribution of microfacet normals across
// the area being drawn (aka D()) Implementation from "Average Irregularity
// Representation of a Roughened Surface for Ray Reflection" by T. S.
// Trowbridge, and K. P. Reitz Follows the distribution function recommended in
// the SIGGRAPH 2013 course notes from EPIC Games [1], Equation 3.
// n_dot_h : cos angle between half vector and normal
// ar : alphaRoughness
float D_GGX(float n_dot_h, float ar) {
  float arNdotH = ar * n_dot_h;
  float k = ar/(1-(n_dot_h * n_dot_h) + (arNdotH*arNdotH));
  //division by Pi performed later
  return k * k;
}
//Above should be more accurate, with less chance of underflow
float D_GGX_old(float n_dot_h, float ar) {
  float arSq = ar * ar;
  float f = (n_dot_h * n_dot_h) * (arSq - 1.0) + 1.0;
  //division by Pi performed later
  return arSq / (f * f);
}



#if (LIGHT_COUNT > 0)
//Burley diffuse contribution is scaled by diffuse color f0 after scatter contributions are calculated
const float BUR_DIFF_F0 = 1.0f;
// Burley/Disney diffuse BRDF, from here :
// http://blog.selfshadow.com/publications/s2012-shading-course/burley/s2012_pbs_disney_brdf_notes_v3.pdf
// diffuseColor : metallness-scaled basedcolor
// l : LightInfo structure describing current light
// alphaRoughness : roughness (perceived roughness squared)
vec3 BRDF_BurleyDiffuse(vec3 diffuseColor,
                        LightInfo l,
                        float alphaRoughness) {
  float fd90 = 0.5f + (2.0 * l.v_dot_h * l.v_dot_h * alphaRoughness);
  return fresnelSchlick(BUR_DIFF_F0, fd90, l.n_dot_l) * fresnelSchlick(BUR_DIFF_F0, fd90, l.n_dot_v) * diffuseColor;
}//BRDF_BurleyDiffuse


// Empirically derived max energy correcting factor, based on perceptual roughness - see Frostbite engine doc
const float NRG_INTERP_MAX = 0.662251656;//1.0/1.51;
// Burley/Disney diffuse BRDF, from here :
// http://blog.selfshadow.com/publications/s2012-shading-course/burley/s2012_pbs_disney_brdf_notes_v3.pdf
// renormalized for better energy conservation from
// https://media.contentapi.ea.com/content/dam/eacom/frostbite/files/course-notes-moving-frostbite-to-pbr-v32.pdf
// diffuseColor : metallness-scaled basedcolor
// l : LightInfo structure describing current light
// alphaRoughness : roughness (perceived roughness squared)
vec3 BRDF_BurleyDiffuseRenorm(vec3 diffuseColor,
                        LightInfo l,
                        float alphaRoughness) {
  float energyBias = mix(0.0, 0.5, alphaRoughness);
  float energyFactor = mix(1.0, NRG_INTERP_MAX, alphaRoughness);
  float fd90 = energyBias + (2.0 * l.v_dot_h * l.v_dot_h * alphaRoughness);
  return fresnelSchlick(BUR_DIFF_F0, fd90, l.n_dot_l) * fresnelSchlick(BUR_DIFF_F0, fd90, l.n_dot_v) * energyFactor * diffuseColor;
}//BRDF_BurleyDiffuseRenorm

// Calculate the specular contribution
//  https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#acknowledgments
//  AppendixB
// fresnel : Schlick approximation of Fresnel coefficient
// l : LightInfo structure describing current light
// alphaRoughness: roughness of the surface (perceived roughness squared)
vec3 BRDF_Specular(vec3 fresnel,
                  LightInfo l,
                  float alphaRoughness) {
  vec3 specular = fresnel *
                  // Visibility function == G()/4(n_dot_l)(n_dot_v).
                  // Smith Height-Correlated visibility/occlusion function
                  // https://jcgt.org/published/0003/02/03/paper.pdf
                  V_GGX(l, alphaRoughness * alphaRoughness) *
                  // Specular D, normal distribution function (NDF),
                  // also known as ggxDistribution
                  D_GGX(l.n_dot_h, alphaRoughness);
  return specular;
}//BRDF_Specular


#if defined(CLEAR_COAT)
// Calculate clearcoat specular contribution
// ccFresnel : clearCoat-scaled specular f0 contribution of polyurethane coating
// cc_l : LightInfo structure describing current light using CC normal.
// clearCoatRoughness : clearCoatPerceptualRoughness squared
vec3 BRDF_ClearCoatSpecular(vec3 ccFresnel,
                  LightInfo cc_l,
                  float clearCoatRoughness) {
  //ccFresnel term is using polyurethane f0 = vec3(0.4)
  vec3 ccSpecular = ccFresnel *
                  // Visibility function == G()/4(n_dot_l)(n_dot_v).
                  // Smith Height-Correlated visibility/occlusion function
                  // https://jcgt.org/published/0003/02/03/paper.pdf
                   V_Kelemen(cc_l.v_dot_h) *
                  // Specular D, normal distribution function (NDF),
                  // also known as ggxDistribution, using clearcoat roughness
                  D_GGX(cc_l.n_dot_h, clearCoatRoughness);
  return ccSpecular;
}//BRDF_ClearCoatSpecular
#endif // clearcoat support

#if defined(ANISOTROPY_LAYER)
// Specular BRDF for anisotropic layer
// fresnel : Schlick approximation of Fresnel coefficient
// l : LightInfo structure describing current light
// anisoInfo : AnisotropyInfo structure for material
// anisoLightInfo : Light-specific anisotropic tangent and bitangent cosines
vec3 BRDF_specularAnisotropicGGX(
      vec3 fresnel,
      LightInfo l,
      AnisotropyInfo anisoInfo,
      AnistropyDirectLight anisoLightInfo){
    // Using base material fresnel
    vec3 anisoSpecular = fresnel *
            //Visibility term
            V_GGX_anisotropic(l, anisoInfo, anisoLightInfo) *
            //microfacet normal distribution
            D_GGX_anisotropic(l, anisoInfo, anisoLightInfo);

    return anisoSpecular;
} // BRDF_specularAnisotropicGGX

#endif // ANISOTROPY_LAYER

#endif // (LIGHT_COUNT > 0)
