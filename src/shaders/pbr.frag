// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This is an implementation of
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

#if (LIGHT_COUNT > 0)
// -------------- lights -------------------
// NOTE: In this shader, the light intensity is considered in the lightColor!!
uniform vec3 LightColors[LIGHT_COUNT];
uniform float LightRanges[LIGHT_COUNT];

// lights in world space!
// if .w == 0, it means it is a directional light, .xyz is the direction;
// if .w == 1, it means it is a point light, .xyz is the light position;
// it is NOT put in the Light Structure, simply because we may modify the code
// so it is computed in the vertex shader.
uniform vec4 LightDirections[LIGHT_COUNT];
#endif

// -------------- material, textures ------------------
struct MaterialData {
  vec4 baseColor;   // diffuse color, if BaseColorTexture exists,
                    // multiply it with the BaseColorTexture
  float roughness;  // roughness of a surface, if roughness texture exists,
                    // multiply it with the MetallicRoughnessTexture
  float metallic;   // metalness of a surface, if metallic texture exists,
                    // multiply it the MetallicRoughnessTexture
  float ior;  // index of refraction.  Default 1.5 gives DielectricSpecular
              // value 0.04
  vec3 emissiveColor;  // emissiveColor, if emissive texture exists,
                       // multiply it the EmissiveTexture
};
uniform MaterialData Material;
/////////////////
//Clearcoat layer support
#if defined(CLEAR_COAT)
struct ClearCoatData{
  float factor;             // clearcoat factor/intensity. If ClearCoatTexture exists,
                            // multiply it by this factor. If this is 0 the layer is
                            // ignored, as per standard.
  float roughness;          // clearcoat perceived roughness. If ClearCoatRoughnessTexture
                            // exists, multiply it by this roughness value
  float normalTextureScale; // xy scale value for clearcoat normal texture. Multiply
                            // the x,y channels of ClearCoatNormalTexture if exists.
};
uniform ClearCoatData ClearCoat;
#endif //CLEAR_COAT

/////////////////
//Specular layer support
#if defined(SPECULAR_LAYER)
struct SpecularLayerData{
  float factor;     // The strength of the specular reflection.
                    // If SpecularLayerTexture exists, multiply it with this value.
  vec3 colorFactor; // The F0 (Fresnel reflectance at normal incidence) color of
                    // the specular reflection (linear RGB). If SpecularLayerColorTexture exists,
                    // multiply it with this value.
};
uniform SpecularLayerData SpecularLayer;
#endif // SPECULAR_LAYER


#if defined(BASECOLOR_TEXTURE)
uniform sampler2D BaseColorTexture;
#endif
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
uniform sampler2D MetallicRoughnessTexture;
#endif
#if defined(NORMAL_TEXTURE)
uniform sampler2D NormalTexture;
#endif
// TODO: separate occlusion texture
// (if it is not packed with metallicRoughness texture)
#if defined(EMISSIVE_TEXTURE)
uniform sampler2D EmissiveTexture;
#endif

#if defined(CLEAR_COAT_TEXTURE)
uniform sampler2D ClearCoatTexture;
#endif

#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
uniform sampler2D ClearCoatRoughnessTexture;
#endif

#if defined(CLEAR_COAT_NORMAL_TEXTURE)
uniform sampler2D ClearCoatNormalTexture;
#endif

#if defined(SPECULAR_LAYER_TEXTURE)
uniform sampler2D SpecularLayerTexture;
#endif

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)
uniform sampler2D SpecularLayerColorTexture;
#endif


/////////////////
//IBL Support
#if defined(IMAGE_BASED_LIGHTING)
uniform samplerCube IrradianceMap;
uniform sampler2D BrdfLUT;
uniform samplerCube PrefilteredMap;
#endif


// -------------- uniforms ----------------
#if defined(OBJECT_ID)
uniform highp uint ObjectId;
#endif

#if defined(NORMAL_TEXTURE) && defined(NORMAL_TEXTURE_SCALE)
uniform mediump float NormalTextureScale
#ifndef GL_ES
    = 1.0
#endif
    ;
#endif

// camera position in world space
uniform highp vec3 CameraWorldPos;

// scales for components in the PBR equation
// [0] = direct diffuse
// [1] = direct specular
// [2] = ibl diffuse
// [3] = ibl specular
const int DirectDiffuse = 0;
const int DirectSpecular = 1;
#if defined(IMAGE_BASED_LIGHTING)
const int IblDiffuse = 2;
const int IblSpecular = 3;
#endif
uniform highp vec4 ComponentScales;

#if defined(IMAGE_BASED_LIGHTING)
uniform uint PrefilteredMapMipLevels;
#endif

uniform int PbrDebugDisplay;

// -------------- shader ------------------

// The following function Uncharted2Tonemap is based on:
// https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/data/shaders/pbr_khr.frag
vec3 Uncharted2Tonemap(vec3 color) {
  float A = 0.15;
  float B = 0.50;
  float C = 0.10;
  float D = 0.20;
  float E = 0.02;
  float F = 0.30;
  float W = 11.2;
  return ((color * (A * color + C * B) + D * E) /
          (color * (A * color + B) + D * F)) -
         E / F;
}

// TODO: make some of these uniform variables
const float exposure = 4.5f;
const float gamma = 2.2f;

// PI is defined in the pbrCommon.glsl
const float INV_PI = 1.0 / PI;
const float epsilon = 0.000001;

const int maxShadowNum = 3;

// The following function tonemap is based on:
// https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/data/shaders/pbr_khr.frag
// Tone mapping is to take a wide dynamic range of values and compressing them
// into a smaller range that is appropriate for the output device.
vec4 tonemap(vec4 color) {
#ifdef TONE_MAP
  vec3 outcol = Uncharted2Tonemap(color.rgb * exposure);
  outcol = outcol * (1.0f / Uncharted2Tonemap(vec3(11.2f)));
  return vec4(pow(outcol, vec3(1.0f / gamma)), color.a);
#else
  return color;
#endif
}

#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
vec3 getNormalFromNormalMap() {
  vec3 tangentNormal =
#if defined(NORMAL_TEXTURE_SCALE)
      normalize((texture(NormalTexture, texCoord).xyz * 2.0 - 1.0) *
                vec3(NormalTextureScale, NormalTextureScale, 1.0));
#else
      texture(NormalTexture, texCoord).xyz * 2.0 - 1.0;
#endif

#if defined(PRECOMPUTED_TANGENT)
  vec3 T = normalize(tangent);
  vec3 B = normalize(biTangent);
  vec3 N = normalize(normal);
#else
// TODO:
// explore robust screen-space normal mapping withOUT precomputed tangents
#error Normal mapping requires precomputed tangents.
#endif
  // negate the TBN matrix for back-facing primitives
  if (gl_FrontFacing == false) {
    T *= -1.0;
    B *= -1.0;
    N *= -1.0;
  }
  mat3 TBN = mat3(T, B, N);

  // TBN transforms tangentNormal from tangent space to world space
  return normalize(TBN * tangentNormal);
}
#endif  // if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)

// Smith Joint GGX visibility function
// Note: Vis = G / (4 * n_dot_l * n_dot_v)
// see Eric Heitz. 2014. Understanding the Masking-Shadowing Function in
// Microfacet-Based BRDFs.
// https://jcgt.org/published/0003/02/03/paper.pdf
// See also
// https://google.github.io/filament/Filament.md.html#materialsystem/specularbrdf/geometricshadowing(specularg)
// n_dot_l : light projected on normal
// n_dot_v : view projected on normal
// arSq : alphaRoughness squared
float V_GGX(float n_dot_l, float n_dot_v, float arSq) {
  float oneMArSq = (1.0 - arSq);

  float GGXL = n_dot_v * sqrt(n_dot_l * n_dot_l * oneMArSq + arSq);
  float GGXV = n_dot_l * sqrt(n_dot_v * n_dot_v * oneMArSq + arSq);

  return 0.5 / (GGXV + GGXL);
}

// Approximation - mathematically wrong but faster without sqrt
// all the sqrt terms are <= 1
//https://google.github.io/filament/Filament.md.html#listing_approximatedspecularv
// NOTE the argument here is alphaRoughness not alphaRoughness squared
float V_GGXFast(float n_dot_l, float n_dot_v, float ar) {
    float oneMAr = (1.0 - ar);
    float GGXL = n_dot_v * (n_dot_l * oneMAr + ar);
    float GGXV = n_dot_l * (n_dot_v * oneMAr + ar);
    return 0.5 / max(GGXV + GGXL, epsilon);
}

// Approximation for clearcoat visibility from Kelemen et al
// http://www.hungrycat.hu/microfacet.pdf
float V_Kelemen(float v_dot_h){
  return .25 / (v_dot_h * v_dot_h);
}

// The following equation models the distribution of microfacet normals across
// the area being drawn (aka D()) Implementation from "Average Irregularity
// Representation of a Roughened Surface for Ray Reflection" by T. S.
// Trowbridge, and K. P. Reitz Follows the distribution function recommended in
// the SIGGRAPH 2013 course notes from EPIC Games [1], Equation 3.
// n_dot_h : half vector projected on normal
// ar : alphaRoughness
float D_GGX(float n_dot_h, float ar) {
  float arNdotH = ar * n_dot_h;
  float k = ar/(1-(n_dot_h * n_dot_h) + (arNdotH*arNdotH));
  //division by Pi performed later
  return k * k;
}
//Above should be more accurate, with less chance of underflow
float D_GGX_old(float n_dot_h, float ar) {
  float f = (n_dot_h * n_dot_h) * ((ar*ar) - 1.0) + 1.0;
  //division by Pi performed later
  return arSq / (f * f);
}

// Approx 2.5 speedup over pow
float fastPow5(float v){
  float v2 = v * v;
  return v2 * v2 * v;
}

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
vec3 fresnelSchlick(vec3 f0, float f90, float v_dot_h) {
  return f0 + (vec3(f90) - f0) * fastPow5(1.0 - v_dot_h);
}
float fresnelSchlick(float f0, float f90s, float v_dot_h) {
  return f0 + (f90s - f0) * fastPow5(1.0 - v_dot_h);
}
float fresnelSchlick(float f0, float v_dot_h) {
  return fresnelSchlick(f0, 1.0, v_dot_h);
}



//Burley diffuse contribution is scaled by diffuse color f0 after scatter contributions are calculated
const float BUR_DIFF_F0 = 1.0f;
// Burley/Disney diffuse BRDF, from here :
// http://blog.selfshadow.com/publications/s2012-shading-course/burley/s2012_pbs_disney_brdf_notes_v3.pdf
// n_dot_v: <normal, view>
// n_dot_l: <normal, light>
// v_dot_h: <view, halfVector>
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
//     halfVector: half vector of light and view
vec3 BRDF_BurleyDiffuse(vec3 diffuseColor,
                        float n_dot_v,
                        float n_dot_l,
                        float v_dot_h,    // == l_dot_h
                        float alphaRoughness) {
  float fd90 = 0.5f + (2.0 * v_dot_h * v_dot_h * alphaRoughness);
  return fresnelSchlick(BUR_DIFF_F0, fd90, n_dot_l) * fresnelSchlick(BUR_DIFF_F0, fd90, n_dot_v) * diffuseColor;
}//BRDF_BurleyDiffuse


// Empirically derived max energy correcting factor, based on perceptual roughness - see Frostbite engine doc
const float NRG_INTERP_MAX = 0.662251656;//1.0/1.51;
// Burley/Disney diffuse BRDF, from here :
// http://blog.selfshadow.com/publications/s2012-shading-course/burley/s2012_pbs_disney_brdf_notes_v3.pdf
// renormalized for better energy conservation from
// https://media.contentapi.ea.com/content/dam/eacom/frostbite/files/course-notes-moving-frostbite-to-pbr-v32.pdf
// n_dot_v: <normal, view>
// n_dot_l: <normal, light>
// v_dot_h: <view, halfVector>
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
//     halfVector: half vector of light and view
vec3 BRDF_BurleyDiffuseRenorm(vec3 diffuseColor,
                        float n_dot_v,
                        float n_dot_l,
                        float v_dot_h,    // == l_dot_h
                        float alphaRoughness) {
  float energyBias = mix(0.0, 0.5, alphaRoughness);
  float energyFactor = mix(1.0, NRG_INTERP_MAX, alphaRoughness);
  float fd90 = energyBias + (2.0 * v_dot_h * v_dot_h * alphaRoughness);
  return fresnelSchlick(BUR_DIFF_F0, fd90, n_dot_l) * fresnelSchlick(BUR_DIFF_F0, fd90, n_dot_v) * energyFactor * diffuseColor;
}//BRDF_BurleyDiffuseRenorm

// Calculate the specular contribution
//  https://github.com/KhronosGroup/glTF/tree/master/specification/2.0#acknowledgments
//  AppendixB
// fresnel : Schlick approximation of Fresnel coefficient
// alphaRoughness: roughness of the surface (perceived roughness squared)
// n_dot_l: <normal, light>
// n_dot_v: <normal, view>
// n_dot_h: <normal, halfVector>
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
//     halfVector: half vector of light and view
vec3 BRDF_Specular(vec3 fresnel,
                   float alphaRoughness,
                   float n_dot_l,
                   float n_dot_v,
                   float n_dot_h) {
  vec3 specular = fresnel *
                  // Visibility function == G()/4(n_dot_l)(n_dot_v).
                  // Smith Height-Correlated visibility/occlusion function
                  // https://jcgt.org/published/0003/02/03/paper.pdf
                  V_GGX(n_dot_l, n_dot_v, alphaRoughness * alphaRoughness) *
                  // Specular D, normal distribution function (NDF),
                  // also known as ggxDistribution
                  D_GGX(n_dot_h, alphaRoughness);
  return specular;
}//BRDF_Specular


#if defined(CLEAR_COAT)
// Calculate clearcoat specular contribution
// ccFresnel : clearCoat-scaled specular f0 contribution of polyurethane coating
// clearCoatRoughness : clearCoatPerceptualRoughness squared
// v_dot_h: <view, halfVector> view projected on half vector
// cc_n_do_h : clearcoat normal vector dotted to half vector
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
//     halfVector: half vector of light and view
vec3 BRDF_ClearCoatSpecular(vec3 ccFresnel,
                    float clearCoatRoughness,
                    float v_dot_h,
                   float cc_n_do_h) {
  //ccFresnel term is using polyurethane f0 = vec3(0.4)
  vec3 ccSpecular = ccFresnel *
                  // Visibility function == G()/4(n_dot_l)(n_dot_v).
                  // Smith Height-Correlated visibility/occlusion function
                  // https://jcgt.org/published/0003/02/03/paper.pdf
                   V_Kelemen(v_dot_h) *
                  // Specular D, normal distribution function (NDF),
                  // also known as ggxDistribution, using clearcoat roughness
                  D_GGX(cc_n_do_h, clearCoatRoughness);
  return ccSpecular;
}//BRDF_ClearCoatSpecular
#endif // clearcoat support


#if defined(IMAGE_BASED_LIGHTING)
// diffuseColor: diffuse color
// n: normal on shading location in world space
vec3 computeIBLDiffuse(vec3 diffuseColor, vec3 n) {
  // diffuse part = diffuseColor * irradiance
  // return diffuseColor * texture(IrradianceMap, n).rgb * Scales.iblDiffuse;
  return diffuseColor * tonemap(texture(IrradianceMap, n)).rgb *
         ComponentScales[IblDiffuse];
}

vec3 computeIBLSpecular(float roughness,
                        float n_dot_v,
                        vec3 specularReflectance,
                        vec3 reflectionDir) {
  vec3 brdf = texture(BrdfLUT, vec2(max(n_dot_v, 0.0), 1.0 - roughness)).rgb;
  float lod = roughness * float(PrefilteredMapMipLevels);
  vec3 prefilteredColor =
      tonemap(textureLod(PrefilteredMap, reflectionDir, lod)).rgb;

  return prefilteredColor * (specularReflectance * brdf.x + brdf.y) *
         ComponentScales[IblSpecular];
}
#endif






void main() {
///////////////////////

// normal map will only work if both normal texture and the tangents exist.
// if only normal texture is set, normal mapping will be safely ignored.
// n is the normal in *world* space, NOT camera space
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
  // normal is now in the camera space
  vec3 n = getNormalFromNormalMap();
#else
  vec3 n = normalize(normal);
  // This means backface culling is disabled,
  // which implies it is rendering with the "double sided" material.
  // Based on glTF 2.0 Spec, the normal must be reversed for back faces
  if (gl_FrontFacing == false) {
    n *= -1.0;
  }
#endif


  vec3 emissiveColor = Material.emissiveColor;
#if defined(EMISSIVE_TEXTURE)
  emissiveColor *= texture(EmissiveTexture, texCoord).rgb;
#endif
  fragmentColor = vec4(emissiveColor, 0.0);

#if (LIGHT_COUNT > 0)
  vec4 baseColor = Material.baseColor;
#if defined(BASECOLOR_TEXTURE)
  baseColor *= texture(BaseColorTexture, texCoord);
#endif

/////////////////
// Initialization\
  // Index of refraction 1.5 yields 0.04 dielectric fresnel reflectance at normal incidence
  float ior = Material.ior;

/////////////////
//Roughness calc

  float perceivedRoughness = Material.roughness;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
  perceivedRoughness *= texture(MetallicRoughnessTexture, texCoord).g;
#endif
  // clamp roughness to prevent dennormals in distribution function calc
  perceivedRoughness = clamp(perceivedRoughness, 0.045, 1.0);
  // Roughness is authored as perceptual roughness by convention,
  // convert to more linear roughness mapping by squaring the perceptual roughness.
  float alphaRoughness = perceivedRoughness * perceivedRoughness;


/////////////////
//Metalness calc

float metallic = Material.metallic;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
  metallic *= texture(MetallicRoughnessTexture, texCoord).b;
#endif

/////////////////
// Diffuse color calc

  // diffuse color remapping based on metallic value (aka c_diff)
  // https://google.github.io/filament/Filament.md.html#listing_basecolortodiffuse
  vec3 diffuseColor =  baseColor.rgb * (1 - metallic);



  //TODO : diffuse color is darkened by clearcoat.
  //https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_clearcoat

/////////////////
//Clearcoat layer support

#if defined(CLEAR_COAT)
  //TODO if clearCoatStrength is 0 then ignore clearcoat
  float clearCoatStrength = ClearCoat.factor;
#if defined(CLEAR_COAT_TEXTURE)
  clearCoatStrength *= texture(ClearCoatTexture, texCoord).r;
#endif

  float clearCoatPerceivedRoughness = ClearCoat.roughness;
#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
  clearCoatPerceivedRoughness *= texture(ClearCoatRoughnessTexture, texCoord).g;
#endif
  // clamp clearcoat roughness to prevent dennormals in distribution function calc
  clearCoatPerceivedRoughness = clamp(clearCoatPerceivedRoughness, 0.045, 1.0);
  float clearCoatRoughness = clearCoatPerceivedRoughness * clearCoatPerceivedRoughness;


  // TODO
  //Calculate clearcoat contributions based on an IOR_cc == 1.5
  //since clearcoat affects underlying diffuse material

  //
  // If clearcoatNormalTexture is not given, no normal mapping is applied to the clear
  // coat layer, even if normal mapping is applied to the base material. Otherwise,
  // clearcoatNormalTexture may be a reference to the same normal map used by the
  // base material, or any other compatible normal map.

  vec3 clearCoatNormal = n;
  // TODO Need to explore this
#if defined(CLEAR_COAT_NORMAL_TEXTURE)
    // // NEED tangent frame
  // vec3 clearcoatMapN =
  //  normalize((texture(ClearCoatNormalTexture, texCoord).xyz * 2.0 - 1.0) *
  //               vec3(ClearCoat.normalTextureScale, ClearCoat.normalTextureScale, 1.0));
    // clearcoatNormal = normalize( tbn * clearcoatMapN );
#endif

  // Assuming dielectric reflectance of 0.4, which corresponds to
  // polyurethane coating
  float clearCoatCoating_f0 = 0.4;
  // TODO : need to use this to modify the base dielectric reflectance,
  // since it darkens the reflectance of the underlying material

#endif // CLEAR_COAT



/////////////////
//Specular layer support
  //Modifies fresnel terms

  // DielectricSpecular == 0.04 <--> ior == 1.5
  // If clearcoat is present, may modify IOR
  float DielectricSpecular = pow(((ior - 1) / (ior + 1)), 2);

  // Achromatic dielectric material f0 : fresnel reflectance at normal incidence
  // based on given IOR
  vec3 dielectric_f0 = vec3(DielectricSpecular);

  // glancing incidence specular reflectance
  float specularColor_f90 = 1.0;

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
  // Recalculate specularColor_f0 and specularColor_f90 based on passed quantities
  // see https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_specular
  dielectric_f0 = min(dielectric_f0 *specularLayerColor, vec3(1.0)) * specularWeight;

  specularColor_f90 = mix(specularWeight, 1.0, metallic);

#endif // SPECULAR_LAYER
  // compute specular reflectance (fresnel) at normal incidence
  // for dielectric or metallic, using IOR-derived fresnel reflectance
  // accounting for specular layers
  vec3 specularColor_f0 = mix(dielectric_f0, baseColor.rgb, metallic);



/////////////////
// vectors
  // View is the normalized vector from the shading location to the camera
  // in *world space*
  vec3 view = normalize(CameraWorldPos - position);

  // view projected on normal
  float n_dot_v = abs(dot(n, view));

  //Contributions for diffuse and specular
  vec3 diffuseContrib = vec3(0.0, 0.0, 0.0);
  vec3 specularContrib = vec3(0.0, 0.0, 0.0);

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
    // avoid a NaN when dist is 0 as well (which is the case for
    // directional lights)
    // https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_lights_punctual/README.md#range-property
    // Attenuation is 1 for directional lights, governed by inverse square law
    // otherwise
    highp float attenuation =
        clamp(1 - pow(dist / max(LightRanges[iLight], epsilon), 4.0), 0.0, 1.0) /
        sqDist;

    //if color is not visible, skip contribution
    if (attenuation == 0){
      continue;
    }
    // radiant intensity
    vec3 lightRadiance = LightColors[iLight] * attenuation;

    // light source direction: a vector from the shading location to the
    // light
    vec3 light = normalize(toLightVec);
    // projection of normal to light
    // clamp to small epsilon
    float n_dot_l = clamp(dot(n, light), 0.0, 1.0);

    // if (n_dot_l > 0.0) {
    // halfway between the light and view vector
    vec3 halfVector = normalize(light + view);
    // projection of view on halfway vector
    float v_dot_h = clamp(dot(view, halfVector), 0.0, 1.0);
    // project of normal on halfway vector
    float n_dot_h = clamp(dot(n, halfVector), 0.0, 1.0);


    // if light can hit location
    // Radiance scaled by incident angle cosine, per area
    vec3 projLightRadiance = lightRadiance * n_dot_l;
    // Calculate the Schlick approximation of the Fresnel coefficient
    // Fresnel Specular color at view angle == Schlick approx using view angle
    vec3 fresnel = fresnelSchlick(specularColor_f0, specularColor_f90, v_dot_h);

    // Lambertian diffuse contribution
    // currentDiffuseContrib =
    //     projLightRadiance * INV_PI * diffuseColor;

    // Burley/Disney diffuse contribution
    vec3 currentDiffuseContrib =
        projLightRadiance * INV_PI *
        BRDF_BurleyDiffuse(diffuseColor, n_dot_v, n_dot_l, v_dot_h,
                           alphaRoughness);
    // Specular microfacet - 1/pi from specular D normal dist function
    vec3 currentSpecularContrib = projLightRadiance * INV_PI *
                             BRDF_Specular(fresnel, alphaRoughness, n_dot_l,
                                           n_dot_v, n_dot_h);

#if defined(CLEAR_COAT)
    // scalar clearcoat contribution
    float ccFesnel = fresnelSchlick(clearCoatCoating_f0, v_dot_h) * clearCoatStrength;



#endif // CLEAR_COAT

#if defined(SHADOWS_VSM)
    float shadow =
        (iLight < maxShadowNum
             ? computeShadowVSM(iLight, position, LightDirections[iLight].xyz)
             : 1.0f);
    currentDiffuseContrib *= shadow;
    currentSpecularContrib *= shadow;
#endif
    //}  // for lights with non-transmissive surfaces

    // Transmission here
    diffuseContrib += currentDiffuseContrib;
    specularContrib += currentSpecularContrib;

  }  // for lights

  diffuseContrib *= ComponentScales[DirectDiffuse];
  specularContrib *= ComponentScales[DirectSpecular];

  // TODO: use ALPHA_MASK to discard fragments
  fragmentColor += vec4(diffuseContrib + specularContrib, baseColor.a);
#endif  // if (LIGHT_COUNT > 0)

#if defined(IMAGE_BASED_LIGHTING)
  vec3 iblDiffuseContrib = computeIBLDiffuse(diffuseColor, n);
  fragmentColor.rgb += iblDiffuseContrib;

  vec3 reflection = normalize(reflect(-view, n));
  vec3 iblSpecularContrib =
      computeIBLSpecular(perceivedRoughness, n_dot_v, specularColor_f0, reflection);
  fragmentColor.rgb += iblSpecularContrib;
#endif  // IMAGE_BASED_LIGHTING

#if defined(OBJECT_ID)
  fragmentObjectId = ObjectId;
#endif
// float specCCBlue = 0.0f;
// #if defined(CLEAR_COAT_ROUGHNESS_TEXTURE) && defined(CLEAR_COAT_TEXTURE)
//   specCCBlue = 1.0f;
//   if (((int(10 * texCoord.x)) % 2) == ((int(10 * texCoord.y)) % 2 )){
//   fragmentColor = vec4(texture(ClearCoatTexture, texCoord).r, 0.0, 0.0, 1.0);

//   }  else {
//   fragmentColor = vec4(0.0, texture(ClearCoatRoughnessTexture, texCoord).g, 0.0, 1.0);

//   }
//   //fragmentColor = vec4(0.0, texture(ClearCoatRoughnessTexture, texCoord).g, 0.0, 1.0);
// #endif

// #if defined(SPECULAR_LAYER_TEXTURE) && defined(SPECULAR_LAYER_COLOR_TEXTURE)
//   if (((int(10 * texCoord.x)) % 2) == ((int(10 * texCoord.y)) % 2 )){
//   fragmentColor = vec4(texture(SpecularLayerTexture, texCoord).a, 0.0, specCCBlue, 1.0);

//   }  else {
//   fragmentColor = vec4(texture(SpecularLayerColorTexture, texCoord).rgb, 1.0);

//   }
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
