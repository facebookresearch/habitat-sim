// Copyright (c) Facebook, Inc. and its affiliates.
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
  vec4 baseColor;      // diffuse color, if BaseColorTexture exists,
                       // multiply it with the BaseColorTexture
  float roughness;     // roughness of a surface, if roughness texture exists,
                       // multiply it with the MetallicRoughnessTexture
  float metallic;      // metalness of a surface, if metallic texture exists,
                       // multiply it the MetallicRoughnessTexture
  vec3 emissiveColor;  // emissiveColor, if emissive texture exists,
                       // multiply it the EmissiveTexture
};
uniform MaterialData Material;

#if defined(BASECOLOR_TEXTURE)
uniform sampler2D BaseColorTexture;
#endif
#if defined(METALLIC_TEXTURE) || defined(ROUGHNESS_TEXTURE)
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

// TODO: make them uniform variables
const float exposure = 4.5f;
const float gamma = 2.2f;

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
#endif

// PI is defined in the pbrCommon.glsl
const float INV_PI = 1.0 / PI;
const float Epsilon = 0.0001;
const float DielectricSpecular = 0.04;

// helper function to compute the Specular G
float geometrySchlickGGX(float dotProd, float roughness) {
  float r = (roughness + 1.0);
  float k = (r * r) / 8.0;
  float d = dotProd * (1.0 - k) + k;

  return dotProd / d;
}

// Specular G, specular geometric attenuation
// n_dot_l: <normal, light>
// n_dot_v: <normal, view>
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
float specularGeometricAttenuation(float n_dot_l,
                                   float n_dot_v,
                                   float roughness) {
  float ggx1 = geometrySchlickGGX(n_dot_l, roughness);
  float ggx2 = geometrySchlickGGX(n_dot_v, roughness);

  return ggx1 * ggx2;
}

// Specular F, aka Fresnel, use Schlick's approximation
// specularReflectance: specular reflectance at normal incidence
// v_dot_h: <view, halfVector>
//          view: camera direction, aka light outgoing direction
//          halfVector: half vector of light and view
vec3 fresnelSchlick(vec3 specularReflectance, float v_dot_h) {
  // https://github.com/SaschaWillems/Vulkan-glTF-PBR
  // For typical incident reflectance range (between 4% to 100%)
  // set the grazing reflectance to 100% for typical fresnel effect.
  // For very low reflectance range on highly diffuse objects (below 4%),
  // incrementally reduce grazing reflecance to 0%.
  float reflectance = max(max(specularReflectance.r, specularReflectance.g),
                          specularReflectance.b);
  float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
  return specularReflectance +
         (vec3(reflectance90) - specularReflectance) * pow(1.0 - v_dot_h, 5.0);
}

// specularReflectance: specular reflectance at normal incidence
//     for nonmetal, using constant 0.04
// c_diff: diffuse color
// metallic: metalness of the surface
// roughness: roughness of the surface
// v_dot_h: <view, halfVector>
// n_dot_l: <normal, light>
// n_dot_v: <normal, view>
// n_dot_h: <normal, halfVector>
//     normal: normal direction
//     light: light source direction
//     view: camera direction, aka light outgoing direction
//     halfVector: half vector of light and view
// lightRadiance: the radiance of the light,
//                which equals to intensity * attenuation
// output:
// diffuseContrib: the contribution of the direct diffuse
// specularContrib: the contribution of the direct specular
void microfacetModel(vec3 specularReflectance,
                     vec3 c_diff,
                     float metallic,
                     float roughness,
                     float v_dot_h,
                     float n_dot_l,
                     float n_dot_v,
                     float n_dot_h,
                     vec3 lightRadiance,
                     out vec3 diffuseContrib,
                     out vec3 specularContrib) {
  vec3 Fresnel = fresnelSchlick(specularReflectance, v_dot_h);
  // Diffuse BRDF
  // NOTE: energy conservation requires
  // diffuse + specular <= 1.0, where specular = Fresnel
  vec3 diffuse = (vec3(1.0) - Fresnel) * c_diff * INV_PI;

  // Specular BRDF
  float temp = max(4.0 * n_dot_l * n_dot_v, Epsilon);
  vec3 specular = Fresnel *
                  specularGeometricAttenuation(n_dot_l, n_dot_v, roughness) *
                  // normalDistributionGGX is defined in the pbrCommon.glsl
                  // Specular D, normal distribution function (NDF),
                  // also known as ggxDistribution
                  normalDistributionGGX(n_dot_h, roughness) / temp;

  vec3 tempVec = lightRadiance * n_dot_l;
  diffuseContrib = diffuse * tempVec;
  specularContrib = specular * tempVec;
}

#if defined(IMAGE_BASED_LIGHTING)
// c_diff: diffuse color
// n: normal on shading location in world space
vec3 computeIBLDiffuse(vec3 c_diff, vec3 n) {
  // diffuse part = c_diff * irradiance
  // return c_diff * texture(IrradianceMap, n).rgb * Scales.iblDiffuse;
  return c_diff * tonemap(texture(IrradianceMap, n)).rgb *
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

  float roughness = Material.roughness;
#if defined(ROUGHNESS_TEXTURE)
  roughness *= texture(MetallicRoughnessTexture, texCoord).g;
#endif

  float metallic = Material.metallic;
#if defined(METALLIC_TEXTURE)
  metallic *= texture(MetallicRoughnessTexture, texCoord).b;
#endif

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

  // view is the normalized vector from the shading location to the camera
  // in *world space*
  vec3 view = normalize(CameraWorldPos - position);

  // compute specularReflectance, specular reflectance at normal incidence
  // for nonmetal, using constant 0.04
  vec3 specularReflectance =
      mix(vec3(DielectricSpecular), baseColor.rgb, metallic);

  // diffuse color (c_diff in gltf 2.0 spec:
  // https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#metal-brdf-and-dielectric-brdf)
  // c_diff = lerp(baseColor.rgb * (1 - dielectricSpecular), black, metallic)
  vec3 c_diff = baseColor.rgb * (1.0 - DielectricSpecular) * (1.0 - metallic);
  float n_dot_v = clamp(dot(n, view), 0.001, 1.0);

  vec3 diffuseContrib = vec3(0.0, 0.0, 0.0);
  vec3 specularContrib = vec3(0.0, 0.0, 0.0);

  const int maxShadowNum = 3;
  // compute contribution of each light using the microfacet model
  // the following part of the code is inspired by the Phong.frag in Magnum
  // library (https://magnum.graphics/)
  for (int iLight = 0; iLight < LIGHT_COUNT; ++iLight) {
    // Attenuation. Directional lights have the .w component set to 0, use
    // that to make the distance zero -- which will then ensure the
    // attenuation is always 1.0
    highp float dist = length(LightDirections[iLight].xyz - position) *
                       LightDirections[iLight].w;
    // If range is 0 for whatever reason, clamp it to a small value to
    // avoid a NaN when dist is 0 as well (which is the case for
    // directional lights).
    highp float attenuation = clamp(
        1.0 - pow(dist / max(LightRanges[iLight], 0.0001), 4.0), 0.0, 1.0);
    attenuation = attenuation * attenuation / (1.0 + dist * dist);

    // radiance
    vec3 lightRadiance = LightColors[iLight] * attenuation;

    // light source direction: a vector from the shading location to the light
    vec3 light = normalize(LightDirections[iLight].xyz -
                           position * LightDirections[iLight].w);
    /*
    void microfacetModel(vec3 specularReflectance,
                         vec3 c_diff,
                         float metallic,
                         float roughness,
                         float v_dot_h,
                         float n_dot_l,
                         float n_dot_v,
                         float n_dot_h,
                         vec3 lightRadiance,
                         out vec3 diffuseContrib,
                         out vec3 specularContrib);
    */
    vec3 halfVector = normalize(light + view);
    float v_dot_h = clamp(dot(view, halfVector), 0.0, 1.0);
    float n_dot_l = clamp(dot(n, light), 0.001, 1.0);
    float n_dot_h = clamp(dot(n, halfVector), 0.0, 1.0);
    vec3 currentDiffuseContrib = vec3(0.0, 0.0, 0.0);
    vec3 currentSpecularContrib = vec3(0.0, 0.0, 0.0);
    microfacetModel(specularReflectance, c_diff, metallic, roughness, v_dot_h,
                    n_dot_l, n_dot_v, n_dot_h, lightRadiance,
                    currentDiffuseContrib, currentSpecularContrib);

#if defined(SHADOWS_VSM)
    float shadow =
        (iLight < maxShadowNum
             ? computeShadowVSM(iLight, position, LightDirections[iLight].xyz)
             : 1.0f);
#else
    float shadow = 1.0f;
#endif

    diffuseContrib += shadow * currentDiffuseContrib;
    specularContrib += shadow * currentSpecularContrib;
  }  // for lights

  diffuseContrib *= ComponentScales[DirectDiffuse];
  specularContrib *= ComponentScales[DirectSpecular];

  // TODO: use ALPHA_MASK to discard fragments
  fragmentColor += vec4(diffuseContrib + specularContrib, baseColor.a);
#endif  // if LIGHT_COUNT > 0

#if defined(IMAGE_BASED_LIGHTING)
  vec3 iblDiffuseContrib = computeIBLDiffuse(c_diff, n);
  fragmentColor.rgb += iblDiffuseContrib;

  vec3 reflection = normalize(reflect(-view, n));
  vec3 iblSpecularContrib =
      computeIBLSpecular(roughness, n_dot_v, specularReflectance, reflection);
  fragmentColor.rgb += iblSpecularContrib;
#endif  // IMAGE_BASED_LIGHTING

#if defined(OBJECT_ID)
  fragmentObjectId = ObjectId;
#endif
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
