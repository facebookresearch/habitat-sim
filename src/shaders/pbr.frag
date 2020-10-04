// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This is an implementation of
// Karis, Brian. “Real Shading in Unreal Engine 4 by.” (2013).

// -------------- input ---------------------
// position, normal, tangent in camera space
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

// if .w == 0, it means it is a dirctional light, .xyz is the direction;
// if .w == 1, it means it is a point light, .xyz is the light position;
// it is NOT put in the Light Structure, simply because we may modify the code
// so it is computed in the vertex shader.
uniform vec4 LightDirections[LIGHT_COUNT];
#endif

// -------------- material, textures ------------------
struct MaterialData {
  vec4 baseColor;   // diffuse color, if baseColorTexture exists,
                    // use the baseColorTexture
  float roughness;  // roughness of a surface, if roughness texture exists,
                    // use the roughnessTexture
  float metallic;   // metalness of a surface, if metallic texture exists,
                    // use the metallicTexture
};
uniform MaterialData Material;

#if defined(BASECOLOR_TEXTURE)
uniform sampler2D baseColorTexture;
#endif
#if defined(ROUGHNESS_TEXTURE)
uniform sampler2D roughnessTexture;
#endif
#if defined(METALLIC_TEXTURE)
uniform sampler2D metallicTexture;
#endif
#if defined(NORMAL_TEXTURE)
uniform sampler2D normalTexture;
#endif

#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE) || \
    defined(OCCLUSION_ROUGHNESS_METALLIC_TEXTURE)
uniform sampler2D combinedTexture;
#endif

// -------------- uniforms ----------------
#if defined(OBJECT_ID)
uniform highp uint objectId;
#endif

#if defined(NORMAL_TEXTURE) && defined(NORMAL_TEXTURE_SCALE)
uniform mediump float normalTextureScale
#ifndef GL_ES
    = 1.0
#endif
    ;
#endif

// -------------- shader ------------------
#if defined(NORMAL_TEXTURE)
vec3 getNormalFromNormalMap() {
  vec3 tangentNormal =
#if defined(NORMAL_TEXTURE_SCALE)
      normalize((texture(normalTexture, texCoord).xyz * 2.0 - 1.0) *
                vec3(normalTextureScale, normalTextureScale, 1.0));
#else
      texture(normalTexture, texCoord).xyz * 2.0 - 1.0;
#endif

#if defined(PRECOMPUTED_TANGENT)
  mat3 TBN = mat3(tangent, biTangent, normal);
#else
  // Normal Mapping Without Precomputed Tangents
  // See here: http://www.thetenthplanet.de/archives/1180
  vec3 q1 = dFdx(position);
  vec3 q2 = dFdy(position);
  vec2 st1 = dFdx(texCoord);
  vec2 st2 = dFdy(texCoord);

  vec3 N = normalize(normal);
  vec3 T = normalize(q1 * st2.t - q2 * st1.t);
  vec3 B = -normalize(cross(N, T));
  mat3 TBN = mat3(T, B, N);
#endif

  return normalize(TBN * tangentNormal);
}
#endif

const float PI = 3.14159265359;
const float Epsilon = 0.0001;

// Specular D, normal distribution function (NDF),
// also known as ggxDistribution
// normal: normal direction
// light: light source direction
// viwer: camera direction, aka light outgoing direction
// half: half vector of light and view
float normalDistribution(vec3 normal, vec3 half, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
  float n_dot_h = max(dot(normal, half), 0.0);

  float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
  d = PI * d * d;

  return a2 / d;
}

// helper function to compute the Specular G
float geometrySchlickGGX(float dotProd, float roughness) {
  float r = (roughness + 1.0);
  float k = (r * r) / 8.0;
  float d = dotProd * (1.0 - k) + k;

  return dotProd / d;
}

// Specular G, specular geometric attenuation
// normal: normal direction
// light: light source direction
// view: camera direction, aka light outgoing direction
float specularGeometricAttenuation(vec3 normal,
                                   vec3 light,
                                   vec3 view,
                                   float roughness) {
  float n_dot_l = max(dot(normal, light), 0.0);
  float n_dot_v = max(dot(normal, view), 0.0);
  float ggx1 = geometrySchlickGGX(n_dot_l, roughness);
  float ggx2 = geometrySchlickGGX(n_dot_v, roughness);

  return ggx1 * ggx2;
}

// Specular F, aka Fresnel, use Schlick's approximation
// F0: specular reflectance at normal incidence
// view: camera direction, aka light outgoing direction
// half: half vector of light and view
vec3 fresnelSchlick(vec3 F0, vec3 view, vec3 half) {
  float v_dot_h = max(dot(view, half), 0.0);
  return F0 + (1.0 - F0) * pow(1.0 - v_dot_h, 5.0);
}

// baseColor: diffuse color from baseColorTexture
//              or from baseColor in the material;
// metallic: metalness of the surface, from metallicTexture
//           or from metallic in the material;
// roughness: roughness of the surface, from roughnessTexture
//            or from roughness in the material;
// normal: normal direction
// light: light source direction
// view: camera direction, aka light outgoing direction
// lightRadiance: the radiance of the light,
//                which equals to intensity * attenuation
vec3 microfacetModel(vec3 baseColor,
                     float metallic,
                     float roughness,
                     vec3 normal,
                     vec3 light,
                     vec3 view,
                     vec3 lightRadiance) {
  vec3 half = normalize(light + view);
  // compute F0, specular reflectance at normal incidence
  // for nonmetal, using constant 0.04
  vec3 F0 = mix(vec3(0.04), baseColor, metallic);
  vec3 Fresnel = fresnelSchlick(F0, view, half);

  // Diffuse BRDF
  // NOTE: energy conservation requires
  // diffuse + specular <= 1.0, where specular = Fresnel
  // Also: result does not need to be scaled by 1/PI
  // See details:
  // https://seblagarde.wordpress.com/2012/01/08/pi-or-not-to-pi-in-game-lighting-equation/
  vec3 diffuse = mix(vec3(1.0) - Fresnel, vec3(0.0), metallic) * baseColor;

  // Specular BDDF
  float temp =
      max(4.0 * max(dot(normal, light), 0.0) * max(dot(normal, view), 0.0),
          Epsilon);
  vec3 specular = Fresnel *
                  specularGeometricAttenuation(normal, light, view, roughness) *
                  normalDistribution(normal, half, roughness) / temp;

  return (diffuse + specular) * lightRadiance * max(dot(normal, light), 0.0);
}

void main() {
#if (LIGHT_COUNT > 0)
  vec4 baseColor = Material.baseColor;
#if defined(BASECOLOR_TEXTURE)
  baseColor *= texture(baseColorTexture, texCoord);
#endif

  float roughness = Material.roughness;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE) || \
    defined(OCCLUSION_ROUGHNESS_METALLIC_TEXTURE)
  roughness *= texture(combinedTexture, texCoord).g;
#elif defined(ROUGHNESS_TEXTURE)
  roughness *= texture(roughnessTexture, texCoord).r;
#endif

  float metallic = Material.metallic;
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE) || \
    defined(OCCLUSION_ROUGHNESS_METALLIC_TEXTURE)
  metallic *= texture(combinedTexture, texCoord).b;
#elif defined(METALLIC_TEXTURE)
  metallic *= texture(metallicTexture, texCoord).r;
#endif

#if defined(NORMAL_TEXTURE)
  // normal is now in the camera space
  vec3 n = getNormalFromNormalMap();
#else
  vec3 n = normal;
#endif

  // view dirction: a vector from current position to camera
  // in camera space, camera is at the origin
  vec3 view = normalize(-position);

  vec3 finalColor = vec3(0.0);
  highp float finalAlpha = 0.0;
  // compute contribution of each light using the microfacet model
  // the following part of the code is inspired by the Phong.frag in Magnum
  // library (https://magnum.graphics/)
  for (int iLight = 0; iLight < LIGHT_COUNT; ++iLight) {
    // Attenuation. Directional lights have the .w component set to 0, use
    // that to make the distance zero -- which will then ensure the
    // attenuation is always 1.0
    highp float dist =
        length(LightDirections[iLight].xyz) * LightDirections[iLight].w;
    // If range is 0 for whatever reason, clamp it to a small value to
    // avoid a NaN when dist is 0 as well (which is the case for
    // directional lights).
    highp float attenuation =
        clamp(1.0 - pow(dist / max(LightRanges[iLight], 0.0001), 4.0), 0.0,
              1.0) /
        (1.0 + dist);
    attenuation = attenuation * attenuation;

    // radiance
    vec3 lightRadiance = LightColors[iLight] * attenuation;

    // light source direction: a vector from current position to the light
    vec3 light = normalize(LightDirections[iLight].xyz -
                           position * LightDirections[iLight].w);

    finalColor += microfacetModel(baseColor.rgb, metallic, roughness, n, light,
                                  view, lightRadiance);
    finalAlpha += baseColor.w;
  }  // for lights

  // TODO: use ALPHA_MASK to discard fragments
  fragmentColor = vec4(finalColor, finalAlpha / float(LIGHT_COUNT));
#else
fragmentColor = vec4(0.0, 0.0, 0.0, 1.0);
#endif  // if LIGHT_COUNT > 0

#if defined(OBJECT_ID)
  fragmentObjectId = objectId;
#endif
}
