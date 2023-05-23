// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// TODO : make exposure a uniform?
const highp float exposure = 4.5f;
// TODO : make gamma a uniform?
const float gamma = 2.2f;

const float invGamma = 1.0f / gamma;

#if (LIGHT_COUNT > 0)
// -------------- lights -------------------
// NOTE: In this shader, the light intensity is considered in the lightColor!!
uniform vec3 uLightColors[LIGHT_COUNT];
uniform float uLightRanges[LIGHT_COUNT];

// lights in world space!
// if .w == 0, it means it is a directional light, .xyz is the direction;
// if .w == 1, it means it is a point light, .xyz is the light position;
// it is NOT put in the Light Structure, simply because we may modify the code
// so it is computed in the vertex shader.
uniform vec4 uLightDirections[LIGHT_COUNT];

/////////////////////
// Structure holding light information to facilitate passing as
// function arguments
struct LightInfo {
  // normalize vector to light from illumination point
  vec3 light;
  // distance-attenuated light color vector
  vec3 lightIrradiance;
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
  vec3 projLightIrradiance;
};

// Configure a LightInfo object
// light : normalized point to light vector
// lightIrradiance : distance-attenuated light intensity/irradiance color
// n : normal
// view : normalized view vector (point to camera)
// n_dot_v : cos angle between n and view
// (out) l : LightInfo structure being popualted
void configureLightInfo(vec3 light,
                        vec3 lightIrradiance,
                        vec3 n,
                        vec3 view,
                        float n_dot_v,
                        out LightInfo l) {
  l.light = light;
  l.lightIrradiance = lightIrradiance;
  l.n_dot_v = n_dot_v;
  l.n_dot_l = clamp(dot(n, light), 0.0, 1.0);
  l.halfVector = normalize(light + view);
  l.v_dot_h = clamp(dot(view, l.halfVector), 0.0, 1.0);  // == l_dot_h
  l.n_dot_h = clamp(dot(n, l.halfVector), 0.0, 1.0);
  l.projLightIrradiance = lightIrradiance * l.n_dot_l;
}  // configureLightInfo
#endif  // (LIGHT_COUNT > 0)

/////////////////
// IBL Support
#if defined(IMAGE_BASED_LIGHTING)
uniform samplerCube uIrradianceMap;
uniform sampler2D uBrdfLUT;
uniform samplerCube uPrefilteredMap;
#endif

// scales for components in the PBR equation - only necessary if -both- lighting
// types are present.
// [0] = direct diffuse [1] = direct specular [2] = ibl
// diffuse [3] = ibl specular

#if defined(IMAGE_BASED_LIGHTING) && (LIGHT_COUNT > 0)
const int DirectDiffuse = 0;
const int DirectSpecular = 1;
const int IblDiffuse = 2;
const int IblSpecular = 3;
uniform highp vec4 uComponentScales;
#endif

#if defined(IMAGE_BASED_LIGHTING)
uniform uint uPrefilteredMapMipLevels;
#endif

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

// The following function tonemap is based on:
// https://github.com/SaschaWillems/Vulkan-glTF-PBR/blob/master/data/shaders/pbr_khr.frag
// Tone mapping is to take a wide dynamic range of values and compressing them
// into a smaller range that is appropriate for the output device.
vec4 tonemap(vec4 color) {
#ifdef TONE_MAP
  vec3 outcol = Uncharted2Tonemap(color.rgb * exposure);
  outcol = outcol * (1.0f / Uncharted2Tonemap(vec3(11.2f)));
  return vec4(pow(outcol, vec3(invGamma)), color.a);
#else
  return color;
#endif
}

#if defined(IMAGE_BASED_LIGHTING)
// diffuseColor: diffuse color
// n: normal on shading location in world space
vec3 computeIBLDiffuse(vec3 diffuseColor, vec3 n) {
  // diffuse part = diffuseColor * irradiance
  // return diffuseColor * texture(uIrradianceMap, n).rgb * Scales.iblDiffuse;
  return diffuseColor * tonemap(texture(uIrradianceMap, n)).rgb;
}

vec3 computeIBLSpecular(float roughness,
                        float n_dot_v,
                        vec3 specularReflectance,
                        vec3 reflectionDir) {
  vec3 brdf = texture(uBrdfLUT, vec2(n_dot_v, roughness)).rgb;
  float lod = roughness * float(uPrefilteredMapMipLevels - 1);
  vec3 prefilteredColor =
      tonemap(textureLod(uPrefilteredMap, reflectionDir, lod)).rgb;

  return prefilteredColor * (specularReflectance * brdf.x + brdf.y);
}
#endif  // IMAGE_BASED_LIGHTING
