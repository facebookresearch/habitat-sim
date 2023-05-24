// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// -------------- structs for lights and materials ------------------
struct MaterialData {
  vec4 baseColor;   // diffuse color, if BaseColorTexture exists,
                    // multiply it with the BaseColorTexture
  float roughness;  // roughness of a surface, if roughness texture exists,
                    // multiply it with the MetallicRoughnessTexture
  float metallic;   // metalness of a surface, if metallic texture exists,
                    // multiply it the MetallicRoughnessTexture
  float ior;  // index of refraction.  Default 1.5 gives DielectricReflectance
              // value 0.04
  vec3 emissiveColor;  // emissiveColor, if emissive texture exists,
                       // multiply it the EmissiveTexture
};
/////////////////
// Clearcoat layer support
// Struct holding uniform data
#if defined(CLEAR_COAT)
struct ClearCoatData {
  float factor;     // clearcoat factor/intensity. If ClearCoatTexture exists,
                    // multiply it by this factor. If this is 0 the layer is
                    // ignored, as per standard.
  float roughness;  // clearcoat perceived roughness. If
                    // ClearCoatRoughnessTexture exists, multiply it
                    // by this roughness value
#if defined(CLEAR_COAT_NORMAL_TEXTURE)
  float normalTextureScale;  // xy scale value for clearcoat normal texture.
                             // Multiply the x,y channels of
                             // ClearCoatNormalTexture if exists.
#endif                       // if defined(CLEAR_COAT_NORMAL_TEXTURE)
};
#endif  // CLEAR_COAT

/////////////////
// Specular layer support
#if defined(SPECULAR_LAYER)
// Struct holding uniform data
struct SpecularLayerData {
  float factor;  // The strength of the specular reflection.
                 // If SpecularLayerTexture exists, multiply it with this value.
  vec3 colorFactor;  // The F0 (Fresnel reflectance at normal incidence) color
                     // of the specular reflection (linear RGB). If
                     // SpecularLayerColorTexture exists, multiply it with this
                     // value.
};
#endif  // SPECULAR_LAYER

/////////////////
// Anisotropy layer support
#if defined(ANISOTROPY_LAYER)
// Struct holding uniform data
struct AnisotropyLayerData {
  float factor;    // The anisotropy strength. When anisotropyTexture is
                   // present, this value is multiplied by the blue channel.
  vec2 direction;  // [ cos(anisotropyRotation), sin(anisotropyRotation) ]
                   // Built from the rotation of the anisotropy in tangent,
                   // bitangent space, measured in radians counter-clockwise
                   // from the tangent. When anisotropyTexture is present,
                   // anisotropyRotation provides additional rotation to
                   // the vectors in the texture.
};

//////////////////////////
// Structure holding base anisotropy info to faciliate passing as function
// argument
struct AnisotropyInfo {
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

#if (LIGHT_COUNT > 0)

//////////////////////////
// Structure holding per-light anisotropy-related lighting info to faciliate
// passing as function args
struct AnistropyDirectLight {
  // cos angle between tangent and light
  float t_dot_l;
  // cos angle between tangent and halfvector
  float t_dot_h;
  // cos angle between bitangent and light
  float b_dot_l;
  // cos angle between bitangent and halfvector
  float b_dot_h;
};

#endif  // if (LIGHT_COUNT > 0)

#endif  // ANISOTROPY_LAYER

#if (LIGHT_COUNT > 0)
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

#endif  // (LIGHT_COUNT > 0)
