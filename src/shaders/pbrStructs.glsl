// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// -------------- structs for lights and material layers ------------------
struct MaterialData {
  vec4 baseColor;   // diffuse color, if BaseColorTexture exists,
                    // multiply it with the BaseColorTexture
  float roughness;  // roughness of a surface, if roughness texture exists,
                    // multiply it with the MetallicRoughnessTexture
  float metallic;   // metalness of a surface, if metallic texture exists,
                    // multiply it the MetallicRoughnessTexture
  float ior;  // index of refraction.  Default 1.5 gives dielectricReflectance
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
// Struct holding uniform data for anisotropy layer
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

#if (LIGHT_COUNT > 0)

//////////////////////////
// Structure holding per-light anisotropy-related lighting info to faciliate
// passing as function args
struct AnisotropyDirectLight {
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

// Structure holding PBR global material values used by direct/indirect lighting
// calcs and derived from various uniforms
struct PBRData {
  // Normal at position
  vec3 n;
#if (defined(NORMAL_TEXTURE) || defined(CLEAR_COAT_NORMAL_TEXTURE) || \
     defined(ANISOTROPY_LAYER))
  // Tangent-bitangent-normal frame, either provided by precomputed tangent or
  // derived using partial derivatives
  mat3 TBN;
#endif
  // View vector at position
  vec3 view;
  // Cos angle between view and normal
  float n_dot_v;
  // base/diffuse color provided by material
  vec4 baseColor;
  // emissive color provided by material
  vec3 emissiveColor;
  // Index of refraction of material surface
  float ior;
  // Index of refraction of adjacent material (air unless clearcoat is
  // present)
  float ior_adj;
  // metallic value of material. Usually 0 or 1.
  float metallic;
  // Perceived roughness of material
  float perceivedRoughness;
  // 'actual' roughness, square of perceived roughness to linearize roughness
  // results
  float alphaRoughness;
  // diffuse color used by calculations
  vec3 diffuseColor;
  // Achromatic dielectric material f0 : fresnel reflectance at normal
  // incidence based on given IOR. Specular layer data modfies this
  vec3 dielectric_f0;
  // dielextric/metallic f0 : fresnel reflectance color at normal
  // incidence based on given IOR. Specular layer data modfies this
  vec3 specularColor_f0;
  // glancing incidence dielectric/metallic reflectance. Specular layer data
  // modifies this
  vec3 specularColor_f90;

// Clearcoat layer quantities used in shader
#if defined(CLEAR_COAT)
  // Strength of clearcoat effect
  float clearCoatStrength;
  // Perceived value of clearcoat roughness
  float clearCoatPerceivedRoughness;
  // linearized clearcoat roughness (clearCoatPerceivedRoughness *
  // clearCoatPerceivedRoughness)
  float clearCoatRoughness;
  // Clearcoat normal for lighting calculations
  vec3 clearCoatNormal;
  // Clearcoat cos angle between clearcoat normal and view
  float cc_n_dot_v;
  // Assuming dielectric reflectance of 0.04, which corresponds to
  // polyurethane coating with IOR == 1.5
  vec3 clearCoatCoating_f0;
  // Glancing angle reflectance
  vec3 clearCoatCoating_f90;
  // Global normal-based clearcoat fresnel contribution for IBL and aggregate
  // direct lit clearcoat contribution
  // https://google.github.io/filament/Filament.md.html#toc5.3.5
  vec3 OneM_ccFresnelGlbl;
#endif  // if defined(CLEAR_COAT)

// Anisotropic layer quantities used in shader
#if defined(ANISOTROPY_LAYER)
  // Strength of anisotropy effect
  float anisotropy;
  // Anistropy tangent
  vec3 anisotropicT;
  // Anisotropy bitangent
  vec3 anisotropicB;
#if defined(IMAGE_BASED_LIGHTING)
  // Precalc bent normal for reflection
  vec3 bentNormal;
#if defined(CLEAR_COAT)
  // Precalc clearcoat bent normal for reflection of clearcoat with anisotropy
  vec3 cc_BentNormal;
#endif  // CLEAR_COAT
#endif  // IMAGE_BASED_LIGHTING
  // anisotropic roughness value along tangent direction
  float aT;
  // anisotropic roughness value along bitangent direction
  float aB;
  // anisotropic roughness sq for microfacet dist model
  float aSqr;
  // cos angle between tangent and view
  float t_dot_v;
  // cos angle between bitangent and view
  float b_dot_v;

#endif  // if defined(ANISOTROPY_LAYER)

// Specular layer quantities used in shader
#if defined(SPECULAR_LAYER)
  // Strength of specular layer contribution
  float specularWeight;
  // F0 Color of specular layer contribution (linear RGB)
  // NOTE : specular layer texture is specified in standard as sRGB
  vec3 specularLayerColor;

#endif  // SPECULAR_LAYER

};  // PBRData struct

// Structure holding the result of direct and indirect lighting calculations
struct PBRResultData {
  // Contributions for diffuse, specular, clearcoat
  // for direct and image-based lighting
  // Aggregate direct lighting contribution for diffuse color
  vec3 diffuseContrib;
  // Aggregate direct lighting contribution for specular color
  vec3 specularContrib;
#if defined(CLEAR_COAT)
  // Aggregate direct lighting contribution for clearCoat
  vec3 clearCoatContrib;
#endif  // Clearcoat

  // Aggregate image-basedlighting contribution for diffuse color
  vec3 iblDiffuseContrib;
  // Aggregate image-basedlighting contribution for specular color
  vec3 iblSpecularContrib;
#if defined(CLEAR_COAT)
  // Aggregate image-basedlighting contribution for clearCoat
  vec3 iblClearCoatContrib;
#endif  // Clearcoat
};      // PBRResultData
