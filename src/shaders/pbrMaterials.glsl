// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// -------------- uniforms for material and textures ------------------

uniform MaterialData uMaterial;

#if defined(BASECOLOR_TEXTURE)
uniform sampler2D uBaseColorTexture;
#endif
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
uniform sampler2D uMetallicRoughnessTexture;
#endif
#if defined(NORMAL_TEXTURE)
uniform sampler2D uNormalTexture;
#endif
// TODO: separate occlusion texture
// (if it is not packed with metallicRoughness texture)
#if defined(EMISSIVE_TEXTURE)
uniform sampler2D uEmissiveTexture;
#endif

#if defined(CLEAR_COAT)
uniform ClearCoatData uClearCoat;

#if defined(CLEAR_COAT_TEXTURE)
uniform sampler2D uClearCoatTexture;
#endif  // CLEAR_COAT_TEXTURE

#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
uniform sampler2D uClearCoatRoughnessTexture;
#endif  // CLEAR_COAT_TEXTURE

#if defined(CLEAR_COAT_NORMAL_TEXTURE)
uniform sampler2D uClearCoatNormalTexture;
#endif  // CLEAR_COAT_NORMAL_TEXTURE

#endif  // CLEAR_COAT

#if defined(SPECULAR_LAYER)
uniform SpecularLayerData uSpecularLayer;

#if defined(SPECULAR_LAYER_TEXTURE)
uniform sampler2D uSpecularLayerTexture;
#endif  // SPECULAR_LAYER_TEXTURE

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)
uniform sampler2D uSpecularLayerColorTexture;
#endif  // SPECULAR_LAYER_COLOR_TEXTURE

#endif  // SPECULAR_LAYER

#if defined(ANISOTROPY_LAYER)
uniform AnisotropyLayerData uAnisotropyLayer;

#if defined(ANISOTROPY_LAYER_TEXTURE)
uniform sampler2D uAnisotropyLayerTexture;
#endif  // ANISOTROPY_LAYER_TEXTURE

#endif  // ANISOTROPY_LAYER

// ------------------ Utility Functions for materials/config ------------------
#if defined(ANISOTROPY_LAYER)

// Configure an AnisotropyInfo object for direct or indirect lighting
// dir : the tangent-space direction of the anisotropy
// anisotropy : strength of anisotropy
// anisotropicT : world space tangent
// anisotropicB : world space bitangent
// view : view vector
// alphaRoughness : linearized material roughness (perceived roughness squared)
// info : AnisotropyInfo structure to be populated
void configureAnisotropyInfo(vec2 dir,
                             float anisotropy,
                             vec3 anisotropicT,
                             vec3 anisotropicB,
                             vec3 view,
                             float alphaRoughness,
                             out AnisotropyInfo info) {
  info.dir = dir;
  info.anisotropy = anisotropy;
  // Derive roughness in each direction
  // From standard
  // https://github.com/KhronosGroup/glTF/tree/main/extensions/2.0/Khronos/KHR_materials_anisotropy#implementation
  // info.aT = mix(alphaRoughness, 1.0, anisotropy*anisotropy);
  // info.aB = alphaRoughness;
  // From
  // https://google.github.io/filament/Filament.md.html#materialsystem/anisotropicmodel/anisotropicspecularbrdf
  // If anisotropy == 0 then just alpharoughness in each direction
  info.aT = max(alphaRoughness * (1.0 + anisotropy), epsilon);
  info.aB = max(alphaRoughness * (1.0 - anisotropy), epsilon);

  info.anisotropicT = anisotropicT;
  info.anisotropicB = anisotropicB;
  info.t_dot_v = dot(anisotropicT, view);
  info.b_dot_v = dot(anisotropicB, view);

}  // configureAnisotropyInfo

#if (LIGHT_COUNT > 0)

// Configure a light-dependent AnistropyDirectLight object
// aInfo : AnisotropyInfo object for current material
// l : LightInfo structure for current light
// (out) info : AnisotropyInfo structure to be populated
void configureAnisotropyLightInfo(LightInfo l,
                                  AnisotropyInfo aInfo,
                                  out AnistropyDirectLight info) {
  info.t_dot_l = dot(aInfo.anisotropicT, l.light);
  info.b_dot_l = dot(aInfo.anisotropicB, l.light);
  info.t_dot_h = dot(aInfo.anisotropicT, l.halfVector);
  info.b_dot_h = dot(aInfo.anisotropicB, l.halfVector);

}  // configureAnisotropyLightInfo

#endif  // (LIGHT_COUNT > 0)

#endif  // ANISOTROPY_LAYER
