// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

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

// -------------- uniforms ----------------
#if defined(OBJECT_ID)
uniform highp uint uObjectId;
#endif

// camera position in world space
uniform highp vec3 uCameraWorldPos;

#if defined(PBR_DEBUG_DISPLAY)
uniform int uPbrDebugDisplay;
#endif

/////////////////////
// DEBUG - disable calcs only
// define these macros to disable layer calculations for debugging or perf tests
#if defined(ANISOTROPY_LAYER)
//#define SKIP_CALC_ANISOTROPY_LAYER
#endif

#if defined(CLEAR_COAT)
//#define SKIP_CALC_CLEAR_COAT
#endif

#if defined(SPECULAR_LAYER)
//#define SKIP_CALC_SPECULAR_LAYER
#endif

#if defined(NORMAL_TEXTURE)
//#define SKIP_CALC_NORMAL_TEXTURE
#endif

// -------------- uniforms for material and textures ------------------
// MaterialData defined in pbrMaterials.glsl
uniform MaterialData uMaterial;

#if defined(BASECOLOR_TEXTURE)
uniform sampler2D uBaseColorTexture;
#endif
#if defined(NONE_ROUGHNESS_METALLIC_TEXTURE)
uniform sampler2D uMetallicRoughnessTexture;
#endif
#if defined(NORMAL_TEXTURE)
uniform float uNormalTextureScale;
uniform sampler2D uNormalTexture;
#endif
// TODO: separate occlusion texture
// (if it is not packed with metallicRoughness texture)
#if defined(EMISSIVE_TEXTURE)
uniform sampler2D uEmissiveTexture;
#endif

#if defined(CLEAR_COAT)
// ClearCoatData defined in pbrMaterials.glsl
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
// SpecularLayerData defined in pbrMaterials.glsl
uniform SpecularLayerData uSpecularLayer;

#if defined(SPECULAR_LAYER_TEXTURE)
uniform sampler2D uSpecularLayerTexture;
#endif  // SPECULAR_LAYER_TEXTURE

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)
uniform sampler2D uSpecularLayerColorTexture;
#endif  // SPECULAR_LAYER_COLOR_TEXTURE

#endif  // SPECULAR_LAYER

#if defined(ANISOTROPY_LAYER)
// AnisotropyLayerData defined in pbrMaterials.glsl
uniform AnisotropyLayerData uAnisotropyLayer;

#if defined(ANISOTROPY_LAYER_TEXTURE)
uniform sampler2D uAnisotropyLayerTexture;
#endif  // ANISOTROPY_LAYER_TEXTURE

#endif  // ANISOTROPY_LAYER

// -------------- lights and/or IBL -------------------

// uExposure is used for any tonemapping, direct or IBL
#if (defined(DIRECT_LIGHTING) && defined(DIRECT_TONE_MAP)) || \
    (defined(IMAGE_BASED_LIGHTING) && defined(IBL_TONE_MAP))
uniform float uExposure;
#endif  // #if (defined(DIRECT_LIGHTING) && defined(DIRECT_TONE_MAP)) ||
        // (defined(IMAGE_BASED_LIGHTING) && defined(IBL_TONE_MAP))

// uGamma is used for any remapping
#if defined(MAP_MAT_TXTRS_TO_LINEAR) || defined(MAP_IBL_TXTRS_TO_LINEAR)
uniform vec3 uGamma;
#endif

#if defined(MAP_OUTPUT_TO_SRGB)
uniform vec3 uInvGamma;
#endif  // defined(MAP_OUTPUT_TO_SRGB)

#if defined(DIRECT_LIGHTING)

// NOTE: In this shader, the light intensity is already combined with the color
// in each uLightColors vector;
uniform vec3 uLightColors[LIGHT_COUNT];
uniform float uLightRanges[LIGHT_COUNT];

// lights in world space!
// if .w == 0, it means it is a directional light, .xyz is the direction;
// if .w == 1, it means it is a point light, .xyz is the light position;
// it is NOT put in the Light Structure, simply because we may modify the code
// so it is computed in the vertex shader.
uniform vec4 uLightDirections[LIGHT_COUNT];

// Config driven overall direct lighting intensity
uniform float uDirectLightIntensity;

#endif  // DIRECT_LIGHTING

#if defined(IMAGE_BASED_LIGHTING)

uniform samplerCube uIrradianceMap;
uniform sampler2D uBrdfLUT;
uniform samplerCube uPrefilteredMap;
uniform uint uPrefilteredMapMipLevels;

// scales for components in the PBR equation - only necessary if -both- lighting
// types are present.
// [0] = direct diffuse [1] = direct specular [2] = ibl
// diffuse [3] = ibl specular

#if defined(DIRECT_LIGHTING)
const int DirectDiffuse = 0;
const int DirectSpecular = 1;
const int IblDiffuse = 2;
const int IblSpecular = 3;
uniform vec4 uComponentScales;
#endif  // DIRECT_LIGHTING
#endif  // IMAGE_BASED_LIGHTING
