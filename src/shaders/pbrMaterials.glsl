// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// -------------- uniforms for material and textures ------------------
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
uniform MaterialData uMaterial;
/////////////////
// Clearcoat layer support
#if defined(CLEAR_COAT)
struct ClearCoatData {
  float factor;  // clearcoat factor/intensity. If ClearCoatTexture exists,
                 // multiply it by this factor. If this is 0 the layer is
                 // ignored, as per standard.
  float
      roughness;  // clearcoat perceived roughness. If ClearCoatRoughnessTexture
                  // exists, multiply it by this roughness value
#if defined(CLEAR_COAT_NORMAL_TEXTURE)
  float normalTextureScale;  // xy scale value for clearcoat normal texture.
                             // Multiply the x,y channels of
                             // ClearCoatNormalTexture if exists.
#endif                       // if defined(CLEAR_COAT_NORMAL_TEXTURE)
};
uniform ClearCoatData uClearCoat;
#endif  // CLEAR_COAT

/////////////////
// Specular layer support
#if defined(SPECULAR_LAYER)
struct SpecularLayerData {
  float factor;  // The strength of the specular reflection.
                 // If SpecularLayerTexture exists, multiply it with this value.
  vec3 colorFactor;  // The F0 (Fresnel reflectance at normal incidence) color
                     // of the specular reflection (linear RGB). If
                     // SpecularLayerColorTexture exists, multiply it with this
                     // value.
};
uniform SpecularLayerData uSpecularLayer;
#endif  // SPECULAR_LAYER

#if defined(ANISOTROPY_LAYER)
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
uniform AnisotropyLayerData uAnisotropyLayer;
#endif  // ANISOTROPY_LAYER

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

#if defined(CLEAR_COAT_TEXTURE)
uniform sampler2D uClearCoatTexture;
#endif

#if defined(CLEAR_COAT_ROUGHNESS_TEXTURE)
uniform sampler2D uClearCoatRoughnessTexture;
#endif

#if defined(CLEAR_COAT_NORMAL_TEXTURE)
uniform sampler2D uClearCoatNormalTexture;
#endif

#if defined(SPECULAR_LAYER_TEXTURE)
uniform sampler2D uSpecularLayerTexture;
#endif

#if defined(SPECULAR_LAYER_COLOR_TEXTURE)
uniform sampler2D uSpecularLayerColorTexture;
#endif

#if defined(ANISOTROPY_LAYER_TEXTURE)
uniform sampler2D uAnisotropyLayerTexture;
#endif
