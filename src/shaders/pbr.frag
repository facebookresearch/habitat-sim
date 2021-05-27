// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This is an implementation of
// Karis, Brian. “Real Shading in Unreal Engine 4” (2013).

precision highp float;

#undef DOUBLE_SIDED // XXX
// -------------- input ---------------------
// position, normal, tangent, biTangent in world space, NOT camera space
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
  vec4 baseColor;     // diffuse color, if BaseColorTexture exists,
                      // multiply it with the BaseColorTexture
  float roughness;    // roughness of a surface, if roughness texture exists,
                      // multiply it with the MetallicRoughnessTexture
  float metallic;     // metalness of a surface, if metallic texture exists,
                      // multiply it the MetallicRoughnessTexture
  vec3 emissiveColor; // emissiveColor, if emissive texture exists,
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

struct PbrDebugToggle {
  float directDiffuse;
  float directSpecular;
  float iblDiffuse;
  float iblSpecular;
};
uniform PbrDebugToggle PbrDebug;

uniform int PbrDebugDisplay;

// -------------- shader ------------------
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
vec3 getNormalFromNormalMap() {
#error hahaha Normal mapping requires precomputed tangents. // XXX
  vec3 tangentNormal =
#if defined(NORMAL_TEXTURE_SCALE)
      normalize((texture(NormalTexture, texCoord).xyz * 2.0 - 1.0) *
                vec3(NormalTextureScale, NormalTextureScale, 1.0));
#else
      texture(NormalTexture, texCoord).xyz * 2.0 - 1.0;
#endif

//XXX
	// Perturb normal, see http://www.thetenthplanet.de/archives/1180
	// vec3 tangentNormal = texture(normalMap, material.normalTextureSet == 0 ? inUV0 : inUV1).xyz * 2.0 - 1.0;
  tangentNormal = texture(NormalTexture, texCoord).xyz * 2.0 - 1.0;

	vec3 q1 = dFdx(position);
	vec3 q2 = dFdy(position);
	vec2 st1 = dFdx(texCoord);
	vec2 st2 = dFdy(texCoord);

	vec3 N = normalize(inNormal);
	vec3 T = normalize(q1 * st2.t - q2 * st1.t);
	vec3 B = -normalize(cross(N, T));
  N = vec3(0.0, 0.0, 1.0);
  T = vec3(1.0, 0.0, 0.0);
  B = vec3(0.0, 1.0, 0.0);
	mat3 TBN = mat3(T, B, N);

	return normalize(TBN * tangentNormal);

// XXXX

#if defined(PRECOMPUTED_TANGENT)
  mat3 TBN = mat3(tangent, biTangent, normal);
#else
// TODO:
// explore robust screen-space normal mapping withOUT precomputed tangents
#error Normal mapping requires precomputed tangents.
#endif

  // TBN transforms tangentNormal from tangent space to world space
  return normalize(TBN * tangentNormal);
}
#endif

const float PI = 3.14159265359;
const float INV_PI = 1.0 / PI;
const float Epsilon = 0.0001;
const float DielectricSpecular = 0.04;

// Specular D, normal distribution function (NDF),
// also known as ggxDistribution
// normal: normal direction
// light: light source direction
// viwer: camera direction, aka light outgoing direction
// halfVector: half vector of light and view
float normalDistribution(vec3 normal, vec3 halfVector, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;
#if defined(DOUBLE_SIDED)
  float n_dot_h = clamp(abs(dot(normal, halfVector)), 0.0, 1.0);
#else
  float n_dot_h = clamp(dot(normal, halfVector), 0.0, 1.0);
#endif

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
#if defined(DOUBLE_SIDED)
  float n_dot_l = clamp(abs(dot(normal, light)), 0.001, 1.0);
  float n_dot_v = clamp(abs(dot(normal, view)), 0.001, 1.0);
#else
  float n_dot_l = clamp(dot(normal, light), 0.001, 1.0);
  float n_dot_v = clamp(dot(normal, view), 0.001, 1.0);
#endif
  float ggx1 = geometrySchlickGGX(n_dot_l, roughness);
  float ggx2 = geometrySchlickGGX(n_dot_v, roughness);

  return ggx1 * ggx2;
}

// Specular F, aka Fresnel, use Schlick's approximation
// specularReflectance: specular reflectance at normal incidence
// view: camera direction, aka light outgoing direction
// halfVector: half vector of light and view
vec3 fresnelSchlick(vec3 specularReflectance,
                    vec3 view,
                    vec3 halfVector) {
  float v_dot_h = clamp(dot(view, halfVector), 0.0, 1.0);

  // https://github.com/SaschaWillems/Vulkan-glTF-PBR
  // For typical incident reflectance range (between 4% to 100%)
  // set the grazing reflectance to 100% for typical fresnel effect.
	// For very low reflectance range on highly diffuse objects (below 4%),
  // incrementally reduce grazing reflecance to 0%.
  float reflectance = max(max(specularReflectance.r, specularReflectance.g), specularReflectance.b);
  float reflectance90 = clamp(reflectance * 25.0, 0.0, 1.0);
  return specularReflectance +
         (vec3(reflectance90) - specularReflectance) *
         pow(1.0 - v_dot_h, 5.0);
}

// specularReflectance: specular reflectance at normal incidence
//     for nonmetal, using constant 0.04
// c_diff: diffuse color
// metallic: metalness of the surface
// roughness: roughness of the surface
// normal: normal direction
// light: light source direction
// view: camera direction, aka light outgoing direction
// lightRadiance: the radiance of the light,
//                which equals to intensity * attenuation
vec3 microfacetModel(vec3 specularReflectance,
                     vec3 c_diff,
                     float metallic,
                     float roughness,
                     vec3 normal,
                     vec3 light,
                     vec3 view,
                     vec3 lightRadiance) {
  vec3 halfVector = normalize(light + view);
  vec3 Fresnel = fresnelSchlick(specularReflectance, view, halfVector);

  // Diffuse BRDF
  // NOTE: energy conservation requires
  // diffuse + specular <= 1.0, where specular = Fresnel
  vec3 diffuse = (vec3(1.0) - Fresnel) * c_diff * INV_PI;

  // Specular BRDF
#if defined(DOUBLE_SIDED)
  float n_dot_l = clamp(abs(dot(normal, light)), 0.001, 1.0);
  float n_dot_v = clamp(abs(dot(normal, view)), 0.001, 1.0);
#else
  float n_dot_l = clamp(dot(normal, light), 0.001, 1.0);
  float n_dot_v = clamp(dot(normal, view), 0.001, 1.0);
#endif
  float temp = max(4.0 * n_dot_l * n_dot_v, Epsilon);
  vec3 specular = Fresnel *
                  specularGeometricAttenuation(normal, light, view, roughness) *
                  normalDistribution(normal, halfVector, roughness) / temp;
  return (diffuse * PbrDebug.directDiffuse +
          specular * PbrDebug.directSpecular) * lightRadiance * n_dot_l;
}

#if defined(IMAGE_BASED_LIGHTING)
// c_diff: diffuse color
vec3 computeIBL(vec c_diff) {
  // XXX
  // diffuse part = c_diff * irradiance
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
  vec3 n = getNormalFromNormalMap();
#else
  vec3 n = normal;
#endif

  // view is the normalized vector from the shading location to the camera
  // in *world space*
  vec3 view = normalize(CameraWorldPos - position);

  // compute specularReflectance, specular reflectance at normal incidence
  // for nonmetal, using constant 0.04
  vec3 specularReflectance = mix(vec3(DielectricSpecular), baseColor.rgb, metallic);

  // diffuse color (c_diff in gltf 2.0 spec:
  // https://github.com/KhronosGroup/glTF/blob/master/specification/2.0/README.md#metal-brdf-and-dielectric-brdf)
  // c_diff = lerp(baseColor.rgb * (1 - dielectricSpecular), black, metallic)
  vec3 c_diff = baseColor.rgb * (1.0 - DielectricSpecular) * (1.0 - metallic);

  vec3 finalColor = vec3(0.0);
  // compute contribution of each light using the microfacet model
  // the following part of the code is inspired by the Phong.frag in Magnum
  // library (https://magnum.graphics/)
  for (int iLight = 0; iLight < LIGHT_COUNT; ++iLight) {
    // Attenuation. Directional lights have the .w component set to 0, use
    // that to make the distance zero -- which will then ensure the
    // attenuation is always 1.0
    highp float dist =
        length(LightDirections[iLight].xyz - position) * LightDirections[iLight].w;
    // If range is 0 for whatever reason, clamp it to a small value to
    // avoid a NaN when dist is 0 as well (which is the case for
    // directional lights).
    highp float attenuation =
        clamp(1.0 - pow(dist / max(LightRanges[iLight], 0.0001), 4.0), 0.0,
              1.0);
    attenuation = attenuation * attenuation / (1.0 + dist * dist);

    // radiance
    vec3 lightRadiance = LightColors[iLight] * attenuation;

    // light source direction: a vector from the shading location to the light
    vec3 light = normalize(LightDirections[iLight].xyz -
                           position * LightDirections[iLight].w);
    /*
    vec3 microfacetModel(vec3 specularReflectance,
                         vec3 c_diff,
                         float metallic,
                         float roughness,
                         vec3 normal,
                         vec3 light,
                         vec3 view,
                         vec3 lightRadiance)
    */
    finalColor += microfacetModel(specularReflectance, c_diff, metallic, roughness, n, light,
                                  view, lightRadiance);
  }  // for lights

  // TODO: use ALPHA_MASK to discard fragments
  fragmentColor += vec4(finalColor, baseColor.a);
#endif  // if LIGHT_COUNT > 0

#if defined(IMAGE_BASED_LIGHTING)
  // TODO: compute lighting contribution from image based lighting (IBL)
#endif

#if defined(OBJECT_ID)
  fragmentObjectId = ObjectId;
#endif


// If Debug display
// PBR equation debug
	// "none", "Diff (l,n)", "F (l,h)", "G (l,v,h)", "D (h)", "Specular"
	if (PbrDebugDisplay > 0) {
		switch (PbrDebugDisplay) {
			case 1:
				fragmentColor.rgb = normal;
				break;
    /*
			case 2:
				outColor.rgb = F;
				break;
			case 3:
				outColor.rgb = vec3(G);
				break;
			case 4:
				outColor.rgb = vec3(D);
				break;
			case 5:
				outColor.rgb = specContrib;
				break;
    */
		}
	}
}
