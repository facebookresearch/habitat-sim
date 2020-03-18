/*
    This file is part of Magnum.

    Copyright © 2010, 2011, 2012, 2013, 2014, 2015, 2016, 2017, 2018, 2019
              Vladimír Vondruš <mosra@centrum.cz>

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the "Software"),
    to deal in the Software without restriction, including without limitation
    the rights to use, copy, modify, merge, publish, distribute, sublicense,
    and/or sell copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
    DEALINGS IN THE SOFTWARE.
*/

/* Outputs */
#define COLOR_OUTPUT_ATTRIBUTE_LOCATION 0
#define OBJECT_ID_OUTPUT_ATTRIBUTE_LOCATION 1

#ifndef RUNTIME_CONST
#define const
#endif

#ifdef AMBIENT_TEXTURE
uniform lowp sampler2D ambientTexture;
#endif

uniform lowp vec4 ambientColor
#ifndef GL_ES
#ifndef AMBIENT_TEXTURE
    = vec4(0.0)
#else
    = vec4(1.0)
#endif
#endif
    ;

#if LIGHT_COUNT
#ifdef DIFFUSE_TEXTURE
uniform lowp sampler2D diffuseTexture;
#endif

uniform lowp vec4 diffuseColor
#ifndef GL_ES
    = vec4(1.0)
#endif
    ;

#ifdef SPECULAR_TEXTURE
uniform lowp sampler2D specularTexture;
#endif

#ifdef NORMAL_TEXTURE
uniform lowp sampler2D normalTexture;
#endif

uniform lowp vec4 specularColor
#ifndef GL_ES
    = vec4(1.0)
#endif
    ;

uniform mediump float shininess
#ifndef GL_ES
    = 80.0
#endif
    ;

uniform float shadowBias = 0.001;
uniform sampler2DArrayShadow shadowmapTexture;
uniform highp vec3 shadowLightDirection = vec3(1.0);
uniform bool shadeFacesFacingAwayFromLight = false;
#endif

#ifdef ALPHA_MASK
uniform lowp float alphaMask
#ifndef GL_ES
    = 0.5
#endif
    ;
#endif

#ifdef OBJECT_ID
/* mediump is just 2^10, which might not be enough, this is 2^16 */
uniform highp uint objectId; /* defaults to zero */
#endif

#if LIGHT_COUNT
/* Needs to be last because it uses locations 10 + LIGHT_COUNT to
   10 + 2*LIGHT_COUNT - 1. Location 10 is lightPositions. Also it can't be
   specified as 10 + LIGHT_COUNT because that requires ARB_enhanced_layouts. */
uniform lowp vec4 lightColors[LIGHT_COUNT]
#ifndef GL_ES
    = vec4[](LIGHT_COLOR_INITIALIZER)
#endif
    ;
#endif

#if LIGHT_COUNT
in mediump vec3 transformedNormal;
in mediump vec3 absoluteTransformedNormal;

#ifdef NORMAL_TEXTURE
in mediump vec3 transformedTangent;
#endif
in highp vec3 lightDirections[LIGHT_COUNT];
in highp vec3 shadowCoords[NUM_SHADOW_MAP_LEVELS];
in highp vec3 cameraDirection;
#endif

#if defined(AMBIENT_TEXTURE) || defined(DIFFUSE_TEXTURE) || \
    defined(SPECULAR_TEXTURE) || defined(NORMAL_TEXTURE)
in mediump vec2 interpolatedTextureCoords;
#endif

#ifdef VERTEX_COLOR
in lowp vec4 interpolatedVertexColor;
#endif

layout(location = COLOR_OUTPUT_ATTRIBUTE_LOCATION) out lowp vec4 fragmentColor;
#ifdef OBJECT_ID
layout(location = OBJECT_ID_OUTPUT_ATTRIBUTE_LOCATION)
    /* mediump is just 2^10, which might not be enough, this is 2^16 */
    out highp uint fragmentObjectId;
#endif

void main() {
  lowp const vec4 finalAmbientColor =
#ifdef AMBIENT_TEXTURE
      texture(ambientTexture, interpolatedTextureCoords) *
#endif
      ambientColor;
#if LIGHT_COUNT
  lowp const vec4 finalDiffuseColor =
#ifdef DIFFUSE_TEXTURE
      texture(diffuseTexture, interpolatedTextureCoords) *
#endif
#ifdef VERTEX_COLOR
      interpolatedVertexColor *
#endif
      diffuseColor;
  lowp const vec4 finalSpecularColor =
#ifdef SPECULAR_TEXTURE
      texture(specularTexture, interpolatedTextureCoords) *
#endif
      specularColor;
#endif

  /* Ambient color */
  fragmentColor = finalAmbientColor;

#if LIGHT_COUNT
  /* Normal */
  mediump vec3 normalizedTransformedNormal = normalize(transformedNormal);

  /* shadows */
  mediump vec3 normalizedAbsoluteTransformedNormal =
      normalize(absoluteTransformedNormal);

  /* You might want to source this from a texture or a vertex color */
  vec3 albedo = vec3(1.0, 1.0, 1.0);

#ifdef NORMAL_TEXTURE
  mediump vec3 normalizedTransformedTangent = normalize(transformedTangent);
  mediump mat3 tbn = mat3(normalizedTransformedTangent,
                          normalize(cross(normalizedTransformedNormal,
                                          normalizedTransformedTangent)),
                          normalizedTransformedNormal);
  normalizedTransformedNormal =
      tbn *
      (texture(normalTexture, interpolatedTextureCoords).rgb * 2.0 - vec3(1.0));
#endif

  /* Add diffuse color for each light */
  for (int i = 0; i < LIGHT_COUNT; ++i) {
    highp vec3 normalizedLightDirection = normalize(lightDirections[i]);
    lowp float intensity =
        max(0.0, dot(normalizedTransformedNormal, normalizedLightDirection));
    lowp vec4 colorFromLight =
        vec4(finalDiffuseColor.rgb * lightColors[i].rgb * intensity,
             lightColors[i].a * finalDiffuseColor.a / float(LIGHT_COUNT));

    /* Add specular color, if needed */
    if (intensity > 0.001) {
      highp vec3 reflection =
          reflect(-normalizedLightDirection, normalizedTransformedNormal);
      mediump float specularity = clamp(
          pow(max(0.0, dot(normalize(cameraDirection), reflection)), shininess),
          0.0, 1.0);
      colorFromLight +=
          vec4(finalSpecularColor.rgb * specularity, finalSpecularColor.a);
    }

    /* Shadow calculation */
    /* TODO: support multiple lights, use actual light direction here */
    /* Is the normal of this face pointing towards the light?
       if pointing away from the light anyway, we know it's in the shade, don't
        bother shadow map lookup */
    float visibility = 1.0;
    if (shadeFacesFacingAwayFromLight &&
        dot(normalize(absoluteTransformedNormal),
            normalize(shadowLightDirection)) <= 0) {
      visibility = 0.0f;
    } else {
      int shadowLevel = 0;
      bool inRange = false;

      /* Starting with highest resolution shadow map, find one we're in range
         of */
      for (; shadowLevel < NUM_SHADOW_MAP_LEVELS; ++shadowLevel) {
        vec3 shadowCoord = shadowCoords[shadowLevel];
        inRange = shadowCoord.x >= 0 && shadowCoord.y >= 0 &&
                  shadowCoord.x < 1 && shadowCoord.y < 1 &&
                  shadowCoord.z >= 0 && shadowCoord.z < 1;
        if (inRange) {
          visibility = texture(
              shadowmapTexture,
              vec4(shadowCoord.xy, shadowLevel, shadowCoord.z - shadowBias));
          break;
        }
      }
#ifdef DEBUG_SHADOWMAP_LEVELS
      switch (shadowLevel) {
        case 0:
          albedo *= vec3(1, 0, 0);
          break;
        case 1:
          albedo *= vec3(1, 1, 0);
          break;
        case 2:
          albedo *= vec3(0, 1, 0);
          break;
        case 3:
          albedo *= vec3(0, 1, 1);
          break;
        default:
          albedo *= vec3(1, 0, 1);
          break;
      }
#endif
    }

    colorFromLight.rgb = colorFromLight.rgb * visibility * albedo;
    fragmentColor += colorFromLight;
  }
#endif

#ifdef ALPHA_MASK
  /* Using <= because if mask is set to 1.0, it should discard all, similarly
     as when using 0, it should only discard what's already invisible
     anyway. */
  if (fragmentColor.a <= alphaMask)
    discard;
#endif

#ifdef OBJECT_ID
  fragmentObjectId = objectId;
#endif
}
