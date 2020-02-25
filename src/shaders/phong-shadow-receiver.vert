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

#define POSITION_ATTRIBUTE_LOCATION 0
#define TEXTURECOORDINATES_ATTRIBUTE_LOCATION 1
#define NORMAL_ATTRIBUTE_LOCATION 2
#define COLOR_ATTRIBUTE_LOCATION 3
#define TANGENT_ATTRIBUTE_LOCATION 4

uniform highp mat4 transformationMatrix = mat4(1.0);

uniform highp mat4 projectionMatrix = mat4(1.0);

#if LIGHT_COUNT
uniform mediump mat3 normalMatrix = mat3(1.0);
#endif

#if LIGHT_COUNT
// for shadows
uniform highp mat4 modelMatrix = mat4(1.0);

/* Needs to be last because it uses locations 10 to 10 + LIGHT_COUNT - 1 */
uniform highp vec3 lightPositions[LIGHT_COUNT]; /* defaults to zero */

// for shadows
uniform highp mat4 shadowmapMatrix[NUM_SHADOW_MAP_LEVELS];
#endif

layout(location = POSITION_ATTRIBUTE_LOCATION) in highp vec4 position;

#if LIGHT_COUNT
layout(location = NORMAL_ATTRIBUTE_LOCATION) in mediump vec3 normal;

#ifdef NORMAL_TEXTURE
layout(location = TANGENT_ATTRIBUTE_LOCATION) in mediump vec3 tangent;
#endif
#endif

#ifdef TEXTURED
layout(location = TEXTURECOORDINATES_ATTRIBUTE_LOCATION) in mediump vec2
    textureCoords;

out mediump vec2 interpolatedTextureCoords;
#endif

#ifdef VERTEX_COLOR
layout(location = COLOR_ATTRIBUTE_LOCATION) in lowp vec4 vertexColor;

out lowp vec4 interpolatedVertexColor;
#endif

#if LIGHT_COUNT
out mediump vec3 transformedNormal;
out mediump vec3 absoluteTransformedNormal;
#ifdef NORMAL_TEXTURE
out mediump vec3 transformedTangent;
#endif
out highp vec3 lightDirections[LIGHT_COUNT];
out highp vec3 shadowCoords[NUM_SHADOW_MAP_LEVELS];
out highp vec3 cameraDirection;
#endif

void main() {
  /* Transformed vertex position */
  highp vec4 transformedPosition4 = transformationMatrix * position;
  highp vec3 transformedPosition =
      transformedPosition4.xyz / transformedPosition4.w;

#if LIGHT_COUNT
  /* Transformed normal and tangent vector */
  transformedNormal = normalMatrix * normal;
#ifdef NORMAL_TEXTURE
  transformedTangent = normalMatrix * tangent;
#endif

  /* Direction to the light */
  for (int i = 0; i < LIGHT_COUNT; ++i)
    lightDirections[i] = normalize(lightPositions[i] - transformedPosition);

  /* Direction to the camera */
  cameraDirection = -transformedPosition;

  /* Shadows */

  /* normal in world coordinates */
  absoluteTransformedNormal = mat3(modelMatrix) * normal;

  vec4 worldPos4 = modelMatrix * position;
  for (int i = 0; i < shadowmapMatrix.length(); i++) {
    shadowCoords[i] = (shadowmapMatrix[i] * worldPos4).xyz;
  }
#endif

  /* Transform the position */
  gl_Position = projectionMatrix * transformedPosition4;

#ifdef TEXTURED
  /* Texture coordinates, if needed */
  interpolatedTextureCoords = textureCoords;
#endif

#ifdef VERTEX_COLOR
  /* Vertex colors, if enabled */
  interpolatedVertexColor = vertexColor;
#endif
}
