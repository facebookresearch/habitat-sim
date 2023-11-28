// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

// ------------ input ------------------------
// position, normal, tangent in model local space
layout(location = ATTRIBUTE_LOCATION_POSITION) in highp vec4 vertexPosition;
layout(location = ATTRIBUTE_LOCATION_NORMAL) in highp vec3 vertexNormal;
#if defined(TEXTURED)
layout(location = ATTRIBUTE_LOCATION_TEXCOORD) in highp vec2 vertexTexCoord;
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
layout(location = ATTRIBUTE_LOCATION_TANGENT4) in highp vec4 vertexTangent;
#endif

#ifdef VERTEX_COLOR
layout(location = ATTRIBUTE_LOCATION_COLOR) in highp vec4 vertexColor;
#endif

#ifdef JOINT_COUNT
#if PER_VERTEX_JOINT_COUNT
layout(location = ATTRIBUTE_LOCATION_WEIGHTS) in mediump vec4 weights;
layout(location = ATTRIBUTE_LOCATION_JOINTIDS) in mediump uvec4 jointIds;
#endif  // PER_VERTEX_JOINT_COUNT

#if SECONDARY_PER_VERTEX_JOINT_COUNT
layout(location = SECONDARY_WEIGHTS_ATTRIBUTE_LOCATION) in mediump vec4
    secondaryWeights;

layout(location = SECONDARY_JOINTIDS_ATTRIBUTE_LOCATION) in mediump uvec4
    secondaryJointIds;
#endif  // SECONDARY_PER_VERTEX_JOINT_COUNT
#endif

// -------------- output ---------------------
// position, normal, tangent in *world* space, NOT camera space!
out highp vec3 position;
out highp vec3 normal;
#if defined(TEXTURED)
out highp vec2 texCoord;

#if defined(TEXTURE_TRANSFORMATION)
uniform highp mat3 uTextureMatrix
#ifndef GL_ES
    = mat3(1.0)
#endif
    ;
#endif
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
out highp vec3 tangent;
out highp vec3 biTangent;
#endif
#ifdef VERTEX_COLOR
out highp vec4 interpolatedVertexColor;
#endif
// ------------ uniform ----------------------
uniform highp mat4 uViewMatrix;
uniform highp mat3 uNormalMatrix;  // inverse transpose of 3x3 model matrix, NOT
                                   // modelview matrix
uniform highp mat4 uModelMatrix;
uniform highp mat4 uProjectionMatrix;

// -------------- skin-related uniforms --------------
#ifdef DYNAMIC_PER_VERTEX_JOINT_COUNT
uniform mediump uvec2 perVertexJointCount
#ifndef GL_ES
    = uvec2(PER_VERTEX_JOINT_COUNT, SECONDARY_PER_VERTEX_JOINT_COUNT)
#endif
    ;
#endif  // DYNAMIC_PER_VERTEX_JOINT_COUNT

#ifdef JOINT_COUNT
uniform mat4 jointMatrices[JOINT_COUNT]
#ifndef GL_ES
    = mat4[](JOINT_MATRIX_INITIALIZER)
#endif
    ;
// Defaults to 0
uniform uint perInstanceJointCount;
#endif  // JOINT_COUNT

// ------------ shader -----------------------
void main() {
  //------------ skin support

// Build skin transformation matrix
// See Magnum phong.vert for source of this code
#ifdef JOINT_COUNT
  mediump uint jointOffset = uint(gl_InstanceID) * perInstanceJointCount;
  mat4 skinMatrix = mat4(0.0);
#if PER_VERTEX_JOINT_COUNT
  for (uint i = 0u; i != PER_VERTEX_JOINT_COUNT
#ifdef DYNAMIC_PER_VERTEX_JOINT_COUNT
                    && i != perVertexJointCount.x
#endif  // DYNAMIC_PER_VERTEX_JOINT_COUNT
       ;
       ++i)
    skinMatrix += weights[i] * jointMatrices[jointOffset + jointIds[i]];
#endif  // PER_VERTEX_JOINT_COUNT

#if SECONDARY_PER_VERTEX_JOINT_COUNT
  for (uint i = 0u; i != SECONDARY_PER_VERTEX_JOINT_COUNT
#ifdef DYNAMIC_PER_VERTEX_JOINT_COUNT
                    && i != perVertexJointCount.y
#endif  // DYNAMIC_PER_VERTEX_JOINT_COUNT
       ;
       ++i)
    skinMatrix +=
        secondaryWeights[i] * jointMatrices[jointOffset + secondaryJointIds[i]];
#endif  // SECONDARY_PER_VERTEX_JOINT_COUNT
#endif  // JOINT_COUNT

  //------------ end skin support

  vec4 vertexWorldPosition = uModelMatrix *
#ifdef JOINT_COUNT
                             skinMatrix *
#endif
                             vertexPosition;

  position = vertexWorldPosition.xyz;
  normal = normalize(uNormalMatrix * vertexNormal);
#if defined(TEXTURED)
  texCoord =
#if defined(TEXTURE_TRANSFORMATION)
      (uTextureMatrix * vec3(vertexTexCoord, 1.0)).xy;
#else
      vertexTexCoord;
#endif  // TEXTURE_TRANSFORMATION
#endif  // TEXTURED

#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENT)
  tangent = normalize(uNormalMatrix * vec3(vertexTangent));
  // Gram–Schmidt
  tangent = normalize(tangent - dot(tangent, normal) * normal);
  biTangent = normalize(cross(normal, tangent) * vertexTangent.w);
  // later in .frag, TBN will transform the normal perturbation
  // (read from normal map) from tangent space to world space,
  // NOT camera space
#endif
#ifdef VERTEX_COLOR
  /* Vertex colors, if enabled */
  interpolatedVertexColor = vertexColor;
#endif

  gl_Position = uProjectionMatrix * uViewMatrix * vertexWorldPosition;
}
