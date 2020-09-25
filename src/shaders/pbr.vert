// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ input ------------------------
layout(location = 0) in highp vec4 vertexPosition;
layout(location = 1) in highp vec3 vertexNormal;
#if defined(ALBEDO_TEXTURE) || defined(ROUGHNESS_TEXTURE) || defined(METALLIC_TEXTURE) || defined(NORMAL_TEXTURE)
layout(location = 2) in mediump vec2 vertexTexCoord;
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENTS)
layout(location = 3) in highp vec4 vertexTangent;
#endif

// -------------- output ---------------------
// position, normal, tangent in camera space
out highp vec3 position;
out highp vec3 normal;
#if defined(ALBEDO_TEXTURE) || defined(ROUGHNESS_TEXTURE) || defined(METALLIC_TEXTURE) || defined(NORMAL_TEXTURE)
out mediump vec2 texCoord;
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENTS)
out highp vec3 tangent;
out highp vec3 biTangent;
#endif

// ------------ uniform ----------------------
uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;  // inverse transpose of 3x3 modelview matrix
uniform mat4 MVP;

void main() {
  position = vec3(ModelViewMatrix * vertexPosition);
  normal = normalize(NormalMatrix * vertexNormal);
#if defined(ALBEDO_TEXTURE) || defined(ROUGHNESS_TEXTURE) || defined(METALLIC_TEXTURE) || defined(NORMAL_TEXTURE)
  texCoord = vertexTexCoord;
#endif
#if defined(NORMAL_TEXTURE) && defined(PRECOMPUTED_TANGENTS)
  tangent = normalize(NormalMatrix * vec3(vertexTangent));
  // Gram–Schmidt
  tangent = normalize(tangent - dot(tangent, normal) * normal);
  biTangent = normalize(cross(normal, tangent) * vertexTangent.w);
  // later in .frag, TBN will transform the normal perturbation
  // (read from normal map) from tangent space to camera space
#endif

  gl_Position = MVP * vec4(vertexPosition, 1.0);
}
