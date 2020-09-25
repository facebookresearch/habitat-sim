// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ input ------------------------
layout(location = 0) in highp vec4 vertexPosition;
layout(location = 1) in mediump vec3 vertexNormal;
#if defined(ALBEDO_TEXTURE) || defined(ROUGHNESS_TEXTURE) || defined(METALLIC_TEXTURE) || defined(NORMAL_TEXTURE)
layout(location = 2) in mediump vec3 vertexTexCoord;
#endif
#if defined(NORMAL_TEXTURE)
layout(location = 3) in mediump vec3 vertexTengent;
#endif

// -------------- output ---------------------
// position, normal, tangent in camera space
out vec3 Position;
out vec3 Normal;
#ifdef NORMAL_TEXTURE
out vec3 Tangent;
// out vec3 tangentLightPosition[MAX_NUM_POINT_LIGHTS];
#endif

// ------------ uniform ----------------------
uniform mat4 ModelViewMatrix;
uniform mat3 NormalMatrix;  // inverse transpose of 3x3 modelview matrix
uniform mat4 MVP;

void main() {
  gl_Position = MVP * vec4(vertexPosition, 1.0);
}
