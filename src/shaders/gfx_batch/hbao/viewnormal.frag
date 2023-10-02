// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;
in vec2 texCoord;
/*
 * Projection information
 * idx 0 : 2 / proj_mat[0.0]
 * idx 1 : 2 / proj_mat[1,1]
 * idx 2 : x / proj_mat[0.0]
 * idx 3 : y / proj_mat[1,1]
 * where (x, y) are
 * if orthographic : (-(1.0f + proj_mat[3][0]), -(1.0f - proj_mat[3][1]))
 * if perspective :  (-(1.0f - proj_mat[2][0]), -(1.0f + proj_mat[2][1]))
 */
uniform vec4 uProjInfo;
uniform int uProjOrtho;
uniform vec2 uInvFullResolution;

uniform sampler2D uTexLinearDepth;

out vec4 out_Color;

//----------------------------------------------------------------------------------

vec3 UVToView(vec2 uv, float eye_z) {
  return vec3(
      (uv * uProjInfo.xy + uProjInfo.zw) * (uProjOrtho != 0 ? 1. : eye_z),
      eye_z);
}

vec3 FetchViewPos(vec2 UV) {
  float ViewDepth = textureLod(uTexLinearDepth, UV, 0.0f).x;
  return UVToView(UV, ViewDepth);
}

vec3 MinDiff(vec3 P, vec3 Pr, vec3 Pl) {
  vec3 V1 = Pr - P;
  vec3 V2 = P - Pl;
  return (dot(V1, V1) < dot(V2, V2)) ? V1 : V2;
}

vec3 ReconstructNormal(vec2 UV, vec3 P) {
  vec3 Pr = FetchViewPos(UV + vec2(uInvFullResolution.x, 0));
  vec3 Pl = FetchViewPos(UV + vec2(-uInvFullResolution.x, 0));
  vec3 Pt = FetchViewPos(UV + vec2(0, uInvFullResolution.y));
  vec3 Pb = FetchViewPos(UV + vec2(0, -uInvFullResolution.y));
  return normalize(cross(MinDiff(P, Pr, Pl), MinDiff(P, Pt, Pb)));
}

//----------------------------------------------------------------------------------

void main() {
  vec3 P = FetchViewPos(texCoord);
  vec3 N = ReconstructNormal(texCoord, P);

  out_Color = vec4(N * 0.5 + 0.5, 0);
}
