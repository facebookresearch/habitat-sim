// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifdef USE_TEXTURE_GATHER
#extension GL_ARB_gpu_shader5 : enable
#endif

precision highp float;

// xy are uv offset coordinates
// zw are inverse resolution values
uniform vec4 uUVOffsetInvResInfo;

uniform sampler2D uTexLinearDepth;

out float out_Color[8];

//----------------------------------------------------------------------------------

void main() {
  vec2 uvOffset = uUVOffsetInvResInfo.xy;
  vec2 invResolution = uUVOffsetInvResInfo.zw;
#ifdef USE_TEXTURE_GATHER
  vec2 uv = floor(gl_FragCoord.xy) * 4.0 + uvOffset + 0.5;
  uv *= invResolution;

  vec4 S0 = textureGather(uTexLinearDepth, uv, 0);
  vec4 S1 = textureGatherOffset(uTexLinearDepth, uv, ivec2(2, 0), 0);

  out_Color[0] = S0.w;
  out_Color[1] = S0.z;
  out_Color[2] = S1.w;
  out_Color[3] = S1.z;
  out_Color[4] = S0.x;
  out_Color[5] = S0.y;
  out_Color[6] = S1.x;
  out_Color[7] = S1.y;

#else
  vec2 uv = floor(gl_FragCoord.xy) * 4.0 + uvOffset;
  ivec2 tc = ivec2(uv);

  out_Color[0] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(0, 0)).x;
  out_Color[1] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(1, 0)).x;
  out_Color[2] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(2, 0)).x;
  out_Color[3] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(3, 0)).x;
  out_Color[4] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(0, 1)).x;
  out_Color[5] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(1, 1)).x;
  out_Color[6] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(2, 1)).x;
  out_Color[7] = texelFetchOffset(uTexLinearDepth, tc, 0, ivec2(3, 1)).x;

#endif
}
