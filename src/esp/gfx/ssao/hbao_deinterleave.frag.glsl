/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */


#version 430

layout(location=0) uniform vec4      info; // xy
vec2 uvOffset = info.xy;
vec2 invResolution = info.zw;

layout(binding=0)  uniform sampler2D texLinearDepth;

layout(location=0,index=0) out float out_Color[8];

//----------------------------------------------------------------------------------

#if 1
void main() {
  vec2 uv = floor(gl_FragCoord.xy) * 4.0 + uvOffset + 0.5;
  uv *= invResolution;

  vec4 S0 = textureGather(texLinearDepth, uv, 0);
  vec4 S1 = textureGatherOffset(texLinearDepth, uv, ivec2(2,0), 0);

  out_Color[0] = S0.w;
  out_Color[1] = S0.z;
  out_Color[2] = S1.w;
  out_Color[3] = S1.z;
  out_Color[4] = S0.x;
  out_Color[5] = S0.y;
  out_Color[6] = S1.x;
  out_Color[7] = S1.y;
}
#else
void main() {
  vec2 uv = floor(gl_FragCoord.xy) * 4.0 + uvOffset;
  ivec2 tc = ivec2(uv);

  out_Color[0] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(0,0)).x;
  out_Color[1] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(1,0)).x;
  out_Color[2] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(2,0)).x;
  out_Color[3] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(3,0)).x;
  out_Color[4] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(0,1)).x;
  out_Color[5] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(1,1)).x;
  out_Color[6] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(2,1)).x;
  out_Color[7] = texelFetchOffset(texLinearDepth, tc, 0, ivec2(3,1)).x;
}

#endif
