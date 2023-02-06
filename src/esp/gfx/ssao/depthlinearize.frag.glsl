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

#ifndef DEPTHLINEARIZE_USEMSAA
#define DEPTHLINEARIZE_USEMSAA 0
#endif

layout(location=0) uniform vec4 clipInfo; // z_n * z_f,  z_n - z_f,  z_f, perspective = 1 : 0

#if DEPTHLINEARIZE_MSAA
layout(location=1) uniform int sampleIndex;
layout(binding=0)  uniform sampler2DMS inputTexture;
#else
layout(binding=0)  uniform sampler2D inputTexture;
#endif

layout(location=0,index=0) out float out_Color;

float reconstructCSZ(float d, vec4 clipInfo) {
  if (clipInfo[3] != 0) {
    return (clipInfo[0] / (clipInfo[1] * d + clipInfo[2]));
  }
  else {
    return (clipInfo[1]+clipInfo[2] - d * clipInfo[1]);
  }
}
/*
    if (in_perspective == 1.0) // perspective
    {
        ze = (zNear * zFar) / (zFar - zb * (zFar - zNear));
    }
    else // orthographic proj
    {
        ze  = zNear + zb  * (zFar - zNear);
    }
*/
void main() {
#if DEPTHLINEARIZE_MSAA
  float depth = texelFetch(inputTexture, ivec2(gl_FragCoord.xy), sampleIndex).x;
#else
  float depth = texelFetch(inputTexture, ivec2(gl_FragCoord.xy), 0).x;
#endif

  out_Color = reconstructCSZ(depth, clipInfo);
}
