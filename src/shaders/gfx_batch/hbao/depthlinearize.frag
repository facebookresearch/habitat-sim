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
precision highp float;

// Holds clip plane and projection-type info
// idx 0 : zNear * zFar
// idx 1 : zNear - zFar
// idx 2 : zFar
// idx 3 : 1 == perspective, 0 == orthographic
uniform vec4 uClipInfo;

#ifdef DEPTHLINEARIZE_MSAA
uniform int uSampleIndex;
uniform sampler2DMS uInputTexture;
#else
uniform sampler2D uInputTexture;
#endif

out float out_Color;

float reconstructCSZ(float depth, vec4 clipInfo) {
  if (clipInfo[3] == 1.0f) {
    // perspective projection
    return (clipInfo[0] / (clipInfo[1] * depth + clipInfo[2]));
  } else {
    // orthographic projection
    return (clipInfo[1] + clipInfo[2] - depth * clipInfo[1]);
  }
}
/*
    if (in_perspective == 1.0) // perspective
        ze = (zNear * zFar) / (zFar - zb * (zFar - zNear));
    else // orthographic proj
        ze  = zNear + zb  * (zFar - zNear);
*/
void main() {
#ifdef DEPTHLINEARIZE_MSAA
  float depth =
      texelFetch(uInputTexture, ivec2(gl_FragCoord.xy), uSampleIndex).x;
#else
  float depth = texelFetch(uInputTexture, ivec2(gl_FragCoord.xy), 0).x;
#endif

  out_Color = reconstructCSZ(depth, uClipInfo);
}
