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
const float KERNEL_RADIUS = 3.0f;

uniform float uGaussSharpness;
uniform vec2 uGaussInvResDirection;
// either set x to 1/width or y to 1/height

uniform sampler2D uTexSource;

in vec2 texCoord;

out vec4 out_Color;

//-------------------------------------------------------------------------

float BlurFunction(vec2 uv,
                   float r,
                   float center_c,
                   float center_d,
                   inout float w_total) {
  vec2 aoz = texture(uTexSource, uv).xy;
  float c = aoz.x;
  float d = aoz.y;

  const float BlurSigma = float(KERNEL_RADIUS) * 0.5;
  const float BlurFalloff = 1.0 / (2.0 * BlurSigma * BlurSigma);

  float ddiff = (d - center_d) * uGaussSharpness;
  float w = exp2(-r * r * BlurFalloff - ddiff * ddiff);
  w_total += w;

  return c * w;
}

void main() {
  vec2 aoz = texture(uTexSource, texCoord).xy;
  float center_c = aoz.x;
  float center_d = aoz.y;

  float c_total = center_c;
  float w_total = 1.0;

  for (float r = 1.0f; r <= KERNEL_RADIUS; ++r) {
    vec2 uv = texCoord + uGaussInvResDirection * r;
    c_total += BlurFunction(uv, r, center_c, center_d, w_total);
  }

  for (float r = 1.0f; r <= KERNEL_RADIUS; ++r) {
    vec2 uv = texCoord - uGaussInvResDirection * r;
    c_total += BlurFunction(uv, r, center_c, center_d, w_total);
  }

// This is the 2nd pass of the blur algorithm (ie bluring the previous pass)
#ifdef AO_BLUR_SECOND_PASS
  out_Color = vec4(c_total / w_total);
#else
  // First pass of blur algorithm
  out_Color = vec4(c_total / w_total, center_d, 0, 0);
#endif
}
