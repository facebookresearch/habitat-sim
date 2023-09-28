// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;
const float KERNEL_RADIUS = 3.0f;

uniform float uGaussSharpness;
uniform vec2 uGaussInvResDirection;
// either set x to 1/width or y to 1/height

uniform sampler2D uTexSource;
uniform sampler2D uTexLinearDepth;

in vec2 texCoord;

out vec4 out_Color;

//-------------------------------------------------------------------------

vec4 BlurFunction(vec2 uv,
                  float r,
                  vec4 center_c,
                  float center_d,
                  inout float w_total) {
  vec4 c = texture(uTexSource, uv);
  float d = texture(uTexLinearDepth, uv).x;

  const float BlurSigma = KERNEL_RADIUS * 0.5f;
  const float BlurFalloff = 1.0 / (2.0 * BlurSigma * BlurSigma);

  float ddiff = (d - center_d) * uGaussSharpness;
  float w = exp2(-r * r * BlurFalloff - ddiff * ddiff);
  w_total += w;

  return c * w;
}

void main() {
  vec4 center_c = texture(uTexSource, texCoord);
  float center_d = texture(uTexLinearDepth, texCoord).x;

  vec4 c_total = center_c;
  float w_total = 1.0;

  for (float r = 1.0f; r <= KERNEL_RADIUS; ++r) {
    vec2 uv = texCoord + uGaussInvResDirection * r;
    c_total += BlurFunction(uv, r, center_c, center_d, w_total);
  }

  for (float r = 1.0f; r <= KERNEL_RADIUS; ++r) {
    vec2 uv = texCoord - uGaussInvResDirection * r;
    c_total += BlurFunction(uv, r, center_c, center_d, w_total);
  }

  out_Color = c_total / w_total;
}
