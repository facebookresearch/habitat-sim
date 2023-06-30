// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This is an implementation based on:
// https://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/

precision highp float;

// ------------ input -----------------------
in highp vec2 textureCoordinates;

// ------------ uniforms --------------------
uniform highp sampler2D SourceTexture;

// (1.0, 0.0) for x direction
// (0.0, 1.0) for y direction
uniform vec2 FilterDirection;

//------------- output ----------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out highp vec4 fragmentColor;

//------------- shader ----------------------
//
void main(void) {
  ivec2 dims = textureSize(SourceTexture, 0);  // lod = 0
  vec2 scales = FilterDirection / dims;

  const int samples = 3;
  float weight[samples] = float[](0.2270270270, 0.3162162162, 0.0702702703);

  // const int samples = 5;
  // float weight[samples] = float[](0.2270270270, 0.1945945946, 0.1216216216,
  // 0.0540540541, 0.0162162162);

  vec3 result = texture(SourceTexture, textureCoordinates).rgb * weight[0];
  for (int i = 1; i < samples; ++i) {
    vec2 offset = scales * i;
    result +=
        texture(SourceTexture, textureCoordinates + offset).rgb * weight[i];
    result +=
        texture(SourceTexture, textureCoordinates - offset).rgb * weight[i];
  }  // for
  fragmentColor = vec4(result, 1.0);

  /*
  // box filter
  fragmentColor = vec4(0.0, 0.0, 0.0, 0.0);
  vec2 uv = gl_FragCoord.xy / scale;
  vec2 stepSize = vec2(1.0 / scale, 0.0);
  if (!FilterHorizontally) {
    stepSize = vec2(0.0, 1.0 / scale);
  }
  const int r = 3;
  vec4 sum = vec4(0.0, 0.0, 0.0, 0.0);
  for(int i = -r; i <= r; ++i)
  {
      sum += texture(SourceTexture, uv + float(i) * stepSize);
  }

  fragmentColor = sum / float(2 * r + 1);
  */
}
