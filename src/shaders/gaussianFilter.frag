// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// This is an implementation based on:
// https://rastergrid.com/blog/2010/09/efficient-gaussian-blur-with-linear-sampling/

precision highp float;

// ------------ input -----------------------
in highp vec2 textureCoordinates;

// ------------ uniforms --------------------
uniform highp sampler2D SourceTexture;
uniform bool FilterHorizontally; // true on x or false on y axis

//------------- output ----------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out highp vec4 fragmentColor;

//------------- shader ----------------------
//
void main(void) {
  ivec2 dims = textureSize(SourceTexture, 0);  // lod = 0
  float scale = 1.0 / dims.x;
  if (!FilterHorizontally) {
    scale = 1.0 / dims.y;
  }

  const int samples = 3;
  float weight[samples] = float[](0.2270270270, 0.3162162162, 0.0702702703);

  // If you see artifact, then tune the vsmBias in the shadowsVSM.glsl
  // const int samples = 5;
  // float weight[samples] = float[](0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162);

  vec3 result = texture(SourceTexture, textureCoordinates).rgb * weight[0];
  for (int i = 1; i < samples; ++i) {
    if (FilterHorizontally) {
      // x axis
      result += texture(SourceTexture, textureCoordinates + vec2(scale * i, 0.0)).rgb * weight[i];
      result += texture(SourceTexture, textureCoordinates - vec2(scale * i, 0.0)).rgb * weight[i];
    } else {
      // y axis
      result += texture(SourceTexture, textureCoordinates + vec2(0.0, scale * i)).rgb * weight[i];
      result += texture(SourceTexture, textureCoordinates - vec2(0.0, scale * i)).rgb * weight[i];
    }
  } // for
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
