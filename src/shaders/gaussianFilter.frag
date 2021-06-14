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
  float scale = dims.x;
  if (!FilterHorizontally) {
    scale = dims.y;
  }

  const int samples = 3;
  float offset[samples] = float[]( 0.0, 1.0, 2.0);
  float weight[samples] = float[]( 0.2270270270, 0.3162162162, 0.0702702703 );

  // If you see artifact, then tune the vsmBias in the shadowsVSM.glsl
  /*
  const int samples = 5;
  float offset[samples] = float[]( 0.0, 1.0, 2.0, 3.0, 4.0 );
  float weight[samples] = float[]( 0.2270270270, 0.1945945946, 0.1216216216, 0.0540540541, 0.0162162162 );
  */

  // fragmentColor = texture2D(SourceTexture, vec2(gl_FragCoord) / scale) * weight[0];
  fragmentColor = texture2D(SourceTexture, textureCoordinates) * weight[0];
  for (int i = 1; i < samples; ++i) {
    if (FilterHorizontally) {
      // x axis
      /*
      fragmentColor += texture2D(SourceTexture, (vec2(gl_FragCoord) + vec2(offset[i], 0.0)) / scale) * weight[i];
      fragmentColor += texture2D(SourceTexture, (vec2(gl_FragCoord) - vec2(offset[i], 0.0)) / scale) * weight[i];
      */
      fragmentColor += texture2D(SourceTexture, textureCoordinates + vec2(offset[i], 0.0) / scale) * weight[i];
      fragmentColor += texture2D(SourceTexture, textureCoordinates - vec2(offset[i], 0.0) / scale) * weight[i];
    } else {
      // y axis
      /*
      fragmentColor += texture2D(SourceTexture, (vec2(gl_FragCoord) + vec2(0.0, offset[i])) / scale) * weight[i];
      fragmentColor += texture2D(SourceTexture, (vec2(gl_FragCoord) - vec2(0.0, offset[i])) / scale) * weight[i];
      */
      fragmentColor += texture2D(SourceTexture, textureCoordinates + vec2(0.0, offset[i]) / scale) * weight[i];
      fragmentColor += texture2D(SourceTexture, textureCoordinates - vec2(0.0, offset[i]) / scale) * weight[i];
    }
  } // for

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
