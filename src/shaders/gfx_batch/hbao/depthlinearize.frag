// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

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
  }
  // orthographic projection
  return (clipInfo[1] + clipInfo[2] - depth * clipInfo[1]);
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
