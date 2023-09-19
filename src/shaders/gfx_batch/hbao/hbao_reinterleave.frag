// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;
precision highp sampler2DArray;
uniform sampler2DArray uTexResultsArray;

out vec4 out_Color;

//----------------------------------------------------------------------------------

void main() {
  ivec2 FullResPos = ivec2(gl_FragCoord.xy);
  ivec2 Offset = FullResPos & 3;
  int SliceId = Offset.y * 4 + Offset.x;
  ivec2 QuarterResPos = FullResPos >> 2;

#ifdef AO_SPECIAL_BLUR
  out_Color = vec4(
      texelFetch(uTexResultsArray, ivec3(QuarterResPos, SliceId), 0).xy, 0, 0);

#else
  out_Color =
      vec4(texelFetch(uTexResultsArray, ivec3(QuarterResPos, SliceId), 0).x);

#endif
}
