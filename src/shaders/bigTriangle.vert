// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// For technical details, see here:
// https://rauwendaal.net/2014/06/14/rendering-a-screen-covering-triangle-in-opengl/

/*
v0(-1, 1)  v2(3, 1)
uv(0, 1)   uv(2, 1)
__________
|    |   /
|    |  /
|    | /
|____|/
|    /
|   /
|  /
| /
|/
v1(-1, -3)
uv(0, -1)
*/

#ifdef OUTPUT_UV
out highp vec2 textureCoordinates;
#endif
void main() {
  // a big triangle that definitely covers the screen.
  gl_Position = vec4((gl_VertexID == 2) ? 3.0 : -1.0,
                     (gl_VertexID == 1) ? -3.0 : 1.0, 0.0, 1.0);
#ifdef OUTPUT_UV
  textureCoordinates = gl_Position.xy * 0.5 + vec2(0.5);
#endif
}
