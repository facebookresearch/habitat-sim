// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

out vec2 texCoord;

void main() {
  int idx = gl_VertexID % 3;  // allows rendering multiple fullscreen triangles
  vec4 pos = vec4((float(idx & 1)) * 4.0 - 1.0,
                  (float((idx >> 1) & 1)) * 4.0 - 1.0, 0, 1.0);
  gl_Position = pos;
  texCoord = pos.xy * 0.5 + 0.5;
}
