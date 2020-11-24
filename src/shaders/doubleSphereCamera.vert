// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
void main() {
  // a big triangle that definitely covers the screen.
  gl_Position = vec4((gl_VertexID == 2) ? 3.0 : -1.0,
                     (gl_VertexID == 1) ? -3.0 : 1.0, 0.0, 1.0);
}
