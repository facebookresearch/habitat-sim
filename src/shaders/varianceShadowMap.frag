// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
precision highp float;

// ------------ input ------------------------
// fragment position in *light* space
in highp vec3 position;
// ------------ output -----------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out vec4 fragmentColor;
// ------------ shader -----------------------
void main() {
  // Careful!
  // abs(position.z) is the depth in the light space.
  // The following is the depth in the world space!!
  fragmentColor = vec4(0.0, 0.0, 0.0, 0.0);
  float d = length(position);
  fragmentColor.x = d;
  fragmentColor.y = d * d;
}
