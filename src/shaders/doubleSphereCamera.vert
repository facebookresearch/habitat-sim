// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ input ------------------------
/*
the vertex position of the cube in camera space
NOTE:
The cube is in the camera space;
The cube center is at origin and 8 vertices are:
( 1.0,  1.0,  1.0)
(-1.0,  1.0,  1.0)
( 1.0, -1.0,  1.0)
(-1.0, -1.0,  1.0)
( 1.0,  1.0, -1.0)
(-1.0,  1.0, -1.0)
( 1.0, -1.0, -1.0)
(-1.0, -1.0, -1.0)
*/

layout(location = ATTRIBUTE_LOCATION_POSITION) in highp vec4 cubeVertexPosition;
// -------------- output ---------------------

// ------------ uniform ----------------------
// in this case, the projection matrix is actually the mvp
uniform highp mat4 mvp;

// ------------ shader -----------------------
void main() {
  gl_Position = mvp * cubeVertexPosition;
}
