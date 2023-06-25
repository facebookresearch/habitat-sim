// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree

// ------------ input ------------------------
// the input is a cube whose vertices are expressed in world space
layout(location = ATTRIBUTE_LOCATION_POSITION) in highp vec4 vertexPosition;

// ------------ uniform ----------------------
uniform highp mat4 ModelViewMatrix;
uniform highp mat4 ProjectionMatrix;

// -------------- output ---------------------
out highp vec4 position;  // world position

// -------------- shader ---------------------
void main() {
  position = vertexPosition;
  gl_Position = ProjectionMatrix * ModelViewMatrix * vertexPosition;
}
