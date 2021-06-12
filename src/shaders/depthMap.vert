// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
precision highp float;

// ------------ input ------------------------
// position in model local space
layout(location = ATTRIBUTE_LOCATION_POSITION) in highp vec4 vertexPosition;

uniform highp mat4 LightModelViewMatrix;
uniform highp mat4 LightProjectionMatrix;

void main() {
    gl_Position = LightProjectionMatrix * LightModelViewMatrix * vertexPosition;
}
