// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

layout(triangles) in;

#ifdef USE_GEOMETRY_SHADER_PASSTHROUGH
#extension GL_NV_geometry_shader_passthrough : enable
#endif

#ifdef USE_GEOMETRY_SHADER_PASSTHROUGH

layout(passthrough) in gl_PerVertex {
  vec4 gl_Position;
}
gl_in[];
layout(passthrough) in Inputs {
  vec2 texCoord;
}
IN[];

void main() {
  gl_Layer = gl_PrimitiveIDIn;
  gl_PrimitiveID = gl_PrimitiveIDIn;
}

#else

layout(triangle_strip, max_vertices = 3) out;

in vec2 texCoord[];

out vec2 texGeomCoord;

void main() {
  for (int i = 0; i < 3; ++i) {
    texGeomCoord = texCoord[i];
    gl_Layer = gl_PrimitiveIDIn;
    gl_PrimitiveID = gl_PrimitiveIDIn;
    gl_Position = gl_in[i].gl_Position;
    EmitVertex();
  }
}

#endif
