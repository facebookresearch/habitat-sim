#ifndef UNPROJECT_EXISTING_DEPTH
uniform highp mat4 transformationMatrix;
uniform highp mat4 projectionMatrix;
#endif

#ifdef UNPROJECT_EXISTING_DEPTH
out highp vec2 textureCoordinates;
#else
layout(location = 0) in highp vec4 position;
out highp float depth;
#endif

void main() {
  #ifndef UNPROJECT_EXISTING_DEPTH
  vec4 transformed = transformationMatrix*position;
  gl_Position = projectionMatrix*transformed;
  depth = -transformed.z;
  #else
  gl_Position = vec4((gl_VertexID == 2) ?  3.0 : -1.0,
                     (gl_VertexID == 1) ? -3.0 :  1.0, 0.0, 1.0);
  textureCoordinates = gl_Position.xy*0.5 + vec2(0.5);
  #endif
}
