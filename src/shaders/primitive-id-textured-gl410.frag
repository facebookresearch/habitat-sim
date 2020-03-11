in mediump vec3 v_color;
flat in mediump uint v_objectId;
in int gl_PrimitiveID;

layout(location = 0) out mediump vec4 color;
layout(location = 1) out uint objectId;
layout(location = 2) out int triangleId;

void main () {
  color = vec4(v_color, 1.0);
  objectId = v_objectId;
  triangleId = gl_PrimitiveID;
}
