in mediump vec3 v_color;
//in int gl_PrimitiveID;

layout(location = 0) out mediump vec4 color;
layout(location = 1) out int triangleId;

void main() {
    color = vec4(v_color, 1.0);
    triangleId = 7;
}
