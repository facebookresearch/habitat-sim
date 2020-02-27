
layout(location = 0) in highp vec4 position;
layout(location = 1) in int gl_PrimitiveID;

out int v_triangle_id;

void main() {
    gl_Position = position;
    v_triangle_id = gl_PrimitiveID;
}
