uniform highp mat4 transformationProjectionMatrix;

// IDs corresponding to Magnum's generic attribute definitions and
// PrimitiveIDTexturedShader::{Position,Color3}
layout(location = 0) in highp vec4 position;
layout(location = 3) in mediump vec3 color;

out mediump vec3 v_color;

void main() {
  gl_Position = transformationProjectionMatrix * vec4(position.xyz, 1.0);
  v_color = color;
}
