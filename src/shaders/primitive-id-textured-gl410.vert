uniform highp mat4 transformationProjectionMatrix;

// IDs corresponding to Magnum's generic attribute definitions and
// PrimitiveIDShader::{Position,Color3}
layout(location = 0) in highp vec4 position;
layout(location = 3) in mediump vec3 color;
layout(location = 5) in mediump uint objectId;

out mediump vec3 v_color;
flat out mediump uint v_objectId;

void main() {
  gl_Position = transformationProjectionMatrix * vec4(position.xyz, 1.0);
  v_color = color;
  v_objectId = objectId;
}
