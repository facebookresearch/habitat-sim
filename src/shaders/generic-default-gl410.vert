uniform highp mat4 transformationProjectionMatrix;
uniform highp mat4 projectionMatrix;

layout(location = 0) in highp vec4 position;

#ifdef TEXTURED
layout(location = 1) in mediump vec2 textureCoordinates;
out mediump vec2 interpolatedTextureCoordinates;
#endif

#ifdef VERTEX_COLORED
layout(location = 1) in vec3 color;
#endif

out vec3 v_color;
out float v_depth;

#ifdef PER_VERTEX_IDS
flat out uint v_objectId;
#endif

void main() {
  gl_Position = transformationProjectionMatrix * vec4(position.xyz, 1.0);

  #ifdef TEXTURED
  interpolatedTextureCoordinates = textureCoordinates;
  #endif

  vec4 pointInCameraCoords = projectionMatrix * vec4(position.xyz, 1.0);
  pointInCameraCoords /= pointInCameraCoords.w;

  #ifdef VERTEX_COLORED
  v_color = color;
  #endif
  #ifdef PER_VERTEX_IDS
  v_objectId = uint(position.w);
  #endif

  v_depth = -pointInCameraCoords.z;
}
