in mediump vec3 v_color;

uniform highp usampler2D primTexture;

layout(location = 0) out mediump vec4 color;
layout(location = 1) out uint objectId;

void main () {
  color = vec4(v_color, 1.0);
  objectId = texelFetch(primTexture, ivec2(
      gl_PrimitiveID % PRIMITIVE_TEXTURE_WIDTH,
      gl_PrimitiveID / PRIMITIVE_TEXTURE_WIDTH), 0).r;
}
