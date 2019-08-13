in mediump vec3 v_color;

uniform highp sampler2D primTexture;
uniform highp int texSize;

layout(location = 0) out mediump vec4 color;
layout(location = 1) out uint objectId;

void main () {
  color = vec4(v_color, 1.0);
  objectId = uint(
      texture(primTexture,
              vec2((float(gl_PrimitiveID % texSize) + 0.5f) / float(texSize),
                   (float(gl_PrimitiveID / texSize) + 0.5f) / float(texSize)))
          .r + 0.5);
}
