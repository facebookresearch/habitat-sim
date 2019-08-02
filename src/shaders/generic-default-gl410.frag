in vec3 v_color;
in float v_depth;


#ifdef PER_VERTEX_IDS
flat in uint v_objectId;
#else
uniform highp int objectIdUniform;
#endif

#ifdef TEXTURED
uniform lowp sampler2D textureData;
in mediump vec2 interpolatedTextureCoordinates;
#endif

#ifdef ID_TEXTURED
uniform highp sampler2D primTexture;
uniform highp int texSize;
#endif

#ifndef VERTEX_COLORED
uniform lowp vec4 colorUniform;
#endif

layout(location = 0) out vec4 color;
layout(location = 1) out float depth;
layout(location = 2) out uint objectId;

void main () {
  vec4 baseColor =
    #ifdef VERTEX_COLORED
    vec4(v_color, 1.0);
    #else
    colorUniform;
    #endif
  color =
    #ifdef TEXTURED
    texture(textureData, interpolatedTextureCoordinates) *
    #endif
    baseColor;
  depth = v_depth;
  objectId =
  #ifdef PER_VERTEX_IDS
    v_objectId;
  #else
    uint(objectIdUniform);
  #endif

  #ifdef ID_TEXTURED
  objectId = uint(
      texture(primTexture,
              vec2((float(gl_PrimitiveID % texSize) + 0.5f) / float(texSize),
                   (float(gl_PrimitiveID / texSize) + 0.5f) / float(texSize)))
          .r + 0.5);
  #endif
}
