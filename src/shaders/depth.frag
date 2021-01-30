#ifdef UNPROJECT_EXISTING_DEPTH
uniform highp sampler2D depthTexture;
uniform highp vec2 depthUnprojection;

in highp vec2 textureCoordinates;
#else
in highp float depth;
#endif
#ifdef DEPTH_VISUALIZATER
uniform highp float far;
out highp vec4 fragmentColor;
#else
out highp float originalDepth;
#endif

void main() {
  #ifdef UNPROJECT_EXISTING_DEPTH
    highp float depth = texture(depthTexture, textureCoordinates).r;
    #ifdef DEPTH_VISUALIZATER
    highp float
    #endif
    originalDepth =
      #ifndef NO_FAR_PLANE_PATCHING
      /* We can afford using == for comparison as 1.0f has an exact
         representation and the depth is cleared to exactly this value. */
      depth == 1.0 ? 0.0 :
      #endif
      depthUnprojection[1] / (depth + depthUnprojection[0]);
  #else
    #ifdef DEPTH_VISUALIZATER
    highp float
    #endif
    originalDepth = depth;
  #endif
  #ifdef DEPTH_VISUALIZATER
    fragmentColor = vec4(vec3(originalDepth / far), 0.0);
  #endif
}
