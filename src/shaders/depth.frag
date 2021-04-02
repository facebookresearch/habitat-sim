#ifdef UNPROJECT_EXISTING_DEPTH
uniform highp sampler2D depthTexture;
uniform highp vec2 depthUnprojection;

in highp vec2 textureCoordinates;
#else
in highp float depth;
#endif

out highp float originalDepth;

void main() {
  #ifdef UNPROJECT_EXISTING_DEPTH
  highp float depth = texture(depthTexture, textureCoordinates).r;
  originalDepth =
    #ifndef NO_FAR_PLANE_PATCHING
    /* We can afford using == for comparison as 1.0f has an exact
       representation and the depth is cleared to exactly this value. */
    depth == 1.0 ? 0.0 :
    #endif
    depthUnprojection[1] / (depth + depthUnprojection[0]);
  #else
  originalDepth = depth;
  #endif
}
