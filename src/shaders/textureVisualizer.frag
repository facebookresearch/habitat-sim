// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
precision highp float;

// ------------ input -----------------------
in highp vec2 textureCoordinates;

// ------------ uniforms --------------------
#ifdef DEPTH_TEXTURE
uniform highp sampler2D sourceTexture;
#endif

#ifdef OBJECT_ID_TEXTURE
uniform highp usampler2D sourceTexture;
#endif

uniform highp sampler2D colorMapTexture;
uniform highp vec2 colorMapOffsetScale;
#define colorMapOffset colorMapOffsetScale.x
#define colorMapScale colorMapOffsetScale.y

#ifdef DEPTH_TEXTURE
uniform highp vec2 depthUnprojection;
#endif

//------------- output ----------------------
#ifdef EXPLICIT_ATTRIB_LOCATION
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR)
#endif
out highp vec4 fragmentColor;

//------------- shader ----------------------
void main() {
#ifdef DEPTH_TEXTURE
  highp float depth = texture(sourceTexture, textureCoordinates).r;
  highp float originalDepth =
      depthUnprojection[1] / (depth + depthUnprojection[0]);
#elif defined(OBJECT_ID_TEXTURE)
  highp uint objectId = texture(sourceTexture, textureCoordinates).r;
#endif

  fragmentColor = texture(colorMapTexture, vec2(
    /* Object/primitive IDs are constant across the whole primitive so we
       do the offset/scale mapping here */
    #if defined(DEPTH_TEXTURE) || defined(OBJECT_ID_TEXTURE)
    colorMapOffset + float(
        #ifdef DEPTH_TEXTURE
        originalDepth
        #elif defined(OBJECT_ID_TEXTURE)
        objectId
        #else
        #error sorry, the type of the texture to be visualized is unknown.
        #endif
    ) * colorMapScale, 0.0));
    #else
    0.0, 0.0));
    #endif
}
