// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
precision highp float;

// ------------ uniform ----------------------

uniform highp int ViewportHeight;
uniform highp int ViewportWidth;

#if defined(COLOR_TEXTURE)
uniform samplerCube ColorTexture;
#endif

#if defined(DEPTH_TEXTURE)
uniform samplerCube DepthTexture;
#endif

#if defined(OBJECT_ID_TEXTURE)
uniform usamplerCube ObjectIdTexture;
#endif

// ------------ output -----------------------
#if defined(COLOR_TEXTURE)
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR)
out highp vec4 fragmentColor;
#endif

#if defined(OBJECT_ID_TEXTURE)
layout(location = OUTPUT_ATTRIBUTE_LOCATION_OBJECT_ID)
out highp uint fragmentObjectId;
#endif

// ------------ shader -----------------------
void main(void) {
  vec3 m;
  const float PI = 3.1415926535897932384626433832795;
  float phi = gl_FragCoord.x * 2.0 * PI / float(ViewportWidth);
  float theta = gl_FragCoord.y * PI / float(ViewportHeight);
  m.x = -sin(phi) * sin(theta);
  m.y = -cos(theta);
  m.z = cos(phi) * sin(theta);

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(m));
#endif
#if defined(DEPTH_TEXTURE)
  gl_FragDepth = texture(DepthTexture, normalize(m)).r;
#endif
#if defined(OBJECT_ID_TEXTURE)
  fragmentObjectId = texture(ObjectIdTexture, normalize(m)).r;
#endif
}
