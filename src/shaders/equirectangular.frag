// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ uniform ----------------------

uniform highp int ViewportHeight;
uniform highp int ViewportWidth;

#if defined(COLOR_TEXTURE)
uniform samplerCube ColorTexture;
#endif

#if defined(DEPTH_TEXTURE)
uniform samplerCube DepthTexture;
#endif

// ------------ output -----------------------
#if defined(COLOR_TEXTURE)
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out vec4 fragmentColor;
#endif

void main(void) {
  vec3 m;
  const float PI = 3.1415926535897932384626433832795;
  float phi = gl_FragCoord.x * 2 * PI / ViewportWidth;
  float theta = gl_FragCoord.y * PI / ViewportHeight;
  m.x = sin(phi) * sin(theta);
  m.y = cos(theta) * -1;
  m.z = cos(phi) * sin(theta) * -1;

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(m));
#endif
#if defined(DEPTH_TEXTURE)
  gl_FragDepth = texture(DepthTexture, normalize(m)).r;
#endif
}
