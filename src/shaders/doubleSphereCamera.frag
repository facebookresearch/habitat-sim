// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
precision highp float;

// ------------ uniform ----------------------
uniform highp vec2 FocalLength;
uniform highp vec2 PrincipalPointOffset;
uniform highp float Alpha;
uniform highp float Xi;

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
  m.xy = (gl_FragCoord.xy - PrincipalPointOffset) / FocalLength;
  // MUST flip the x axis
  m.x = -m.x;
  float r2 = dot(m.xy, m.xy);
  float sq1 = 1.0 - (2.0 * Alpha - 1.0) * r2;
  if (sq1 < 0.0) {
    discard;
  }
  // If you're worried about sqrt() called with a negative value, that's fine,
  // you just get a NaN, which on most drivers would result in a black color
  // output.
  m.z = (1.0 - Alpha * Alpha * r2) / (Alpha * sqrt(sq1) + 1.0 - Alpha);
  float mz2 = m.z * m.z;
  float sq2 = mz2 + (1.0 - Xi * Xi) * r2;
  if (sq2 < 0.0) {
    discard;
  }

  // unproject to get the ray direction
  vec3 ray = (m.z * Xi + sqrt(sq2)) / (mz2 + r2) * m - vec3(0.0, 0.0, Xi);

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(ray));
#endif
#if defined(DEPTH_TEXTURE)
  gl_FragDepth = texture(DepthTexture, normalize(ray)).r;
#endif
}
