// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ uniform ----------------------
uniform highp vec2 FocalLength;
uniform highp vec2 PrincipalPointOffset;
uniform float fx = 600;
uniform float fy = 600;
uniform float Alpha;
uniform float Xi;

#if defined(COLOR_TEXTURE)
uniform samplerCube ColorTexture;
#endif

// ------------ output -----------------------
out vec4 fragmentColor;

void main(void) {
  vec3 m;
  m.xy = (gl_FragCoord.xy - PrincipalPointOffset) / FocalLength;
  float r2 = dot(m.xy, m.xy);
  float sq1 = 1.0 - (2 * Alpha - 1.0) * r2;
  if (sq1 < 0.0) {
    discard;
  }
  m.z = (1 - Alpha * Alpha * r2) / (Alpha * sqrt(sq1) + 1.0 - Alpha);
  float mz2 = m.z * m.z;
  float sq2 = mz2 + (1 - Xi * Xi) * r2;
  if (sq2 < 0.0) {
    discard;
  }

  // unproject to get the ray direction
  // vec3 ray = (m.z * Xi + sqrt(sq2)) / (mz2 + r2) * m - vec3(0.0, 0.0, Xi);
  vec3 ray = m;

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(ray));
#endif
  // XXX debug:
  // fragmentColor = vec4(m.xy, 0.0, 1.0);
  // fragmentColor = vec4(gl_FragCoord.xy / FocalLength - vec2(v, v), 0.0, 1.0);
  // fragmentColor = vec4(PrincipalPointOffset / 1200, 0.0, 1.0);
}
