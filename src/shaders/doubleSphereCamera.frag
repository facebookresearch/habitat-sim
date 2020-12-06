// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

// ------------ uniform ----------------------
uniform vec2 FocalLength;
uniform vec2 PrincipalPointOffset;
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
  vec3 ray = (m.z * Xi + sqrt(sq2)) / (mz2 + r2) * m - vec3(0.0, 0.0, Xi);

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(ray));
#endif
  fragmentColor = gl_FragCoord.xy / vec2(800, 600);
}
