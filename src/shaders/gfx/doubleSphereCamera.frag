// Copyright (c) Meta Platforms, Inc. and its affiliates.
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

void main(void) {
  vec3 m;
  m.xy = (gl_FragCoord.xy - PrincipalPointOffset) / FocalLength;
  float r2 = dot(m.xy, m.xy);
  float sq1 = 1.0 - (2.0 * Alpha - 1.0) * r2;
  if (sq1 < 0.0)
    discard;
  m.z = (1.0 - Alpha * Alpha * r2) / (Alpha * sqrt(sq1) + 1.0 - Alpha);
  float mz2 = m.z * m.z;
  // unproject to get the ray direction
  float sq2 = mz2 + (1.0 - Xi * Xi) * r2;
  if (sq2 < 0.0)
    discard;

  // Careful!
  // one cannot flip the z at this point,
  // otherwise a wrong offset would be introducted in the "ray"
  // based on the following equation.

  // unproject to get the ray direction
  vec3 ray = (m.z * Xi + sqrt(sq2)) / (mz2 + r2) * m - vec3(0.0, 0.0, Xi);

  // So far, coordinates are computed in the right-handed Cartesian system.
  // However, OpenGL cubemap uses a left-handed system with z points inwards,
  // (https://www.khronos.org/opengl/wiki/Cubemap_Texture)
  // so flip the z here:
  ray.z = -ray.z;

#if defined(COLOR_TEXTURE)
  fragmentColor = texture(ColorTexture, normalize(ray));
#endif
#if defined(DEPTH_TEXTURE)
  gl_FragDepth = texture(DepthTexture, normalize(ray)).r;
#endif
#if defined(OBJECT_ID_TEXTURE)
  fragmentObjectId = texture(ObjectIdTexture, normalize(ray)).r;
#endif
}
