// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

const float PI = 3.14159265358979;
const float INV_PI = 1.0 / PI;
const float TWO_PI = 2.0 * PI;
const float EPSILON = 0.000001;

// Use the Hammersley point set in 2D for fast and practical generation of
// hemisphere directions in a shader program. See here:
// Holger Dammertz, Hammersley Points on the Hemisphere (2012)
// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html

// This is an efficient implementation of the Van der Corput radical inverse
// see the above link
float radicalInverse_VdC(uint bits) {
  bits = (bits << 16u) | (bits >> 16u);
  bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
  bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
  bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
  bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
  return float(bits) * 2.3283064365386963e-10;  // / 0x100000000
}

// Hammersley Point Set in 2D
vec2 hammersley2d(uint i, uint N) {
  return vec2(float(i) / float(N), radicalInverse_VdC(i));
}

// uniform distributed direction (z-up) from the hammersley point
vec3 hemisphereSample_uniform(float u, float v) {
  float phi = v * TWO_PI;
  float cosTheta = 1.0 - u;
  float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
  return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

// cosines distributed direction (z-up) from the hammersley point
vec3 hemisphereSample_cos(float u, float v) {
  float phi = v * TWO_PI;
  float cosTheta = sqrt(1.0 - u);
  float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
  return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
}

// Specular D, normal distribution function (NDF),
// also known as ggxDistribution
// normal: normal direction
// light: light source direction
// viwer: camera direction, aka light outgoing direction
// n_dot_h: dot product of normal vector and the halfVector (half vector of
// light and view)
//          usually n_dot_h = clamp(dot(normal, halfVector), 0.0, 1.0);
float normalDistributionGGX(float n_dot_h, float alphaRoughness) {
  float a2 = alphaRoughness * alphaRoughness;

  float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
  d = PI * d * d;

  return a2 / d;
}

// Approx 2.5 speedup over pow with integer coeffs
float pow5(float v) {
  float v2 = v * v;
  return v2 * v2 * v;
}

float pow4(float v) {
  float v2 = v * v;
  return v2 * v2;
}

float pow2(float v) {
  return v * v;
}
