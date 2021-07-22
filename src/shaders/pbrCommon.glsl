// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tre

precision highp float;

const float PI = 3.14159265358979;

// Specular D, normal distribution function (NDF),
// also known as ggxDistribution
// normal: normal direction
// light: light source direction
// viwer: camera direction, aka light outgoing direction
// n_dot_h: dot product of normal vector and the halfVector (half vector of light and view)
//          usually n_dot_h = clamp(dot(normal, halfVector), 0.0, 1.0);
float normalDistributionGGX(float n_dot_h, float roughness) {
  float a = roughness * roughness;
  float a2 = a * a;

  float d = n_dot_h * n_dot_h * (a2 - 1.0) + 1.0;
  d = PI * d * d;

  return a2 / d;
}
