// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tre

precision highp float;

// ------------ input ------------------------
in highp vec4 position; // world position

// ------------ uniform ----------------------
uniform uint SampleCounts;
uniform samplerCube EnvironmentMap;

// -------------- output ---------------------
out highp vec4 fragmentColor;

// -------------- shader ---------------------

// Hacker's Delight, Henry S. Warren, 2001, ISBN:0201914654
 float radicalInverse_VdC(uint bits) {
     bits = (bits << 16u) | (bits >> 16u);
     bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
     bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
     bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
     bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
     return float(bits) * 2.3283064365386963e-10; // / 0x100000000
 }

// This is based on
// Holger Dammertz, Hammersley Points on the Hemisphere (2012)
// http://holger.dammertz.org/stuff/notes_HammersleyOnHemisphere.html
 vec2 hammersley2d(uint i, uint N) {
     return vec2(float(i) / float(N), radicalInverse_VdC(i));
 }

 const float PI = 3.14159265358979;

 // Uniform Mapping
 vec3 hemisphereSample_uniform(float u, float v) {
     float phi = v * 2.0 * PI;
     float cosTheta = 1.0 - u;
     float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
     return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
 }

 // Cosinus Mapping
 vec3 hemisphereSample_cos(float u, float v) {
     float phi = v * 2.0 * PI;
     float cosTheta = sqrt(1.0 - u);
     float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
     return vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
 }

 void main() {
   vec3 normal = normalize(position.xyz); // N (z axis)
   vec3 up = vec3(0.0, 1.0, 0.0); // U (y axis)
   vec3 right = normalize(cross(up, normal)); // R (x axis)
   up = normalize(cross(normal, right));

   vec3 irradiance = vec3(0.0);
   for (uint iPoint = 0u; iPoint < SampleCounts; ++iPoint) {
     // sample point on 2D plane
     vec2 xi = hammersley2d(iPoint, SampleCounts);

     // spherical to cartesian (in tangent space)
     vec3 tangentSample = hemisphereSample_cos(xi.u, xi.v);

     // tangent space to world: [R U N][tangentSample] = xR + yU + zN
     vec3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * normal;

     // Careful: we used cosinus mapping so there is no need to multiply cos(theta) * sin(theta)
     irradiance += texture(environmentMap, sampleVec).rgb;
   }

   fragmentColor = irradiance / float(SampleCounts);
 }
