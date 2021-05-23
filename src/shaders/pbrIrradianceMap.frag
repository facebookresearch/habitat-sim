// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tre

precision highp float;

// ------------ input ------------------------
in highp vec4 position; // world position

// ------------ uniform ----------------------
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

 // Uniform Mapping (not used, but put here for reference)
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
   const uint sampleCounts = 8192u;
   for (uint iPoint = 0u; iPoint < sampleCounts; ++iPoint) {
     // sample point on 2D plane
     vec2 xi = hammersley2d(iPoint, sampleCounts);

     // spherical to cartesian (in tangent space) z-up
     // z-up (not y-up) is fine here
     vec3 tangentSample = hemisphereSample_cos(xi.u, xi.v);

     // tangent space to world: [R U N][tangentSample] = xR + yU + zN
     // we make the normal dirction in world space aligned with z-axis in tangent space
     vec3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * normal;

     // Careful:
     // We used cosinus mapping not uniform mapping.
     // We generated proportionally fewer rays at the center top of the hemisphere.
     // So there is no need to compensate for the smaller areas
     // by scaling the area by sinTheta
     irradiance += texture(environmentMap, sampleVec).rgb;
   }

   fragmentColor = irradiance / float(sampleCounts);
 }
