// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tre

precision highp float;

// ------------ input ------------------------
in highp vec4 position; // world position

// ------------ uniform ----------------------
uniform samplerCube EnvironmentMap;

// -------------- output ---------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out highp vec4 fragmentColor;

// -------------- shader ---------------------
 void main() {
   vec3 normal = normalize(position.xyz); // N (z axis)
   vec3 up = vec3(0.0, 1.0, 0.0); // U (y axis)
   vec3 right = normalize(cross(up, normal)); // R (x axis)
   up = normalize(cross(normal, right));

   vec3 irradiance = vec3(0.0);
   const uint sampleCounts = 4096u;
   for (uint iPoint = 0u; iPoint < sampleCounts; ++iPoint) {
     // sample point on 2D plane
     // check pbrCommon.glsl for mor details
     vec2 xi = hammersley2d(iPoint, sampleCounts);

     // spherical to cartesian (in tangent space) z-up
     // z-up (not y-up) is fine here
     // check pbrCommon.glsl for mor details
     vec3 tangentSample = hemisphereSample_cos(xi.x, xi.y);

     // tangent space to world: [R U N][tangentSample] = xR + yU + zN
     // we make the normal dirction in world space aligned with z-axis in tangent space
     vec3 sampleVec = tangentSample.x * right + tangentSample.y * up + tangentSample.z * normal;

     // Careful:
     // We used cosinus mapping not uniform mapping.
     // We generated proportionally fewer rays at the center top of the hemisphere.
     // So there is no need to compensate for the smaller areas
     // by scaling the area by sinTheta
     // irradiance += textureLod(EnvironmentMap, sampleVec, 0.0).rgb;
     // Dev note:
     // if the irradianceMap is full black,
     // check if Environment enables mipmaps and you do populate them correctly.
     irradiance += texture(EnvironmentMap, sampleVec).rgb;
   }

   fragmentColor = vec4(irradiance / float(sampleCounts), 1.0);
 }
