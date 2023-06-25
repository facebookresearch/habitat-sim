// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tre

precision highp float;

// ------------ input ------------------------
in highp vec4 position;  // world position

// ------------ uniform ----------------------
uniform samplerCube EnvironmentMap;
uniform float Roughness;

// -------------- output ---------------------
layout(location = OUTPUT_ATTRIBUTE_LOCATION_COLOR) out highp vec4 fragmentColor;

// ------------ shader -----------------------

// Compute a halfway vector that is biased towards the preferred alignment
// direction (importance sampling). The implementation is based on: Křivánek J,
// Colbert M. Real‐time shading with filtered importance sampling Computer
// Graphics Forum. Wiley/Blackwell (10.1111), 2008, 27(4): 1147-1154.
// http://dcgi.felk.cvut.cz/publications/2008/krivanek-cgf-rts
//
// https://github.com/SaschaWillems/Vulkan-glTF-PBR/
// https://github.com/KhronosGroup/glTF-Sample-Viewer/
// http://blog.selfshadow.com/publications/s2013-shading-course/karis/s2013_pbs_epic_slides.pdf
// https://www.tobias-franke.eu/log/2014/03/30/notes_on_importance_sampling.html

vec3 importanceSamplingGGX(vec2 xi, float roughness, vec3 normal) {
  // Maps a 2D point to a hemisphere with spread based on roughness
  float alpha = roughness * roughness;
  // In https://github.com/SaschaWillems/Vulkan-glTF-PBR/ it has one extra term
  // "random(normal.xz) * 0.1;". Here we follow
  // https://github.com/KhronosGroup/glTF-Sample-Viewer/
  // and do not have it here.
  float phi = TWO_PI * xi.x;

  float cosTheta = clamp(
      sqrt((1.0 - xi.y) / (1.0 + (alpha * alpha - 1.0) * xi.y)), 0.0, 1.0);
  float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

  // build halfway vector in cartesian coordinates (local tangent space)
  vec3 h = vec3(sinTheta * cos(phi), sinTheta * sin(phi), cosTheta);

  // Tangent space
  vec3 up = abs(normal.z) < 0.999 ? vec3(0.0, 0.0, 1.0) : vec3(1.0, 0.0, 0.0);
  vec3 right = normalize(cross(up, normal));
  up = normalize(cross(normal, right));

  // Convert halfvector from tangent space to world Space: [R U N] * h
  return normalize(right * h.x + up * h.y + normal * h.z);
}

// https://placeholderart.wordpress.com/2015/07/28/implementation-notes-runtime-environment-map-filtering-for-image-based-lighting/
void main() {
  vec3 normal = normalize(position.xyz);  // normal
  // in the name of speed, assuming that V equals R equals N
  //
  vec3 r = normal;  // R: reflection
  vec3 v = r;       // V: view (from the shading location to the camera)
  vec3 prefilteredColor = vec3(0.0);
  float totalWeight = 0.0;

  float imageSize = float(textureSize(EnvironmentMap, 0).s);
  // solid angle of 1 pixel across all cube faces
  float solidAnglePixel = 4.0 * PI / (6.0 * imageSize * imageSize);
  // fewer samples due to filtered importance sampling
  const uint sampleCounts = 32u;
  float invSampleCounts = 1.0f / float(sampleCounts);

  for (uint iPoint = 0u; iPoint < sampleCounts; ++iPoint) {
    // sample point on 2D plane
    vec2 xi = hammersley2d(iPoint, sampleCounts);
    vec3 h = importanceSamplingGGX(xi, Roughness, normal);
    vec3 l = normalize(2.0 * dot(v, h) * h - v);

    float n_dot_l = clamp(dot(normal, l), 0.0, 1.0);
    if (n_dot_l > 0.0) {
      float n_dot_h = clamp(dot(normal, h), 0.0, 1.0);
      float v_dot_h = clamp(dot(v, h), 0.00001, 1.0);

      // Probability Distribution Function
      float pdf =
          normalDistributionGGX(n_dot_h, Roughness) * n_dot_h / (4.0 * v_dot_h);

      // Solid angle for the current sample point
      float solidAngle = invSampleCounts / pdf;

      // Biased (+1.0) mip level for better result
      // see:
      // https://github.com/SaschaWillems/Vulkan-glTF-PBR/
      float mipLevel =
          Roughness == 0.0
              ? 0.0
              : max(0.5 * log2(solidAngle / solidAnglePixel) + 1.0, 0.0);

      prefilteredColor += textureLod(EnvironmentMap, l, mipLevel).rgb * n_dot_l;
      totalWeight += n_dot_l;
    }  // if
  }

  fragmentColor = vec4(prefilteredColor / totalWeight, 1.0);
}
