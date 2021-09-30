// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#if defined(SHADOWS_VSM)
precision highp float;
uniform samplerCube ShadowMap[3];
const float vsmBias = 0.008f;

float computeShadowUpperBound(vec2 moments, float fragLinearDepth) {
  // eliminates cubemap boundary thin line
  // p == 0 if moments.x < fragLinearDepth;
  // p == 1 otherwise
  // reference:
  // https://github.com/sydneyzh/variance_shadow_mapping_vk/
  float p = step(fragLinearDepth, moments.x + vsmBias);

  float variance = max(moments.y - moments.x * moments.x, 0.000001);
  float d = fragLinearDepth - moments.x;
  // avoid VSM light-bleeding
  // see here:
  // https://dontnormalize.me/2014/01/19/variance-shadow-maps/
  float pMax = smoothstep(0.4, 1.0, variance / (variance + d * d));
  // float pMax = variance / (variance + d * d);

  return max(p, pMax);
}

float computeShadowVSM(int idx, vec3 fragPos, vec3 lightPos) {
  if (idx >= 3) {
    return 1.0;  // at most 3 shadow map are supported
  }
  vec3 lightToFrag = fragPos - lightPos;
  float d = length(lightToFrag);

  vec2 moments = texture(ShadowMap[idx], normalize(lightToFrag)).xy;
  // moments.x is the mean value while moments.y equals to depth * depth
  return computeShadowUpperBound(moments, d);
}

vec3 visualizePointShadowMap(int idx, vec3 fragPos, vec3 lightPos) {
  // get vector between fragment position and light position
  vec3 lightToFrag = fragPos - lightPos;
  // use the fragment to light vector to sample from the depth map
  float depth = texture(ShadowMap[idx], normalize(lightToFrag)).r;
  const float lightFar = 20.0f;  // can be a uniform in the future
  float d = depth / lightFar;
  return vec3(0.0, 0.0, d);
}

#endif
