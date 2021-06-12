// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;

#if defined(SHADOWS) && defined(SHADOWS_PCF)
// ------------ uniform ----------------------
// let us try 1 point shadow first
uniform samplerCube PointShadowMap0;
uniform float LightNearPlane;
uniform float LightFarPlane;


// for instant depth visualization purpose
float LinearizeDepth(float depth) {
    float z = depth * 2.0 - 1.0; // back to NDC
    return (2.0 * LightNearPlane * LightFarPlane) /
      (LightFarPlane + LightNearPlane - z * (LightFarPlane - LightNearPlane));
}

// array of offset direction for sampling
vec3 gridSamplingDisk[20] = vec3[](
   vec3(1, 1,  1), vec3( 1, -1,  1), vec3(-1, -1,  1), vec3(-1, 1,  1),
   vec3(1, 1, -1), vec3( 1, -1, -1), vec3(-1, -1, -1), vec3(-1, 1, -1),
   vec3(1, 1,  0), vec3( 1, -1,  0), vec3(-1, -1,  0), vec3(-1, 1,  0),
   vec3(1, 0,  1), vec3(-1,  0,  1), vec3( 1,  0, -1), vec3(-1, 0, -1),
   vec3(0, 1,  1), vec3( 0, -1,  1), vec3( 0, -1, -1), vec3( 0, 1, -1)
);

// see here:
// https://stackoverflow.com/questions/10786951/omnidirectional-shadow-mapping-with-depth-cubemap/10789527#10789527
float vecToDepthValue(vec3 vec) {
  vec3 v = abs(vec);
  float originalZ = max(v.x, max(v.y, v.z));

  float temp = LightFarPlane - LightNearPlane;
  float z = (LightFarPlane + LightNearPlane) / temp -
      (2 * LightFarPlane * LightNearPlane) / (temp * originalZ);
  return (z + 1.0) * 0.5;
}

float computeShadow(vec3 fragPos, vec3 lightPos, vec3 viewPos) {
    // vector from light position to the shading location=
    vec3 lightToFrag = fragPos - lightPos;
    float d = vecToDepthValue(lightToFrag);

    float bias = 0.00001;
    int samples = 20;
    float shadow = 0.0;
    float viewDistance = length(viewPos - fragPos);
    float diskRadius = (1.0 + (viewDistance / LightFarPlane)) / 50.0;
    for(int i = 0; i < samples; ++i) {
        float closestDepth = texture(PointShadowMap0, normalize(lightToFrag + gridSamplingDisk[i] * diskRadius)).r;
        if(d - bias > closestDepth)
            shadow += 1.0;
    }
    shadow /= float(samples);

    return shadow;
}

vec3 visualizePointShadowMap(vec3 fragPos, vec3 lightPos) {
    // get vector between fragment position and light position
    vec3 lightToFrag = fragPos - lightPos;
    // use the fragment to light vector to sample from the depth map
    float depth = texture(PointShadowMap0, lightToFrag).r;
    float d = LinearizeDepth(depth) / LightFarPlane;
    return vec3(0.0, 0.0, d);
}
#endif
