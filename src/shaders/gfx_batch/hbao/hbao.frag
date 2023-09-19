/*
 * Copyright (c) 2014-2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2014-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

precision highp float;

#if AO_LAYERED == 1
#extension GL_ARB_shader_image_load_store : enable
#endif

/*
Based on DeinterleavedTexturing sample by Louis Bavoil
https://github.com/NVIDIAGameWorks/D3DSamples/tree/master/samples/DeinterleavedTexturing

*/

struct HBAOData {
  float RadiusToScreen;  // radius
  float R2;              // 1/radius
  float NegInvR2;        // radius * radius
  float NDotVBias;

  vec2 InvFullResolution;
  vec2 InvQuarterResolution;

  float AOMultiplier;
  float PowExponent;
  vec2 _pad0;
  /*
   * Projection information
   * idx 0 : 2 / proj_mat[0.0]
   * idx 1 : 2 / proj_mat[1,1]
   * idx 2 : x / proj_mat[0.0]
   * idx 3 : y / proj_mat[1,1]
   * where (x, y) are
   * if orthographic : (-(1.0f + proj_mat[3][0]), -(1.0f - proj_mat[3][1]))
   * if perspective :  (-(1.0f - proj_mat[2][0]), -(1.0f + proj_mat[2][1]))
   */
  vec4 projInfo;
  vec2 projScale;
  int projOrtho;
  int _pad1;

  vec4 float2Offsets[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
  vec4 jitters[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
};

// The pragma below is critical for optimal performance
// in this fragment shader to let the shader compiler
// fully optimize the maths and batch the texture fetches
// optimally

#pragma optionNV(unroll all)
// Use the below to not unroll loops, which will decrease performance by half
// but speed up link time by 10-20x
//#pragma optionNV(inline 0)

#define M_PI 3.14159265f

// tweakables
const float NUM_STEPS = 4;
// texRandom/g_Jitter initialization depends on this
const float NUM_DIRECTIONS = 8;

layout(std140) uniform controlBuffer {
  HBAOData control;
};

#ifdef AO_DEINTERLEAVED

#if AO_LAYERED
vec2 g_Float2Offset = control.float2Offsets[gl_PrimitiveID].xy;
vec4 g_Jitter = control.jitters[gl_PrimitiveID];

uniform sampler2DArray texLinearDepth;
uniform sampler2D texViewNormal;

vec3 getQuarterCoord(vec2 UV) {
  return vec3(UV, float(gl_PrimitiveID));
}

#if AO_LAYERED == 1

#ifdef AO_SPECIAL_BLUR
layout(rg16f) uniform image2DArray imgOutput;
#else   // ifndef AO_SPECIAL_BLUR
layout(r8) uniform image2DArray imgOutput;
#endif  // ifdef AO_SPECIAL_BLUR

void outputColor(vec4 color) {
  imageStore(imgOutput, ivec3(ivec2(gl_FragCoord.xy), gl_PrimitiveID), color);
}
#else   //#if AO_LAYERED > 1 (if 2)
out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif  // if AO_LAYERED == 1 else 2
#else   // if !AO_LAYERED (AO_LAYERED == 0)
uniform vec2 g_Float2Offset;
uniform vec4 g_Jitter;

uniform sampler2D texLinearDepth;
uniform sampler2D texViewNormal;

vec2 getQuarterCoord(vec2 UV) {
  return UV;
}

out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif  // if AO_LAYERED

#else  //  if !AO_DEINTERLEAVED

#if AO_TEXTUREARRAY_LAYER  // AO_TEXTUREARRAY_LAYER == 1
uniform sampler2DArray texLinearDepth;
uniform float g_LinearDepthSlice;
#else                      // if AO_TEXTUREARRAY_LAYER == 0
uniform sampler2D texLinearDepth;
#endif                     // if AO_TEXTUREARRAY_LAYER

uniform sampler2DArray texRandom;
uniform float g_RandomSlice;

out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif                     // ifdef AO_DEINTERLEAVED

#ifdef USE_GEOMETRY_SHADER_PASSTHROUGH
in vec2 texCoordGeometry;
#else
in vec2 texCoord;
#endif

//----------------------------------------------------------------------------------

vec3 UVToView(vec2 uv, float eye_z) {
  return vec3((uv * control.projInfo.xy + control.projInfo.zw) *
                  (control.projOrtho != 0 ? 1. : eye_z),
              eye_z);
}

#ifdef AO_DEINTERLEAVED

vec3 FetchQuarterResViewPos(vec2 UV) {
  float ViewDepth = textureLod(texLinearDepth, getQuarterCoord(UV), 0).x;
  return UVToView(UV, ViewDepth);
}

#else  // AO_DEINTERLEAVED

vec3 FetchViewPos(vec2 UV) {
  float ViewDepth = textureLod(texLinearDepth,
#if AO_TEXTUREARRAY_LAYER
                               vec3(UV, g_LinearDepthSlice)
#else
                               UV
#endif
                                   ,
                               0)
                        .x;
  return UVToView(UV, ViewDepth);
}

vec3 MinDiff(vec3 P, vec3 Pr, vec3 Pl) {
  vec3 V1 = Pr - P;
  vec3 V2 = P - Pl;
  return (dot(V1, V1) < dot(V2, V2)) ? V1 : V2;
}

vec3 ReconstructNormal(vec2 UV, vec3 P) {
  vec3 Pr = FetchViewPos(UV + vec2(control.InvFullResolution.x, 0));
  vec3 Pl = FetchViewPos(UV + vec2(-control.InvFullResolution.x, 0));
  vec3 Pt = FetchViewPos(UV + vec2(0, control.InvFullResolution.y));
  vec3 Pb = FetchViewPos(UV + vec2(0, -control.InvFullResolution.y));
  return normalize(cross(MinDiff(P, Pr, Pl), MinDiff(P, Pt, Pb)));
}

#endif  // AO_DEINTERLEAVED

//----------------------------------------------------------------------------------
float Falloff(float DistanceSquare) {
  // 1 scalar mad instruction
  return DistanceSquare * control.NegInvR2 + 1.0;
}

//----------------------------------------------------------------------------------
// P = view-space position at the kernel center
// N = view-space normal at the kernel center
// S = view-space position of the current sample
//----------------------------------------------------------------------------------
float ComputeAO(vec3 P, vec3 N, vec3 S) {
  vec3 V = S - P;
  float VdotV = dot(V, V);
  float NdotV = dot(N, V) * 1.0 / sqrt(VdotV);

  // Use saturate(x) instead of max(x,0.f) because that is faster on Kepler
  return clamp(NdotV - control.NDotVBias, 0, 1) * clamp(Falloff(VdotV), 0, 1);
}

//----------------------------------------------------------------------------------
vec2 RotateDirection(vec2 Dir, vec2 CosSin) {
  return vec2(Dir.x * CosSin.x - Dir.y * CosSin.y,
              Dir.x * CosSin.y + Dir.y * CosSin.x);
}

//----------------------------------------------------------------------------------
vec4 GetJitter() {
#ifdef AO_DEINTERLEAVED
  // Get the current jitter vector from the per-pass constant buffer
  return g_Jitter;
#else
  // (cos(Alpha),sin(Alpha),rand1,rand2)
  return textureLod(
      texRandom, vec3(gl_FragCoord.xy / AO_RANDOMTEX_SIZE, g_RandomSlice), 0);
#endif
}

//----------------------------------------------------------------------------------
float ComputeCoarseAO(vec2 FullResUV,
                      float RadiusPixels,
                      vec4 Rand,
                      vec3 ViewPosition,
                      vec3 ViewNormal) {
#ifdef AO_DEINTERLEAVED
  RadiusPixels /= 4.0;
#endif

  // Divide by NUM_STEPS+1 so that the farthest samples are not fully attenuated
  float StepSizePixels = RadiusPixels / (NUM_STEPS + 1);

  const float Alpha = 2.0 * M_PI / NUM_DIRECTIONS;
  float AO = 0;

  for (float DirectionIndex = 0; DirectionIndex < NUM_DIRECTIONS;
       ++DirectionIndex) {
    float Angle = Alpha * DirectionIndex;

    // Compute normalized 2D direction
    vec2 Direction = RotateDirection(vec2(cos(Angle), sin(Angle)), Rand.xy);

    // Jitter starting sample within the first step
    float RayPixels = (Rand.z * StepSizePixels + 1.0);

    for (float StepIndex = 0; StepIndex < NUM_STEPS; ++StepIndex) {
#ifdef AO_DEINTERLEAVED
      vec2 SnappedUV =
          round(RayPixels * Direction) * control.InvQuarterResolution +
          FullResUV;
      vec3 S = FetchQuarterResViewPos(SnappedUV);
#else
      vec2 SnappedUV =
          round(RayPixels * Direction) * control.InvFullResolution + FullResUV;
      vec3 S = FetchViewPos(SnappedUV);
#endif

      RayPixels += StepSizePixels;

      AO += ComputeAO(ViewPosition, ViewNormal, S);
    }
  }

  AO *= control.AOMultiplier / (NUM_DIRECTIONS * NUM_STEPS);
  return clamp(1.0 - AO * 2.0, 0, 1);
}

//----------------------------------------------------------------------------------
void main() {
#ifdef AO_DEINTERLEAVED
  vec2 base = floor(gl_FragCoord.xy) * 4.0 + g_Float2Offset;
  vec2 uv = base * (control.InvQuarterResolution / 4.0);

  vec3 ViewPosition = FetchQuarterResViewPos(uv);
  vec4 NormalAndAO = texelFetch(texViewNormal, ivec2(base), 0);
  vec3 ViewNormal = -(NormalAndAO.xyz * 2.0 - 1.0);
#else
  vec2 uv =
#ifdef USE_GEOMETRY_SHADER_PASSTHROUGH
      texCoordGeometry
#else
      texCoord
#endif
      ;
  vec3 ViewPosition = FetchViewPos(uv);

  // Reconstruct view-space normal from nearest neighbors
  vec3 ViewNormal = -ReconstructNormal(uv, ViewPosition);
#endif

  // Compute projection of disk of radius control.R into screen space
  float RadiusPixels =
      control.RadiusToScreen / (control.projOrtho != 0 ? 1.0 : ViewPosition.z);

  // Get jitter vector for the current full-res pixel
  vec4 Rand = GetJitter();

  float AO = ComputeCoarseAO(uv, RadiusPixels, Rand, ViewPosition, ViewNormal);

#ifdef AO_SPECIAL_BLUR
  outputColor(vec4(pow(AO, control.PowExponent), ViewPosition.z, 0, 0));
#else
  outputColor(vec4(pow(AO, control.PowExponent)));
#endif
}
