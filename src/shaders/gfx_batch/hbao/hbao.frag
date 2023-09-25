// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

precision highp float;
precision highp sampler2DArray;

// These are defined when shader is built in Habitat.
#ifndef AO_LAYERED
#define AO_LAYERED 1
#endif

#ifndef AO_RANDOMTEX_SIZE
#define AO_RANDOMTEX_SIZE 4
#endif

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

const float M_PI = 3.14159265f;

// tweakables
const float NUM_STEPS = 4.0f;
// texRandom/uJitter initialization depends on this
const float NUM_DIRECTIONS = 8.0f;

layout(std140) uniform uControlBuffer {
  HBAOData control;
};

#ifdef AO_DEINTERLEAVED

uniform sampler2DArray texLinearDepth;
uniform sampler2D texViewNormal;

#if AO_LAYERED
vec2 uFloat2Offset = control.float2Offsets[gl_PrimitiveID].xy;
vec4 uJitter = control.jitters[gl_PrimitiveID];

vec3 getQuarterCoord(vec2 UV) {
  return vec3(UV, float(gl_PrimitiveID));
}

#if AO_LAYERED == 1

#ifdef AO_SPECIAL_BLUR
layout(rg16f) uniform image2DArray uImgOutput;
#else   // ifndef AO_SPECIAL_BLUR
layout(r8) uniform image2DArray uImgOutput;
#endif  // ifdef AO_SPECIAL_BLUR

void outputColor(vec4 color) {
  imageStore(uImgOutput, ivec3(ivec2(gl_FragCoord.xy), gl_PrimitiveID), color);
}
#else   //#if AO_LAYERED > 1 (if 2)

out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif  // if AO_LAYERED == 1 else 2
#else   // if !AO_LAYERED (AO_LAYERED == 0)

#ifdef AO_TEXTUREARRAY_LAYER
uniform float uLinearDepthSlice;
#endif  // AO_TEXTUREARRAY_LAYER

uniform vec2 uFloat2Offset;
uniform vec4 uJitter;

vec2 getQuarterCoord(vec2 UV) {
  return UV;
}

out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif  // if AO_LAYERED

#else   //  if !AO_DEINTERLEAVED

uniform sampler2D texLinearDepth;

uniform sampler2DArray texRandom;
uniform float uRandomSlice;

out vec4 out_Color;

void outputColor(vec4 color) {
  out_Color = color;
}
#endif  // ifdef AO_DEINTERLEAVED

#ifndef USE_GEOMETRY_SHADER_PASSTHROUGH
#if AO_LAYERED == 2
in vec2 texGeomCoord;
#else   // AO_LAYERED != 2
in vec2 texCoord;
#endif  // AO_LAYERED
#else   // USE_GEOMETRY_SHADER_PASSTHROUGH defined
in vec2 texCoord;
#endif  // ifndef USE_GEOMETRY_SHADER_PASSTHROUGH

//----------------------------------------------------------------------------------

vec3 UVToView(vec2 uv, float eye_z) {
  return vec3((uv * control.projInfo.xy + control.projInfo.zw) *
                  (control.projOrtho != 0 ? 1. : eye_z),
              eye_z);
}

#ifdef AO_DEINTERLEAVED

vec3 FetchQuarterResViewPos(vec2 UV) {
  float ViewDepth = textureLod(texLinearDepth,
#ifdef AO_TEXTUREARRAY_LAYER
                               vec3(UV, uLinearDepthSlice),
#else
                               getQuarterCoord(UV),
#endif  // AO_TEXTUREARRAY_LAYER
                               0.0f)
                        .x;

  return UVToView(UV, ViewDepth);
}

#else  // not AO_DEINTERLEAVED

vec3 FetchViewPos(vec2 UV) {
  float ViewDepth = textureLod(texLinearDepth, UV, 0.0f).x;
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
  return clamp(NdotV - control.NDotVBias, 0.0f, 1.0f) *
         clamp(Falloff(VdotV), 0.0f, 1.0f);
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
  return uJitter;
#else
  // (cos(Alpha),sin(Alpha),rand1,rand2)
  return textureLod(
      texRandom, vec3(gl_FragCoord.xy / float(AO_RANDOMTEX_SIZE), uRandomSlice),
      0.0f);
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
  float StepSizePixels = RadiusPixels / (NUM_STEPS + 1.0f);

  const float Alpha = 2.0 * M_PI / NUM_DIRECTIONS;
  float AO = 0.0f;

  for (float DirectionIndex = 0.0f; DirectionIndex < NUM_DIRECTIONS;
       ++DirectionIndex) {
    float Angle = Alpha * DirectionIndex;

    // Compute normalized 2D direction
    vec2 Direction = RotateDirection(vec2(cos(Angle), sin(Angle)), Rand.xy);

    // Jitter starting sample within the first step
    float RayPixels = (Rand.z * StepSizePixels + 1.0);

    for (float StepIndex = 0.0f; StepIndex < NUM_STEPS; ++StepIndex) {
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
  return clamp(1.0 - AO * 2.0, 0.0f, 1.0f);
}

//----------------------------------------------------------------------------------
void main() {
#ifdef AO_DEINTERLEAVED
  vec2 base = floor(gl_FragCoord.xy) * 4.0 + uFloat2Offset;
  vec2 uv = base * (control.InvQuarterResolution / 4.0);

  vec3 ViewPosition = FetchQuarterResViewPos(uv);
  vec4 NormalAndAO = texelFetch(texViewNormal, ivec2(base), 0);
  vec3 ViewNormal = -(NormalAndAO.xyz * 2.0 - 1.0);
#else  // AO_DEINTERLEAVED
  vec2 uv =
#ifndef USE_GEOMETRY_SHADER_PASSTHROUGH
#if AO_LAYERED == 2
      texGeomCoord;
#else
      texCoord;
#endif
#else
      texCoord;
#endif
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
