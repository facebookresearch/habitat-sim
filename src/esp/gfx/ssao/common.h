// todo: figure out licensing and header here
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

#define VERTEX_POS 0
#define VERTEX_NORMAL 1
#define VERTEX_COLOR 2

#define UBO_SCENE 0

#define AO_RANDOMTEX_SIZE 4

// todo: clean up this file
#ifdef __cplusplus
namespace esp {
namespace gfx {
namespace ssao {
using namespace nvmath;
#endif

struct SceneData {
  mat4 viewProjMatrix;
  mat4 viewMatrix;
  mat4 viewMatrixIT;

  uvec2 viewport;
  uvec2 _pad;
};

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

  vec4 projInfo;
  vec2 projScale;
  int projOrtho;
  int _pad1;

  vec4 float2Offsets[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
  vec4 jitters[AO_RANDOMTEX_SIZE * AO_RANDOMTEX_SIZE];
};

#ifdef __cplusplus
}
}
}
#endif
