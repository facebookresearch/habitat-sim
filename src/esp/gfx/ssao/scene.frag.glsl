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


#version 430
/**/

#extension GL_ARB_shading_language_include : enable
#include "common.h"

layout(std140,binding=UBO_SCENE) uniform sceneBuffer {
  SceneData   scene;
};

in Interpolants {
  vec3 pos;
  vec3 normal;
  flat vec4 color;
} IN;

layout(location=0,index=0) out vec4 out_Color;

void main()
{
  vec3  light = normalize(vec3(-1,2,1));
  float intensity = dot(normalize(IN.normal),light) * 0.5 + 0.5;
  vec4  color = IN.color * mix(vec4(0,0.25,0.75,0),vec4(1,1,1,0),intensity);

  out_Color = color;
}
