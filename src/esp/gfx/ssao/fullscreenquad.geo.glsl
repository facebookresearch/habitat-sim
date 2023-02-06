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

layout(triangles) in;

#extension GL_NV_geometry_shader_passthrough : enable

#if GL_NV_geometry_shader_passthrough

  layout(passthrough) in gl_PerVertex {
    vec4 gl_Position;
  } gl_in[];
  layout(passthrough) in Inputs {
    vec2 texCoord;
  } IN[];

  void main()
  {
    gl_Layer = gl_PrimitiveIDIn;
    gl_PrimitiveID = gl_PrimitiveIDIn;
  }

#else

  layout(triangle_strip,max_vertices=3) out;

  in Inputs {
    vec2 texCoord;
  } IN[];
  out vec2 texCoord;

  void main()
  {
    for (int i = 0; i < 3; i++){
      texCoord = IN[i].texCoord;
      gl_Layer = gl_PrimitiveIDIn;
      gl_PrimitiveID = gl_PrimitiveIDIn;
      gl_Position = gl_in[i].gl_Position;
      EmitVertex();
    }
  }

#endif
