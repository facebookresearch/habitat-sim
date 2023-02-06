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

#ifndef AO_BLUR
#define AO_BLUR 1
#endif

layout(binding=0)  uniform sampler2DArray texResultsArray;

layout(location=0,index=0) out vec4 out_Color;

//----------------------------------------------------------------------------------

void main() {
  ivec2 FullResPos = ivec2(gl_FragCoord.xy);
  ivec2 Offset = FullResPos & 3;
  int SliceId = Offset.y * 4 + Offset.x;
  ivec2 QuarterResPos = FullResPos >> 2;

#if AO_BLUR
  out_Color = vec4(texelFetch( texResultsArray, ivec3(QuarterResPos, SliceId), 0).xy,0,0);
#else
  out_Color = vec4(texelFetch( texResultsArray, ivec3(QuarterResPos, SliceId), 0).x);
#endif

}
