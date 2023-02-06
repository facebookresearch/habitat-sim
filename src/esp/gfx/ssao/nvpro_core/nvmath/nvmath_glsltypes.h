/*
 * Copyright (c) 2012-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2012-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#ifndef NV_SHADER_TYPES_H
#define NV_SHADER_TYPES_H

#include "../nvp/NvFoundation.h"
#include "nvmath_types.h"

namespace nvmath {

#if defined(__amd64__) || defined(__x86_64__) || defined(_M_X64) || \
    defined(__AMD64__)
// Matrices, must align to 4 vector (16 bytes)
NV_ALIGN(16, typedef mat3f mat3);
NV_ALIGN(16, typedef mat4f mat4);

// vectors, 4-tuples and 3-tuples must align to 16 bytes
//  2-vectors must align to 8 bytes
NV_ALIGN(16, typedef vec4f vec4);
NV_ALIGN(16, typedef vec3f vec3);
NV_ALIGN(8, typedef vec2f vec2);

NV_ALIGN(16, typedef vec4i ivec4);
NV_ALIGN(16, typedef vec3i ivec3);
NV_ALIGN(8, typedef vec2i ivec2);

NV_ALIGN(16, typedef vec4ui uvec4);
NV_ALIGN(16, typedef vec3ui uvec3);
NV_ALIGN(8, typedef vec2ui uvec2);
#else
// Matrices, must align to 4 vector (16 bytes)
typedef mat3f mat3;
typedef mat4f mat4;

// vectors, 4-tuples and 3-tuples must align to 16 bytes
//  2-vectors must align to 8 bytes
typedef vec4f vec4;
typedef vec3f vec3;
typedef vec2f vec2;

typedef vec4i ivec4;
typedef vec3i ivec3;
typedef vec2i ivec2;

typedef vec4ui uvec4;
typedef vec3ui uvec3;
typedef vec2ui uvec2;
#endif

// class to make uint look like bool to make GLSL packing rules happy
struct boolClass {
  unsigned int _rep;

  boolClass() : _rep(false) {}
  boolClass(bool b) : _rep(b) {}
  operator bool() { return _rep == 0 ? false : true; }
  boolClass& operator=(bool b) {
    _rep = b;
    return *this;
  }
};

NV_ALIGN(4, typedef boolClass bool32);

}  // namespace nvmath

#endif
