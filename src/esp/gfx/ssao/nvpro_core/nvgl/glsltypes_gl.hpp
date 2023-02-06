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

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#ifndef NV_GLSLTYPES_INCLUDED
#define NV_GLSLTYPES_INCLUDED

/**
  # type definitions for nvgl and nvmath
  Sets up vector, matrix etc. types available in GLSL
*/

#include <stdint.h>
#include "../nvmath/nvmath_glsltypes.h>

namespace nvgl {
typedef uint64_t sampler1D;
typedef uint64_t sampler2D;
typedef uint64_t sampler2DMS;
typedef uint64_t sampler3D;
typedef uint64_t samplerBuffer;
typedef uint64_t samplerCube;
typedef uint64_t sampler1DArray;
typedef uint64_t sampler2DArray;
typedef uint64_t sampler2DMSArray;
typedef uint64_t samplerCubeArray;

typedef uint64_t usampler1D;
typedef uint64_t usampler2D;
typedef uint64_t usampler2DMS;
typedef uint64_t usampler3D;
typedef uint64_t usamplerBuffer;
typedef uint64_t usamplerCube;
typedef uint64_t usampler1DArray;
typedef uint64_t usampler2DArray;
typedef uint64_t usampler2DMSArray;
typedef uint64_t usamplerCubeArray;

typedef uint64_t isampler1D;
typedef uint64_t isampler2D;
typedef uint64_t isampler2DMS;
typedef uint64_t isampler3D;
typedef uint64_t isamplerBuffer;
typedef uint64_t isamplerCube;
typedef uint64_t isampler1DArray;
typedef uint64_t isampler2DArray;
typedef uint64_t isampler2DMSArray;
typedef uint64_t isamplerCubeArray;

typedef uint64_t image1D;
typedef uint64_t image2D;
typedef uint64_t image2DMS;
typedef uint64_t image3D;
typedef uint64_t imageBuffer;
typedef uint64_t imageCube;
typedef uint64_t image1DArray;
typedef uint64_t image2DArray;
typedef uint64_t image2DMSArray;
typedef uint64_t imageCubeArray;

typedef uint64_t uimage1D;
typedef uint64_t uimage2D;
typedef uint64_t uimage2DMS;
typedef uint64_t uimage3D;
typedef uint64_t uimageBuffer;
typedef uint64_t uimageCube;
typedef uint64_t uimage1DArray;
typedef uint64_t uimage2DArray;
typedef uint64_t uimage2DMSArray;
typedef uint64_t uimageCubeArray;

typedef uint64_t iimage1D;
typedef uint64_t iimage2D;
typedef uint64_t iimage2DMS;
typedef uint64_t iimage3D;
typedef uint64_t iimageBuffer;
typedef uint64_t iimageCube;
typedef uint64_t iimage1DArray;
typedef uint64_t iimage2DArray;
typedef uint64_t iimage2DMSArray;
typedef uint64_t iimageCubeArray;
}  // namespace nvgl

#endif
