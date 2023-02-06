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
/// \nodoc

#include "NvFoundation.h"

#ifndef NVP_PLATFORM_H__
#define NVP_PLATFORM_H__

#if defined(__GNUC__) && \
    (__GNUC__ > 3 || (__GNUC__ >= 3 && __GNUC_MINOR__ >= 4))

#define NV_NOOP(...)
#define NV_BARRIER() __sync_synchronize()

/*
// maybe better than __sync_synchronize?
#if defined(__i386__ ) || defined(__x64__)
#define NVP_BARRIER()  __asm__ __volatile__ ("mfence" ::: "memory")
#endif

#if defined(__arm__)
#define NVP_BARRIER() __asm__ __volatile__ ("dmb" :::"memory")
#endif
*/

#elif defined(__MSC__) || defined(_MSC_VER)

#include <emmintrin.h>

#pragma warning(disable : 4142)  // redefinition of same type
#if (_MSC_VER >= 1400)           // VC8+
#pragma warning(disable : 4996)  // Either disable all deprecation warnings,
#endif                           // VC8+

#define NV_NOOP __noop
#define NV_BARRIER() _mm_mfence()

#else
#error "compiler unkown"
#endif

#endif
