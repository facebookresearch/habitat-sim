/*
 * Copyright (c) 2018-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2018-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */
//--------------------------------------------------------------------
/// \nodoc
#ifndef NV_FOUNDATION_H
#define NV_FOUNDATION_H

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#ifdef _MSC_VER
#ifndef _INTPTR
#define _INTPTR 0
#endif
#endif
#include <stdint.h>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4985)  // 'symbol name': attributes not present on
                                 // previous declaration
#endif
#include <math.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <assert.h>
#include <float.h>

#define NV_ASSERT(exp) (assert(exp))
#define NV_ALWAYS_ASSERT() NV_ASSERT(0)

//***************************************
// FILE: NvVersionNumber.h
//***************************************

/*
VersionNumbers:  The combination of these
numbers uniquely identifies the API, and should
be incremented when the SDK API changes.  This may
include changes to file formats.

This header is included in the main SDK header files
so that the entire SDK and everything that builds on it
is completely rebuilt when this file changes.  Thus,
this file is not to include a frequently changing
build number.  See BuildNumber.h for that.

Each of these three values should stay below 255 because
sometimes they are stored in a byte.
*/

/** \addtogroup foundation
  @{
*/

#define NV_FOUNDATION_VERSION_MAJOR 1
#define NV_FOUNDATION_VERSION_MINOR 1
#define NV_FOUNDATION_VERSION_BUGFIX 0

/**
The constant NV_FOUNDATION_VERSION is used to confirm the version of the
foundation headers. This is to ensure that the application is using the same
header version as the library was built with.
*/
#define NV_FOUNDATION_VERSION                                                  \
  ((NV_FOUNDATION_VERSION_MAJOR << 24) + (NV_FOUNDATION_VERSION_MINOR << 16) + \
   (NV_FOUNDATION_VERSION_BUGFIX << 8) + 0)

//***************************************
// FILE: NvPreprocessor.h
//***************************************

/**
List of preprocessor defines used to configure the SDK
- NV_DEBUG: enable asserts (exactly one needs to be defined)
- NV_CHECKED: enable run time checks, mostly unused or equiv. to NV_DEBUG
- NV_SUPPORT_VISUAL_DEBUGGER: ...
- AG_PERFMON: ... (Deprecated)
*/

/**
Compiler define
*/
#ifdef _MSC_VER
#define NV_VC
#if _MSC_VER >= 1700
#define NV_VC11
#elif _MSC_VER >= 1600
#define NV_VC10
#elif _MSC_VER >= 1500
#define NV_VC9
#elif _MSC_VER >= 1400
#define NV_VC8
#elif _MSC_VER >= 1300
#define NV_VC7
#else
#define NV_VC6
#endif
#elif defined(__ghs__)
#define NV_GHS
#elif __GNUC__ || __SNC__
#define NV_GNUC
#else
#error "Unknown compiler"
#endif

/**
Platform define
*/
#ifdef NV_VC
#ifdef XBOXONE
#define NV_XBOXONE
#define NV_X64
#elif defined(_M_IX86)
#define NV_X86
#define NV_WINDOWS
#elif defined(_M_X64)
#define NV_X64
#define NV_WINDOWS
#elif defined(_M_PPC)
#define NV_PPC
#define NV_X360
#define NV_VMX
#elif defined(_M_ARM)
#define NV_ARM
#define NV_ARM_NEON
#else
#error "Unknown platform"
#endif
#if defined(WINAPI_FAMILY) && WINAPI_FAMILY == WINAPI_PARTITION_APP
#define NV_WINMODERN
#endif
#elif defined NV_GNUC
#ifdef __CELLOS_LV2__
#define NV_PS3
#define NV_VMX
#elif defined(__arm__) || defined(__aarch64__)
#define NV_ARM
#if defined(__SNC__)
#define NV_PSP2
#endif
#if defined(__ARM_NEON__)
#define NV_ARM_NEON
#endif
#elif defined(__i386__)
#define NV_X86
#define NV_VMX
#elif defined(__x86_64__)
#ifdef __PS4__
#define NV_PS4
#define NV_X64
#else
#define NV_X64
#endif
#elif defined(__ppc__)
#define NV_PPC
#elif defined(__ppc64__)
#define NV_PPC
#define NV_PPC64
#else
#error "Unknown platform"
#endif
#if defined(ANDROID)
#define NV_ANDROID
#define NV_UNIX
#elif defined(__linux__)
#define NV_LINUX
#define NV_UNIX
#elif defined(__APPLE__)
#define NV_APPLE
#define NV_UNIX
#if defined(__arm__)
#define NV_APPLE_IOS
#else
#define NV_OSX
#endif
#elif defined(__CYGWIN__)
#define NV_CYGWIN
#define NV_LINUX
#define NV_UNIX
#endif
#elif defined NV_GHS
#define NV_WIIU
#endif

/**
DLL export macros
*/
#if !defined(NV_C_EXPORT)
#if defined(NV_WINDOWS) || defined(NV_WINMODERN)
#define NV_C_EXPORT extern "C"
#else
#define NV_C_EXPORT
#endif
#endif

/**
Calling convention
*/
#ifndef NV_CALL_CONV
#if defined NV_WINDOWS
#define NV_CALL_CONV __cdecl
#else
#define NV_CALL_CONV
#endif
#endif

/**
Pack macros - disabled on SPU because they are not supported
*/
#if defined(NV_VC)
#define NV_PUSH_PACK_DEFAULT __pragma(pack(push, 8))
#define NV_POP_PACK __pragma(pack(pop))
#elif (defined(NV_GNUC) && !defined(__SPU__)) || defined(NV_GHS)
#define NV_PUSH_PACK_DEFAULT _Pragma("pack(push, 8)")
#define NV_POP_PACK _Pragma("pack(pop)")
#else
#define NV_PUSH_PACK_DEFAULT
#define NV_POP_PACK
#endif

/**
Inline macro
*/
#if defined(NV_WINDOWS) || defined(NV_X360) || defined(NV_WINMODERN) || \
    defined(NV_XBOXONE)
#define NV_INLINE inline
#pragma inline_depth(255)
#else
#define NV_INLINE inline
#endif

/**
Force inline macro
*/
#if defined(NV_VC)
#define NV_FORCE_INLINE __forceinline
#elif defined(NV_LINUX)  // Workaround; Fedora Core 3 do not agree with force
                         // inline and NvcPool
#define NV_FORCE_INLINE inline
#elif defined(NV_GNUC) || defined(NV_GHS)
#define NV_FORCE_INLINE inline __attribute__((always_inline))
#else
#define NV_FORCE_INLINE inline
#endif

/**
Noinline macro
*/
#if defined NV_WINDOWS || defined NV_XBOXONE
#define NV_NOINLINE __declspec(noinline)
#elif defined(NV_GNUC) || defined(NV_GHS)
#define NV_NOINLINE __attribute__((noinline))
#else
#define NV_NOINLINE
#endif

/*! restrict macro */
#if defined(__CUDACC__)
#define NV_RESTRICT __restrict__
#elif (defined(NV_GNUC) || defined(NV_VC) || defined(NV_GHS)) && \
    !defined(NV_PS4)  // ps4 doesn't like restricted functions
#define NV_RESTRICT __restrict
#else
#define NV_RESTRICT
#endif

#if defined(NV_WINDOWS) || defined(NV_X360) || defined(NV_WINMODERN) || \
    defined(NV_XBOXONE)
#define NV_NOALIAS __declspec(noalias)
#else
#define NV_NOALIAS
#endif

/**
Alignment macros

NV_ALIGN_PREFIX and NV_ALIGN_SUFFIX can be used for type alignment instead of
aligning individual variables as follows: NV_ALIGN_PREFIX(16) struct A {
...
} NV_ALIGN_SUFFIX(16);
This declaration style is parsed correctly by Visual Assist.

*/
#ifndef NV_ALIGN
#if defined(NV_VC)
#define NV_ALIGN(alignment, decl) __declspec(align(alignment)) decl
#define NV_ALIGN_PREFIX(alignment) __declspec(align(alignment))
#define NV_ALIGN_SUFFIX(alignment)
#elif defined(NV_GNUC) || defined(NV_GHS) || defined(NV_APPLE_IOS)
#define NV_ALIGN(alignment, decl) decl __attribute__((aligned(alignment)))
#define NV_ALIGN_PREFIX(alignment)
#define NV_ALIGN_SUFFIX(alignment) __attribute__((aligned(alignment)))
#else
#define NV_ALIGN(alignment, decl)
#define NV_ALIGN_PREFIX(alignment)
#define NV_ALIGN_SUFFIX(alignment)
#endif
#endif

/**
Deprecated macro
- To deprecate a function: Place NV_DEPRECATED at the start of the function
header (leftmost word).
- To deprecate a 'typdef', a 'struct' or a 'class': Place NV_DEPRECATED directly
after the keywords ('typdef', 'struct', 'class').
*/
#if 0  // set to 1 to create warnings for deprecated functions
#define NV_DEPRECATED __declspec(deprecated)
#else
#define NV_DEPRECATED
#endif

// VC6 no '__FUNCTION__' workaround
#if defined NV_VC6 && !defined __FUNCTION__
#define __FUNCTION__ "Undefined"
#endif

/**
General defines
*/

// static assert
#define NV_COMPILE_TIME_ASSERT(exp) \
  typedef char NvCompileTimeAssert_Dummy[(exp) ? 1 : -1]

#if defined(NV_GNUC)
#define NV_OFFSET_OF(X, Y) __builtin_offsetof(X, Y)
#else
#define NV_OFFSET_OF(X, Y) offsetof(X, Y)
#endif

#define NV_SIZE_OF(Class, Member) sizeof(((Class*)0)->Member)

#define NV_ARRAY_SIZE(X) (sizeof((X)) / sizeof((X)[0]))

#define NV_PAD_POW2(value, pad) (((value) + ((pad)-1)) & (~((pad)-1)))

// _DEBUG is provided only by MSVC, but not GCC.
// NDEBUG is the canonical platform agnostic way to detect debug/release builds
#if !defined(_DEBUG) && !defined(NDEBUG)
#define _DEBUG 1
#endif

// check that exactly one of NDEBUG and _DEBUG is defined
#if !(defined NDEBUG ^ defined _DEBUG)
#error "NDEBUG and _DEBUG are mutually exclusive"
#endif

// make sure NV_CHECKED is defined in all _DEBUG configurations as well
#if !defined(NV_CHECKED) && defined _DEBUG
#define NV_CHECKED
#endif

#ifdef __CUDACC__
#define NV_CUDA_CALLABLE __host__ __device__
#else
#define NV_CUDA_CALLABLE
#endif

// avoid unreferenced parameter warning (why not just disable it?)
// PT: or why not just omit the parameter's name from the declaration????
#if defined(__cplusplus__)
template <class T>
NV_CUDA_CALLABLE NV_INLINE void NV_UNUSED(T const&) {}
#else
#define NV_UNUSED(var) (void)(var)
#endif

// Ensure that the application hasn't tweaked the pack value to less than 8,
// which would break matching between the API headers and the binaries This
// assert works on win32/win64/360/ps3, but may need further specialization on
// other platforms. Some GCC compilers need the compiler flag -malign-double to
// be set. Apparently the apple-clang-llvm compiler doesn't support
// malign-double.

typedef struct NvPackValidation {
  char _;
  long long a;
} NvPackValidation;

#if !defined(NV_APPLE)
NV_COMPILE_TIME_ASSERT(NV_OFFSET_OF(NvPackValidation, a) == 8);
#endif

// use in a cpp file to suppress LNK4221
#if defined(NV_VC)
#define NV_DUMMY_SYMBOL \
  namespace {           \
  char NvDummySymbol;   \
  }
#else
#define NV_DUMMY_SYMBOL
#endif

#ifdef __SPU__
#define NV_IS_SPU 1
#else
#define NV_IS_SPU 0
#endif

#ifdef NV_X64
#define NV_IS_X64 1
#else
#define NV_IS_X64 0
#endif

#ifdef NV_WINDOWS
#define NV_IS_WINDOWS 1
#else
#define NV_IS_WINDOWS 0
#endif

#ifdef NV_X86
#define NV_IS_X86 1
#else
#define NV_IS_X86 0
#endif

#ifdef NV_X64
#define NV_IS_X64 1
#else
#define NV_IS_X64 0
#endif

#if defined(NV_X86) || defined(NV_X64)
#define NV_IS_INTEL 1
#else
#define NV_IS_INTEL 0
#endif

#ifdef NV_X360
#define NV_IS_X360 1
#else
#define NV_IS_X360 0
#endif

#ifdef NV_PS3
#define NV_IS_PS3 1
#else
#define NV_IS_PS3 0
#endif

#define NV_IS_PPU (NV_IS_PS3 && !NV_IS_SPU)  // PS3 PPU

#ifdef NV_GNUC
#define NV_WEAK_SYMBOL \
  __attribute__((weak))  // this is to support SIMD constant merging in template
                         // specialization
#else
#define NV_WEAK_SYMBOL
#endif

// Type ranges
#define NV_MAX_I8 127          // maximum possible sbyte value, 0x7f
#define NV_MIN_I8 (-128)       // minimum possible sbyte value, 0x80
#define NV_MAX_U8 255U         // maximum possible ubyte value, 0xff
#define NV_MIN_U8 0            // minimum possible ubyte value, 0x00
#define NV_MAX_I16 32767       // maximum possible sword value, 0x7fff
#define NV_MIN_I16 (-32768)    // minimum possible sword value, 0x8000
#define NV_MAX_U16 65535U      // maximum possible uword value, 0xffff
#define NV_MIN_U16 0           // minimum possible uword value, 0x0000
#define NV_MAX_I32 2147483647  // maximum possible sdword value, 0x7fffffff
#define NV_MIN_I32 \
  (-2147483647 - 1)             // minimum possible sdword value, 0x80000000
#define NV_MAX_U32 4294967295U  // maximum possible udword value, 0xffffffff
#define NV_MIN_U32 0            // minimum possible udword value, 0x00000000
#define NV_MAX_F32 3.4028234663852885981170418348452e+38F
// maximum possible float value
#define NV_MAX_F64 DBL_MAX  // maximum possible double value

#define NV_EPS_F32 FLT_EPSILON  // maximum relative error of float rounding
#define NV_EPS_F64 DBL_EPSILON  // maximum relative error of double rounding

#define NV_MAX_REAL NV_MAX_F32
#define NV_EPS_REAL NV_EPS_F32
#define NV_NORMALIZATION_EPSILON float(1e-20f)

/** enum for empty constructor tag*/
enum NvEMPTY { NvEmpty };

/**  Basic struct data type for float2 Vectors */
typedef struct {
  float x, y;
} NV_float2;

/**  Basic struct data type for float3 Vectors */
typedef struct {
  float x, y, z;
} NV_float3;

/**  Basic struct data type for float4 Vector or quaternion */
typedef struct {
  float x, y, z, w;
} NV_float4;

/**  Basic struct data type for 7 floats, typically used to represent a 'pose'
 * comprised of a quaternion rotation (x,y,z,w) followed by a position (x,y,z)
 */
typedef struct {
  NV_float4 q;
  NV_float3 p;
} NV_float7;

/**  Basic struct data type for 9 floats, typically a 3x3 matrix */
typedef struct {
  float transform[9];
} NV_float9;

/**  Basic struct data type for 12 floats, typically a 3x4 matrix */
typedef struct {
  float transform[12];
} NV_float12;

/**  Basic struct data type for 16 floats, typically a 4x4 matrix */
typedef struct {
  float transform[16];
} NV_float16;

/** Basic struct data to for a 3d bounding box */
typedef struct {
  float minimum[3];
  float maximum[3];
} NV_bounds3;

/** @} */

#endif  // NV_FOUNDATION_H
