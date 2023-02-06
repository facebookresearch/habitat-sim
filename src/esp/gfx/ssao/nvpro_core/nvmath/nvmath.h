/*
 * Copyright (c) 2002-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 2002-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

/// \nodoc (keyword to exclude this file from automatic README.md generation)

#ifndef _nvmath_h_
#define _nvmath_h_

#include <type_traits>
#include "nvmath_types.h"

namespace nvmath {
// clang-format off
template<class T>  const vector2<T> operator+(const vector2<T>& u, const vector2<T>& v);
template<class T>  const vector2<T> operator+(const vector2<T>& u, const T s);
template<class T>  const vector2<T> operator-(const vector2<T>& u, const vector2<T>& v);
template<class T>  const vector2<T> operator-(const vector2<T>& u, const T s);
template<class T>  const vector2<T> operator*(const T s, const vector2<T>& u);
template<class T>  const vector2<T> operator*(const vector2<T>& u, const T s);
template<class T>  const vector2<T> operator/(const vector2<T>& u, const T s);
template<class T>  const vector2<T> operator*(const vector2<T>&u, const vector2<T>&v);
template<class T>  const vector2<T> operator/(const vector2<T>&u, const vector2<T>&v);

template<class T>  vector3<T> mult(const matrix3<T>& M, const vector3<T>& v);
template<class T>  vector3<T> mult(const matrix4<T>& M, const vector3<T>& v);

template<class T>  const vector3<T> operator+(const vector3<T>& u, const vector3<T>& v);
template<class T>  const vector3<T> operator-(const vector3<T>& u, const vector3<T>& v);
template<class T>  const vector3<T> operator+(const vector3<T>& u, const T v);
template<class T>  const vector3<T> operator-(const vector3<T>& u, const T v);
template<class T>  const vector3<T> operator^(const vector3<T>& u, const vector3<T>& v);
template<class T>  const vector3<T> operator*(const T s, const vector3<T>& u);
template<class T>  const vector3<T> operator*(const vector3<T>& u, const T s);
template<class T>  const vector3<T> operator/(const vector3<T>& u, const T s);
template<class T>  const vector3<T> operator*(const vector3<T>& u, const vector3<T>& v);
template<class T>  const vector3<T> operator/(const vector3<T>& u, const vector3<T>& v);

template<class T>  const vector4<T> operator+(const vector4<T>& u, const vector4<T>& v);
template<class T>  const vector4<T> operator-(const vector4<T>& u, const vector4<T>& v);
template<class T>  const vector4<T> operator+(const vector4<T>& u, const T s);
template<class T>  const vector4<T> operator-(const vector4<T>& u, const T s);
template<class T>  const vector4<T> operator*(const T s, const vector4<T>& u);
template<class T>  const vector4<T> operator*(const vector4<T>& u, const T s);
template<class T>  const vector4<T> operator/(const vector4<T>& u, const T s);

template<class T>  const vector4<T> operator*(const vector4<T>& u, const vector4<T>& v);
template<class T>  const vector4<T> operator/(const vector4<T>& u, const vector4<T>& v);
template<class T>  const vector3<T> operator*(const matrix3<T>&, const vector3<T>&);
template<class T>  const vector3<T> operator*(const vector3<T>&, const matrix3<T>&);

template<class T>  matrix4<T> mult(const matrix4<T>& A, const matrix4<T>& B);

template<class T> const vector4<T> operator*(const matrix4<T>&, const vector4<T>&);
template<class T> const vector4<T> operator*(const matrix4<T>&, const vector3<T>&);
template<class T> const vector4<T> operator*(const vector4<T>&, const matrix4<T>&);

template<class T>  matrix4<T> scale_mat4(const vector3<T> &t);
template<class T>  matrix4<T> translation_mat4(const vector3<T> &t);
template<class T>  matrix4<T> translation_mat4(T x, T y, T z);
template<class T>  matrix4<T> rotation_mat4_x(T a);
template<class T>  matrix4<T> rotation_mat4_y(T a);
template<class T>  matrix4<T> rotation_mat4_z(T a);


template<class T> const quaternion<T> operator*(const quaternion<T>&, const quaternion<T>&);
template<class T> const quaternion<T> mul(const quaternion<T>&, const quaternion<T>&);
template<class T>  quaternion<T> add_quats(const quaternion<T> & q1, const quaternion<T> & q2);
template<class T>  T dot(const quaternion<T> & p, const quaternion<T> & q);
template<class T>  quaternion<T> slerp_quats(T s, const quaternion<T> & q1, const quaternion<T> & q2);
template<class T>  quaternion<T> axis_to_quat(const vector3<T> & a, const T phi);
template<class T>  quaternion<T> mat_2_quat(const matrix3<T> &M);
template<class T>  quaternion<T> normalize(const quaternion<T> & p);
template<class T>  quaternion<T> conj(const quaternion<T> & q);
template<class T>  matrix3<T> quat_2_mat(const quaternion<T> &q );
template<class T>  quaternion<T> mat_2_quat(const matrix4<T> &M);

// constant algebraic values

static const nv_scalar array16_id[] =        { nv_one, nv_zero, nv_zero, nv_zero,
                                        nv_zero, nv_one, nv_zero, nv_zero,
                                        nv_zero, nv_zero, nv_one, nv_zero,
                                        nv_zero, nv_zero, nv_zero, nv_one};

static const nv_scalar array16_zero[] =      { nv_zero, nv_zero, nv_zero, nv_zero,
                                        nv_zero, nv_zero, nv_zero, nv_zero,
                                        nv_zero, nv_zero, nv_zero, nv_zero,
                                        nv_zero, nv_zero, nv_zero, nv_zero};

static const nv_scalar array16_scale_bias[] = { nv_one_half, nv_zero,   nv_zero,   nv_zero,
                                         nv_zero,   nv_one_half, nv_zero,   nv_zero,
                                         nv_zero,   nv_zero,   nv_one_half, nv_zero,
                                         nv_one_half, nv_one_half, nv_one_half, nv_one};

static const nv_scalar array9_id[] =         { nv_one, nv_zero, nv_zero,
                                        nv_zero, nv_one, nv_zero,
                                        nv_zero, nv_zero, nv_one};

static const vector2<nv_scalar>      vec2f_zero(nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_one(nv_one,nv_one,nv_one,nv_one);
static const vector3<nv_scalar>      vec3f_one(nv_one,nv_one,nv_one);
static const vector3<nv_scalar>      vec3f_zero(nv_zero,nv_zero,nv_zero);
static const vector3<nv_scalar>      vec3f_x(nv_one,nv_zero,nv_zero);
static const vector3<nv_scalar>      vec3f_y(nv_zero,nv_one,nv_zero);
static const vector3<nv_scalar>      vec3f_z(nv_zero,nv_zero,nv_one);
static const vector3<nv_scalar>      vec3f_neg_x(-nv_one,nv_zero,nv_zero);
static const vector3<nv_scalar>      vec3f_neg_y(nv_zero,-nv_one,nv_zero);
static const vector3<nv_scalar>      vec3f_neg_z(nv_zero,nv_zero,-nv_one);
static const vector4<nv_scalar>      vec4f_zero(nv_zero,nv_zero,nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_x(nv_one,nv_zero,nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_neg_x(-nv_one,nv_zero,nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_y(nv_zero,nv_one,nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_neg_y(nv_zero,-nv_one,nv_zero,nv_zero);
static const vector4<nv_scalar>      vec4f_z(nv_zero,nv_zero,nv_one,nv_zero);
static const vector4<nv_scalar>      vec4f_neg_z(nv_zero,nv_zero,-nv_one,nv_zero);
static const vector4<nv_scalar>      vec4f_w(nv_zero,nv_zero,nv_zero,nv_one);
static const vector4<nv_scalar>      vec4f_neg_w(nv_zero,nv_zero,nv_zero,-nv_one);
static const quaternion<nv_scalar>   quat_id(nv_zero,nv_zero,nv_zero,nv_one);
static const matrix4<nv_scalar>      mat4f_id(array16_id);
static const matrix3<nv_scalar>      mat3f_id(array9_id);
static const matrix4<nv_scalar>      mat4f_zero(array16_zero);
static const matrix4<nv_scalar>      mat4f_scale_bias(array16_scale_bias);

// normalizes a vector
template<class T>  vector2<T> normalize(const vector2<T> & u);
template<class T>  vector3<T> normalize(const vector3<T> & u);
template<class T>  vector4<T> normalize(const vector4<T> & u);

// Computes the squared magnitude
template<class T>  T nv_sq_norm(const vector2<T> & n);
template<class T>  T nv_sq_norm(const vector3<T> & n);
template<class T>  T nv_sq_norm(const vector4<T> & n);

// Computes the magnitude
template<class T>  T nv_norm(const vector2<T> & n);
template<class T>  T nv_norm(const vector3<T> & n);
template<class T>  T nv_norm(const vector4<T> & n);
template<class T>  T length(const vector2<T> & n);
template<class T>  T length(const vector3<T> & n);
template<class T>  T length(const vector4<T> & n);

// computes the cross product ( v cross w) and stores the result in u
// i.e.     u = v cross w
template<class T>  vector3<T> cross(const vector3<T> & v, const vector3<T> & w);

// computes the dot product ( v dot w) and stores the result in u
// i.e.     u = v dot w
template<class T>  T dot(const vector3<T> & v, const vector3<T> & w);
template<class T>  T dot(const vector4<T> & v, const vector4<T> & w);
template<class T>  T dot(const vector3<T> & v, const vector4<T> & w);
template<class T>  T dot(const vector4<T> & v, const vector3<T> & w);
template<class T>  T dot(const vector2<T> & v, const vector2<T> & w);

// clamp
template<class T> T clamp(T x, T minVal, T maxVal);
template<class T> vector3<T> clamp(const vector3<T>& x, const vector3<T>& minVal, const vector3<T>& maxVal);

// min / max
template<class T> T min(T minVal, T maxVal);
template<class T> T max(T minVal, T maxVal);
template<class T> vector3<T> max(const vector3<T>& minVal, const vector3<T>& maxVal);

// mix
template<class T> T mix(T x, T y, T a);
template<class T> vector3<T> mix(const vector3<T>& x, const vector3<T>& y, T a);
template<class T> vector3<T> mix(const vector3<T>& x, const vector3<T>& y, const vector3<T>& a);

// Pow
template<class T> vector3<T> pow(const vector3<T>& base, const vector3<T>& exponent);

// sqrt
template<class T> vector3<T> sqrt(const vector3<T>& x);

// radians
template<class T> T radians(T x);

// sin
template<class T> vector3<T> sin(const vector3<T>& x);

// mod
template<class T> T mod(T a, T b);
template<class T> vector2<T> mod(const vector2<T>& a, T b);

// fract
template<class T> T fract(T x);



// compute the reflected vector R of L w.r.t N - vectors need to be
// normalized
//
//                R     N     L
//                  _       _
//                 |\   ^   /|
//                   \  |  /
//                    \ | /
//                     \|/
//                      +
template<class T>  vector3<T> reflect(const vector3<T> & n, const vector3<T> & l);

// Computes u = v * lambda + u
template<class T>  vector3<T> madd(const vector3<T> & v, const T & lambda);
// Computes u = v * lambda
template<class T>  vector3<T> mult(const vector3<T> & v, const T & lambda);
// Computes u = v * w
template<class T>  vector3<T> mult(const vector3<T> & v, const vector3<T> & w);
// Computes u = v + w
template<class T>  vector3<T> add(const vector3<T> & v, const vector3<T> & w);
// Computes u = v - w
template<class T>  vector3<T> sub(const vector3<T> & v, const vector3<T> & w);

// Computes u = pow(v, exponemt)
template <class T> vector3<T> pow(const vector3<T>& v, const T & e);

// Computes u = u * s
template<class T>  vector2<T> scale(const vector2<T> & u, const T s);
template<class T>  vector3<T> scale(const vector3<T> & u, const T s);
template<class T>  vector4<T> scale(const vector4<T> & u, const T s);

// Computes u = M * v
template<class T>  vector3<T> mult(const matrix3<T> & M, const vector3<T> & v);
template<class T>  vector4<T> mult(const matrix4<T> & M, const vector4<T> & v);

// Computes u = v * M
template<class T>  vector3<T> mult(const vector3<T> & v, const matrix3<T> & M);
template<class T>  vector4<T> mult(const vector4<T> & v, const matrix4<T> & M);

// Computes u = M(4x4) * v and divides by w
template<class T>  vector3<T> mult_pos(const matrix4<T> & M, const vector3<T> & v);
// Computes u = M(4x4) * v
template<class T>  vector3<T> mult_dir(const matrix4<T> & M, const vector3<T> & v);
// Computes u = M(4x4) * v and does not divide by w (assumed to be 1)
template<class T>  vector3<T> mult(const matrix4<T>& M, const vector3<T>& v);

// Computes u = v * M(4x4) and divides by w
template<class T>  vector3<T> mult_pos(const vector3<T> & v, const matrix4<T> & M);
// Computes u = v * M(4x4)
template<class T>  vector3<T> mult_dir(const vector3<T> & v, const matrix4<T> & M);
// Computes u = v * M(4x4) and does not divide by w (assumed to be 1)
template<class T>  vector3<T> mult(const vector3<T>& v, const matrix4<T>& M);

// Computes C = A + B
template<class T>  matrix4<T> add(const matrix4<T> & A, const matrix4<T> & B);
template<class T>  matrix3<T> add(const matrix3<T> & A, const matrix3<T> & B);

// Computes C = A * B
template<class T>  matrix4<T> mult(const matrix4<T> & A, const matrix4<T> & B);
template<class T>  matrix3<T> mult(const matrix3<T> & A, const matrix3<T> & B);

// Compute M = -M
template<class T>  matrix4<T> negate(const matrix4<T> & M);
template<class T>  matrix3<T> negate(const matrix3<T> & M);

// Computes B = Transpose(A)
//       T
//  B = A
template<class T>  matrix3<T> transpose(const matrix3<T> & A);
template<class T>  matrix4<T> transpose(const matrix4<T> & A);

// Computes B = inverse(A)
//       -1
//  B = A
template<class T>  matrix4<T> invert(const matrix4<T> & A, bool& valid);
template<class T>  matrix4<T> invert(const matrix4<T> & A);
template<class T>  matrix3<T> invert(const matrix3<T> & A);
template<class T>  matrix4<T> inverse(const matrix4<T>& A);
template<class T>  matrix3<T> inverse(const matrix3<T>& A);

// Computes B = inverse(A)
//                                       T  T
//                   (R t)             (R -R t)
// assuming that A = (0 1) so that B = (0    1)
//  B = A
template<class T>  matrix4<T> invert_rot_trans(const matrix4<T> & A);

template<class T>  matrix4<T> look_at(const vector3<T> & eye, const vector3<T> & center, const vector3<T> & up);

template<class T>  matrix4<T> frustum(const T l, const T r, const T b, const T t, const T n, const T f);
template<class T>  matrix4<T> frustum01(const T l, const T r, const T b, const T t, const T n, const T f);
template<class T>  matrix4<T> frustum01Rev(const T l, const T r, const T b, const T t, const T n, const T f);

template<class T>  matrix4<T> perspective(const T fovy, const T aspect, const T n, const T f);
template<class T>  matrix4<T> perspective01(const T fovy, const T aspect, const T n, const T f);
template<class T>  matrix4<T> perspective01Rev(const T fovy, const T aspect, const T n, const T f);

template<class T>  matrix4<T> ortho(const T left,
                              const T right,
                              const T bottom,
                              const T top,
                              const T n,
                              const T f);

/* Decompose Affine Matrix
 *    A = TQS, where
 * A is the affine transform
 * T is the translation vector
 * Q is the rotation (quaternion)
 * S is the scale vector
 * f is the sign of the determinant
*/
template<class T>  void decomp_affine(const matrix4<T> & A, vector3<T> & v3, quaternion<T> & Q, quaternion<T> & U, vector3<T> & S, T & f);


// surface properties
template<class T>  matrix3<T> & tangent_basis(const vector3<T> & v0,const vector3<T> & v1,const vector3<T> & v2,const vector2<T> & t0,const vector2<T> & t1,const vector2<T> & t2, const vector3<T> & n);

// linear interpolation
#ifdef NVP_SUPPORTS_OPTIX
#pragma message("**WARNING** nvmath.h : Canceling the lerp() function here : already declared in OptiX")
#else
template<class T>  T lerp(const T t, const T a, const T b);
template<class T>  vector2<T> lerp(const T & t, const vector2<T> & u, const vector2<T> & v);
template<class T>  vector3<T> lerp(const T & t, const vector3<T> & u, const vector3<T> & v);
template<class T>  vector4<T> lerp(const T & t, const vector4<T> & u, const vector4<T> & v);
#endif

// utilities
template<class T>  T nv_random();

template<class T>  quaternion<T> trackball(vector2<T> & pt1, vector2<T> & pt2, T trackballsize);

template<class T>  vector3<T> cube_map_normal(int i, int x, int y, int cubesize, const vector3<T> & v);

// Componentwise

template<class T>  T nv_min(const T & lambda, const T & n);
template<class T>  vector2<T> nv_min(const vector2<T> & vFirst, const vector2<T> & vSecond);
template<class T>  vector3<T> nv_min(const vector3<T> & vFirst, const vector3<T> & vSecond);
template<class T>  vector4<T> nv_min(const vector4<T> & vFirst, const vector4<T> & vSecond);

template<class T>  T nv_max(const T & lambda, const T & n);
template<class T>  vector3<T> nv_max(const vector3<T> & vFirst, const vector3<T> & vSecond);
template<class T>  vector2<T> nv_max(const vector2<T> & vFirst, const vector2<T> & vSecond);
template<class T>  vector4<T> nv_max(const vector4<T> & vFirst, const vector4<T> & vSecond);

template<class T>  T nv_clamp(const T u, const T min, const T max);
template<class T>  vector2<T> nv_clamp(const vector2<T>& u, const T min, const T max);
template<class T>  vector3<T> nv_clamp(const vector3<T>& u, const T min, const T max);
template<class T>  vector4<T> nv_clamp(const vector4<T>& u, const T min, const T max);

template<class T>  vector2<T> nv_floor(const vector2<T>& u);
template<class T>  vector3<T> nv_floor(const vector3<T>& u);
template<class T>  vector4<T> nv_floor(const vector4<T>& u);


template<class T> T          nv_abs(const T u);
template<class T> vector2<T> nv_abs(const vector2<T> & u);
template<class T> vector3<T> nv_abs(const vector3<T> & u);
template<class T> vector4<T> nv_abs(const vector4<T> & u);

template<class T> T smoothstep(T edge0, T edge1, T x);



// geometry
// computes the area of a triangle
template<class T>  T nv_area(const vector3<T> & v1, const vector3<T> & v2, const vector3<T> &v3);
// computes the perimeter of a triangle
template<class T>  T nv_perimeter(const vector3<T> & v1, const vector3<T> & v2, const vector3<T> &v3);
// find the inscribed circle
template<class T>  T nv_find_in_circle( vector3<T> & center, const vector3<T> & v1, const vector3<T> & v2, const vector3<T> &v3);
// find the circumscribed circle
template<class T>  T nv_find_circ_circle( vector3<T> & center, const vector3<T> & v1, const vector3<T> & v2, const vector3<T> &v3);

// fast cosine functions
template<class T>  T fast_cos(const T x);
template<class T>  T ffast_cos(const T x);

// determinant
template<class T> T det(const matrix3<T> & A);
template<class T> T det(const matrix4<T> & A);

template<class T>  void nv_is_valid(const vector3<T>& v);
template<class T>  void nv_is_valid(T lambda);

// TL : v1 and v2 MUST be normalized. Not done inot this to avoid redundant work...
template<class T>  T get_angle(const vector3<T> & v1, const vector3<T> & v2);
template<class T>  vector3<T> rotate_by(const vector3<T> & src, const quaternion<T>& q);

template<class T>
 matrix4<T> rotation_yaw_pitch_roll( const T yaw , const T pitch , const T roll );

// clang-format on
}  // namespace nvmath

#include "nvmath.inl"

#endif  //_nvmath_h_
