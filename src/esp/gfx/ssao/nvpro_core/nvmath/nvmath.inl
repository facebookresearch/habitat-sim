/*
 * Copyright (c) 1999-2021, NVIDIA CORPORATION.  All rights reserved.
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
 * SPDX-FileCopyrightText: Copyright (c) 1999-2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _WIN32
#define _isnan isnan
#define _finite finite
#endif

#include <cmath>  // std::floor

namespace nvmath {

template <class T>
inline const vector2<T> operator+(const vector2<T>& u, const vector2<T>& v) {
  return vector2<T>(u.x + v.x, u.y + v.y);
}

template <class T>
inline const vector2<T> operator+(const vector2<T>& u, const T s) {
  return vector2<T>(u.x + s, u.y + s);
}

template <class T>
inline const vector2<T> operator-(const vector2<T>& u, const vector2<T>& v) {
  return vector2<T>(u.x - v.x, u.y - v.y);
}

template <class T>
inline const vector2<T> operator-(const vector2<T>& u, const T s) {
  return vector2<T>(u.x - s, u.y - s);
}

template <class T>
inline const vector2<T> operator*(const T s, const vector2<T>& u) {
  return vector2<T>(s * u.x, s * u.y);
}

template <class T>
inline const vector2<T> operator*(const vector2<T>& u, const T s) {
  return vector2<T>(s * u.x, s * u.y);
}

template <class T>
inline const vector2<T> operator/(const vector2<T>& u, const T s) {
  return vector2<T>(u.x / s, u.y / s);
}

template <class T>
inline const vector2<T> operator/(const vector2<T>& u, const vector2<T>& v) {
  return vector2<T>(u.x / v.x, u.y / v.y);
}

template <class T>
inline const vector2<T> operator*(const vector2<T>& u, const vector2<T>& v) {
  return vector2<T>(u.x * v.x, u.y * v.y);
}

template <class T>
inline const vector3<T> operator+(const vector3<T>& u, const vector3<T>& v) {
  return vector3<T>(u.x + v.x, u.y + v.y, u.z + v.z);
}

template <class T>
inline const vector3<T> operator-(const vector3<T>& u, const vector3<T>& v) {
  return vector3<T>(u.x - v.x, u.y - v.y, u.z - v.z);
}

template <class T>
inline const vector3<T> operator+(const vector3<T>& u, const T v) {
  return vector3<T>(u.x + v, u.y + v, u.z + v);
}

template <class T>
inline const vector3<T> operator-(const vector3<T>& u, const T v) {
  return vector3<T>(u.x - v, u.y - v, u.z - v);
}

template <class T>
inline const vector3<T> operator^(const vector3<T>& u, const vector3<T>& v) {
  return vector3<T>(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z,
                    u.x * v.y - u.y * v.x);
}

template <class T>
inline const vector3<T> operator*(const T s, const vector3<T>& u) {
  return vector3<T>(s * u.x, s * u.y, s * u.z);
}

template <class T>
inline const vector3<T> operator*(const vector3<T>& u, const T s) {
  return vector3<T>(s * u.x, s * u.y, s * u.z);
}

template <class T>
inline const vector3<T> operator/(const vector3<T>& u, const T s) {
  return vector3<T>(u.x / s, u.y / s, u.z / s);
}

template <class T>
inline const vector3<T> operator*(const vector3<T>& u, const vector3<T>& v) {
  return vector3<T>(u.x * v.x, u.y * v.y, u.z * v.z);
}

template <class T>
inline const vector3<T> operator/(const vector3<T>& u, const vector3<T>& v) {
  return vector3<T>(u.x / v.x, u.y / v.y, u.z / v.z);
}

template <class T>
inline const vector4<T> operator+(const vector4<T>& u, const vector4<T>& v) {
  return vector4<T>(u.x + v.x, u.y + v.y, u.z + v.z, u.w + v.w);
}

template <class T>
inline const vector4<T> operator-(const vector4<T>& u, const vector4<T>& v) {
  return vector4<T>(u.x - v.x, u.y - v.y, u.z - v.z, u.w - v.w);
}

template <class T>
inline const vector4<T> operator+(const vector4<T>& u, const T s) {
  return vector4<T>(u.x + s, u.y + s, u.z + s, u.w + s);
}

template <class T>
inline const vector4<T> operator-(const vector4<T>& u, const T s) {
  return vector4<T>(u.x - s, u.y - s, u.z - s, u.w - s);
}

template <class T>
inline const vector4<T> operator*(const T s, const vector4<T>& u) {
  return vector4<T>(s * u.x, s * u.y, s * u.z, s * u.w);
}

template <class T>
inline const vector4<T> operator*(const vector4<T>& u, const T s) {
  return vector4<T>(s * u.x, s * u.y, s * u.z, s * u.w);
}

template <class T>
inline const vector4<T> operator/(const vector4<T>& u, const T s) {
  return vector4<T>(u.x / s, u.y / s, u.z / s, u.w / s);
}

template <class T>
inline const vector4<T> operator*(const vector4<T>& u, const vector4<T>& v) {
  return vector4<T>(u.x * v.x, u.y * v.y, u.z * v.z, u.w * v.w);
}

template <class T>
inline const vector4<T> operator/(const vector4<T>& u, const vector4<T>& v) {
  return vector4<T>(u.x / v.x, u.y / v.y, u.z / v.z, u.w / v.w);
}

template <class T>
inline matrix4<T> scale_mat4(const vector3<T>& s) {
  matrix4<T> u;
  u.as_scale(s);
  return u;
}

template <class T>
inline matrix4<T> translation_mat4(const vector3<T>& t) {
  matrix4<T> m;
  m.as_translation(t);
  return m;
}
template <class T>
inline matrix4<T> translation_mat4(T x, T y, T z) {
  matrix4<T> m;
  m.as_translation(vector3<T>(x, y, z));
  return m;
}

template <class T>
inline matrix4<T> rotation_mat4_x(T a) {
  matrix4<T> m;
  m.as_rot(a, vector3<T>(1, 0, 0));
  return m;
}

template <class T>
inline matrix4<T> rotation_mat4_y(T a) {
  matrix4<T> m;
  m.as_rot(a, vector3<T>(0, 1, 0));
  return m;
}

template <class T>
inline matrix4<T> rotation_mat4_z(T a) {
  matrix4<T> m;
  m.as_rot(a, vector3<T>(0, 0, 1));
  return m;
}

template <class T>
inline vector3<T> cross(const vector3<T>& v, const vector3<T>& w) {
  vector3<T> u;
  u.x = v.y * w.z - v.z * w.y;
  u.y = v.z * w.x - v.x * w.z;
  u.z = v.x * w.y - v.y * w.x;
  return u;
}

template <class T>
inline T dot(const vector2<T>& v, const vector2<T>& w) {
  return v.x * w.x + v.y * w.y;
}

template <class T>
inline T dot(const vector3<T>& v, const vector3<T>& w) {
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

template <class T>
inline T dot(const vector4<T>& v, const vector4<T>& w) {
  return v.x * w.x + v.y * w.y + v.z * w.z + v.w * w.w;
}

template <class T>
inline T dot(const vector3<T>& v, const vector4<T>& w) {
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

template <class T>
inline T dot(const vector4<T>& v, const vector3<T>& w) {
  return v.x * w.x + v.y * w.y + v.z * w.z;
}

template <class T>
inline T clamp(T x, T minVal, T maxVal) {
  return std::min(std::max(x, minVal), maxVal);
}

template <class T>
inline vector3<T> clamp(const vector3<T>& x,
                        const vector3<T>& minVal,
                        const vector3<T>& maxVal) {
  return vector3<T>(clamp(x.x, minVal.x, maxVal.x),
                    clamp(x.y, minVal.y, maxVal.y),
                    clamp(x.z, minVal.z, maxVal.z));
}

template <class T>
inline T min(T minVal, T maxVal) {
  return std::min(minVal, maxVal);
}

template <class T>
inline T max(T minVal, T maxVal) {
  return std::max(minVal, maxVal);
}

template <class T>
inline vector3<T> max(const vector3<T>& minVal, const vector3<T>& maxVal) {
  return vector3<T>(max(minVal.x, maxVal.x), max(minVal.y, maxVal.y),
                    max(minVal.z, maxVal.z));
}

template <class T>
inline T mix(T x, T y, T a) {
  return x * (T(1) - a) + y * a;
}

template <class T>
inline vector3<T> mix(const vector3<T>& x, const vector3<T>& y, T a) {
  return vector3<T>(mix(x.x, y.x, a), mix(x.y, y.y, a), mix(x.z, y.z, a));
}

template <class T>
inline vector3<T> mix(const vector3<T>& x,
                      const vector3<T>& y,
                      const vector3<T>& a) {
  return vector3<T>(mix(x.x, y.x, a.x), mix(x.y, y.y, a.y), mix(x.z, y.z, a.z));
}

template <class T>
vector3<T> pow(const vector3<T>& base, const vector3<T>& exponent) {
  return vector3<T>(::pow(base.x, exponent.x), ::pow(base.y, exponent.y),
                    ::pow(base.z, exponent.z));
}

template <class T>
vector3<T> sqrt(const vector3<T>& x) {
  return vector3<T>(sqrtf(x.x), sqrtf(x.y), sqrtf(x.z));
}

template <class T>
T radians(T x) {
  return nv_to_rad * T(x);
}

template <class T>
vector3<T> sin(const vector3<T>& x) {
  return vector3<T>(sinf(x.x), sinf(x.y), sinf(x.z));
}

template <class T>
T mod(T a, T b) {
  return a - b * floor(a / b);
}

template <class T>
vector2<T> mod(const vector2<T>& a, T b) {
  return {mod(a.x, b), mod(a.y, b)};
}

template <class T>
T fract(T x) {
  return x - floor(x);
}

template <class T>
inline vector3<T> reflect(const vector3<T>& n, const vector3<T>& l) {
  vector3<T> r;
  T n_dot_l;
  n_dot_l = nv_two * dot(n, l);
  mult<T>(r, l, -T(1));
  r = madd(n, n_dot_l);
  return r;
}

template <class T>
inline vector3<T> madd(const vector3<T>& v, const T& lambda) {
  vector3<T> u;
  u.x += v.x * lambda;
  u.y += v.y * lambda;
  u.z += v.z * lambda;
  return u;
}

template <class T>
inline vector3<T> mult(const vector3<T>& v, const T& lambda) {
  vector3<T> u;
  u.x = v.x * lambda;
  u.y = v.y * lambda;
  u.z = v.z * lambda;
  return u;
}

template <class T>
inline vector3<T> mult(const vector3<T>& v, const vector3<T>& w) {
  vector3<T> u;
  u.x = v.x * w.x;
  u.y = v.y * w.y;
  u.z = v.z * w.z;
  return u;
}

template <class T>
inline vector3<T> sub(const vector3<T>& v, const vector3<T>& w) {
  vector3<T> u;
  u.x = v.x - w.x;
  u.y = v.y - w.y;
  u.z = v.z - w.z;
  return u;
}

template <class T>
inline vector3<T> add(const vector3<T>& v, const vector3<T>& w) {
  vector3<T> u;
  u.x = v.x + w.x;
  u.y = v.y + w.y;
  u.z = v.z + w.z;
  return u;
}

template <class T>
inline vector3<T> pow(const vector3<T>& v, const T& e) {
  return vector3<T>(::pow(v.x, e), ::pow(v.y, e), ::pow(v.z, e));
}

template <class T>
inline void vector3<T>::orthogonalize(const vector3<T>& v) {
  //  determine the orthogonal projection of this on v : dot( v , this ) * v
  //  and subtract it from this resulting in the orthogonalized this
  vector3<T> res = dot(v, vector3<T>(x, y, z)) * v;
  x -= res.x;
  y -= res.y;
  z -= res.y;
}

template <class T>
inline vector3<T>& vector3<T>::rotateBy(const quaternion<T>& q) {
  matrix3<T> M;
  M = quat_2_mat(q);
  vector3<T> dst;
  dst = mult<T>(M, *this);
  x = dst.x;
  y = dst.y;
  z = dst.z;
  return (*this);
}

template <class T>
inline T vector3<T>::normalize() {
  T norm = sqrtf(x * x + y * y + z * z);
  if (norm > nv_eps)
    norm = T(1) / norm;
  else
    norm = T(0);
  x *= norm;
  y *= norm;
  z *= norm;
  return norm;
}

template <class T>
inline vector2<T> scale(const vector2<T>& u, const T s) {
  vector2<T> v;
  v.x = u.x * s;
  v.y = u.y * s;
  return v;
}

template <class T>
inline vector3<T> scale(const vector3<T>& u, const T s) {
  vector3<T> v;
  v.x = u.x * s;
  v.y = u.y * s;
  v.z = u.z * s;
  return v;
}

template <class T>
inline vector4<T> scale(const vector4<T>& u, const T s) {
  vector4<T> v;
  v.x = u.x * s;
  v.y = u.y * s;
  v.z = u.z * s;
  v.w = u.w * s;
  return v;
}

template <class T>
inline vector3<T> mult(const matrix3<T>& M, const vector3<T>& v) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline vector3<T> mult(const vector3<T>& v, const matrix3<T>& M) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline const vector3<T> operator*(const matrix3<T>& M, const vector3<T>& v) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline const vector3<T> operator*(const vector3<T>& v, const matrix3<T>& M) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline vector4<T> mult(const matrix4<T>& M, const vector4<T>& v) {
  vector4<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z + M.a03 * v.w;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z + M.a13 * v.w;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z + M.a23 * v.w;
  u.w = M.a30 * v.x + M.a31 * v.y + M.a32 * v.z + M.a33 * v.w;
  return u;
}

template <class T>
inline vector4<T>& mult(const vector4<T>& v, const matrix4<T>& M) {
  vector4<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z + M.a30 * v.w;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z + M.a31 * v.w;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z + M.a32 * v.w;
  u.w = M.a03 * v.x + M.a13 * v.y + M.a23 * v.z + M.a33 * v.w;
  return u;
}

template <class T>
inline const vector4<T> operator*(const matrix4<T>& M, const vector4<T>& v) {
  vector4<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z + M.a03 * v.w;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z + M.a13 * v.w;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z + M.a23 * v.w;
  u.w = M.a30 * v.x + M.a31 * v.y + M.a32 * v.z + M.a33 * v.w;
  return u;
}

template <class T>
inline const vector4<T> operator*(const matrix4<T>& M, const vector3<T>& v) {
  vector4<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z + M.a03;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z + M.a13;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z + M.a23;
  u.w = M.a30 * v.x + M.a31 * v.y + M.a32 * v.z + M.a33;
  return u;
}

template <class T>
inline const vector4<T> operator*(const vector4<T>& v, const matrix4<T>& M) {
  vector4<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z + M.a30 * v.w;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z + M.a31 * v.w;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z + M.a32 * v.w;
  u.w = M.a03 * v.x + M.a13 * v.y + M.a23 * v.z + M.a33 * v.w;
  return u;
}

template <class T>
inline vector3<T> mult_pos(const matrix4<T>& M, const vector3<T>& v) {
  vector3<T> u;
  T oow;
  T divider = v.x * M.a30 + v.y * M.a31 + v.z * M.a32 + M.a33;
  if (divider < nv_eps && divider > -nv_eps)
    oow = T(1);
  else
    oow = T(1) / divider;
  u.x = (M.a00 * v.x + M.a01 * v.y + M.a02 * v.z + M.a03) * oow;
  u.y = (M.a10 * v.x + M.a11 * v.y + M.a12 * v.z + M.a13) * oow;
  u.z = (M.a20 * v.x + M.a21 * v.y + M.a22 * v.z + M.a23) * oow;
  return u;
}

template <class T>
inline vector3<T> mult_pos(const vector3<T>& v, const matrix4<T>& M) {
  vector3<T> u;
  T oow;
  T divider = v.x * M.a03 + v.y * M.a13 + v.z * M.a23 + M.a33;
  if (divider < nv_eps && divider > -nv_eps)
    oow = T(1);
  else
    oow = T(1) / divider;

  u.x = (M.a00 * v.x + M.a10 * v.y + M.a20 * v.z + M.a30) * oow;
  u.y = (M.a01 * v.x + M.a11 * v.y + M.a21 * v.z + M.a31) * oow;
  u.z = (M.a02 * v.x + M.a12 * v.y + M.a22 * v.z + M.a32) * oow;
  return u;
}

template <class T>
inline vector3<T> mult_dir(const matrix4<T>& M, const vector3<T>& v) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline vector3<T> mult_dir(const vector3<T>& v, const matrix4<T>& M) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z;
  return u;
}

template <class T>
inline vector3<T> mult(const matrix4<T>& M, const vector3<T>& v) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a01 * v.y + M.a02 * v.z + M.a03;
  u.y = M.a10 * v.x + M.a11 * v.y + M.a12 * v.z + M.a13;
  u.z = M.a20 * v.x + M.a21 * v.y + M.a22 * v.z + M.a23;
  return u;
}

template <class T>
inline vector3<T> mult(const vector3<T>& v, const matrix4<T>& M) {
  vector3<T> u;
  u.x = M.a00 * v.x + M.a10 * v.y + M.a20 * v.z + M.a30;
  u.y = M.a01 * v.x + M.a11 * v.y + M.a21 * v.z + M.a31;
  u.z = M.a02 * v.x + M.a12 * v.y + M.a22 * v.z + M.a32;
  return u;
}

template <class T>
inline matrix4<T>& add(const matrix4<T>& A, const matrix4<T>& B) {
  matrix4<T> C;
  C.a00 = A.a00 + B.a00;
  C.a10 = A.a10 + B.a10;
  C.a20 = A.a20 + B.a20;
  C.a30 = A.a30 + B.a30;
  C.a01 = A.a01 + B.a01;
  C.a11 = A.a11 + B.a11;
  C.a21 = A.a21 + B.a21;
  C.a31 = A.a31 + B.a31;
  C.a02 = A.a02 + B.a02;
  C.a12 = A.a12 + B.a12;
  C.a22 = A.a22 + B.a22;
  C.a32 = A.a32 + B.a32;
  C.a03 = A.a03 + B.a03;
  C.a13 = A.a13 + B.a13;
  C.a23 = A.a23 + B.a23;
  C.a33 = A.a33 + B.a33;
  return C;
}

template <class T>
inline matrix3<T>& add(const matrix3<T>& A, const matrix3<T>& B) {
  matrix3<T> C;
  C.a00 = A.a00 + B.a00;
  C.a10 = A.a10 + B.a10;
  C.a20 = A.a20 + B.a20;
  C.a01 = A.a01 + B.a01;
  C.a11 = A.a11 + B.a11;
  C.a21 = A.a21 + B.a21;
  C.a02 = A.a02 + B.a02;
  C.a12 = A.a12 + B.a12;
  C.a22 = A.a22 + B.a22;
  return C;
}

// C = A * B

// C.a00 C.a01 C.a02 C.a03   A.a00 A.a01 A.a02 A.a03   B.a00 B.a01 B.a02 B.a03
//
// C.a10 C.a11 C.a12 C.a13   A.a10 A.a11 A.a12 A.a13   B.a10 B.a11 B.a12 B.a13
//
// C.a20 C.a21 C.a22 C.a23   A.a20 A.a21 A.a22 A.a23   B.a20 B.a21 B.a22 B.a23
//
// C.a30 C.a31 C.a32 C.a33 = A.a30 A.a31 A.a32 A.a33 * B.a30 B.a31 B.a32 B.a33

template <class T>
inline matrix4<T> mult(const matrix4<T>& A, const matrix4<T>& B) {
  matrix4<T> C;
  C.a00 = A.a00 * B.a00 + A.a01 * B.a10 + A.a02 * B.a20 + A.a03 * B.a30;
  C.a10 = A.a10 * B.a00 + A.a11 * B.a10 + A.a12 * B.a20 + A.a13 * B.a30;
  C.a20 = A.a20 * B.a00 + A.a21 * B.a10 + A.a22 * B.a20 + A.a23 * B.a30;
  C.a30 = A.a30 * B.a00 + A.a31 * B.a10 + A.a32 * B.a20 + A.a33 * B.a30;
  C.a01 = A.a00 * B.a01 + A.a01 * B.a11 + A.a02 * B.a21 + A.a03 * B.a31;
  C.a11 = A.a10 * B.a01 + A.a11 * B.a11 + A.a12 * B.a21 + A.a13 * B.a31;
  C.a21 = A.a20 * B.a01 + A.a21 * B.a11 + A.a22 * B.a21 + A.a23 * B.a31;
  C.a31 = A.a30 * B.a01 + A.a31 * B.a11 + A.a32 * B.a21 + A.a33 * B.a31;
  C.a02 = A.a00 * B.a02 + A.a01 * B.a12 + A.a02 * B.a22 + A.a03 * B.a32;
  C.a12 = A.a10 * B.a02 + A.a11 * B.a12 + A.a12 * B.a22 + A.a13 * B.a32;
  C.a22 = A.a20 * B.a02 + A.a21 * B.a12 + A.a22 * B.a22 + A.a23 * B.a32;
  C.a32 = A.a30 * B.a02 + A.a31 * B.a12 + A.a32 * B.a22 + A.a33 * B.a32;
  C.a03 = A.a00 * B.a03 + A.a01 * B.a13 + A.a02 * B.a23 + A.a03 * B.a33;
  C.a13 = A.a10 * B.a03 + A.a11 * B.a13 + A.a12 * B.a23 + A.a13 * B.a33;
  C.a23 = A.a20 * B.a03 + A.a21 * B.a13 + A.a22 * B.a23 + A.a23 * B.a33;
  C.a33 = A.a30 * B.a03 + A.a31 * B.a13 + A.a32 * B.a23 + A.a33 * B.a33;

  return C;
}

template <class T>
inline matrix4<T> matrix4<T>::operator*(const matrix4<T>& B) const {
  matrix4<T> C;
  C.a00 = a00 * B.a00 + a01 * B.a10 + a02 * B.a20 + a03 * B.a30;
  C.a10 = a10 * B.a00 + a11 * B.a10 + a12 * B.a20 + a13 * B.a30;
  C.a20 = a20 * B.a00 + a21 * B.a10 + a22 * B.a20 + a23 * B.a30;
  C.a30 = a30 * B.a00 + a31 * B.a10 + a32 * B.a20 + a33 * B.a30;
  C.a01 = a00 * B.a01 + a01 * B.a11 + a02 * B.a21 + a03 * B.a31;
  C.a11 = a10 * B.a01 + a11 * B.a11 + a12 * B.a21 + a13 * B.a31;
  C.a21 = a20 * B.a01 + a21 * B.a11 + a22 * B.a21 + a23 * B.a31;
  C.a31 = a30 * B.a01 + a31 * B.a11 + a32 * B.a21 + a33 * B.a31;
  C.a02 = a00 * B.a02 + a01 * B.a12 + a02 * B.a22 + a03 * B.a32;
  C.a12 = a10 * B.a02 + a11 * B.a12 + a12 * B.a22 + a13 * B.a32;
  C.a22 = a20 * B.a02 + a21 * B.a12 + a22 * B.a22 + a23 * B.a32;
  C.a32 = a30 * B.a02 + a31 * B.a12 + a32 * B.a22 + a33 * B.a32;
  C.a03 = a00 * B.a03 + a01 * B.a13 + a02 * B.a23 + a03 * B.a33;
  C.a13 = a10 * B.a03 + a11 * B.a13 + a12 * B.a23 + a13 * B.a33;
  C.a23 = a20 * B.a03 + a21 * B.a13 + a22 * B.a23 + a23 * B.a33;
  C.a33 = a30 * B.a03 + a31 * B.a13 + a32 * B.a23 + a33 * B.a33;
  return C;
}

// C = A * B

// C.a00 C.a01 C.a02   A.a00 A.a01 A.a02   B.a00 B.a01 B.a02
//
// C.a10 C.a11 C.a12   A.a10 A.a11 A.a12   B.a10 B.a11 B.a12
//
// C.a20 C.a21 C.a22 = A.a20 A.a21 A.a22 * B.a20 B.a21 B.a22

template <class T>
inline matrix3<T> mult(const matrix3<T>& A, const matrix3<T>& B) {
  matrix3<T> C;
  // If there is self assignment involved
  // we can't go without a temporary.
  C.a00 = A.a00 * B.a00 + A.a01 * B.a10 + A.a02 * B.a20;
  C.a10 = A.a10 * B.a00 + A.a11 * B.a10 + A.a12 * B.a20;
  C.a20 = A.a20 * B.a00 + A.a21 * B.a10 + A.a22 * B.a20;
  C.a01 = A.a00 * B.a01 + A.a01 * B.a11 + A.a02 * B.a21;
  C.a11 = A.a10 * B.a01 + A.a11 * B.a11 + A.a12 * B.a21;
  C.a21 = A.a20 * B.a01 + A.a21 * B.a11 + A.a22 * B.a21;
  C.a02 = A.a00 * B.a02 + A.a01 * B.a12 + A.a02 * B.a22;
  C.a12 = A.a10 * B.a02 + A.a11 * B.a12 + A.a12 * B.a22;
  C.a22 = A.a20 * B.a02 + A.a21 * B.a12 + A.a22 * B.a22;
  return C;
}

template <class T>
inline matrix3<T> matrix3<T>::operator*(const matrix3<T>& B) const {
  matrix3<T> C;
  // If there is self assignment involved
  // we can't go without a temporary.
  C.a00 = a00 * B.a00 + a01 * B.a10 + a02 * B.a20;
  C.a10 = a10 * B.a00 + a11 * B.a10 + a12 * B.a20;
  C.a20 = a20 * B.a00 + a21 * B.a10 + a22 * B.a20;
  C.a01 = a00 * B.a01 + a01 * B.a11 + a02 * B.a21;
  C.a11 = a10 * B.a01 + a11 * B.a11 + a12 * B.a21;
  C.a21 = a20 * B.a01 + a21 * B.a11 + a22 * B.a21;
  C.a02 = a00 * B.a02 + a01 * B.a12 + a02 * B.a22;
  C.a12 = a10 * B.a02 + a11 * B.a12 + a12 * B.a22;
  C.a22 = a20 * B.a02 + a21 * B.a12 + a22 * B.a22;
  return C;
}

template <class T>
inline matrix4<T> transpose(const matrix4<T>& A) {
  matrix4<T> B;
  B.a00 = A.a00;
  B.a01 = A.a10;
  B.a02 = A.a20;
  B.a03 = A.a30;
  B.a10 = A.a01;
  B.a11 = A.a11;
  B.a12 = A.a21;
  B.a13 = A.a31;
  B.a20 = A.a02;
  B.a21 = A.a12;
  B.a22 = A.a22;
  B.a23 = A.a32;
  B.a30 = A.a03;
  B.a31 = A.a13;
  B.a32 = A.a23;
  B.a33 = A.a33;
  return B;
}

template <class T>
inline matrix3<T> transpose(const matrix3<T>& A) {
  matrix3<T> B;
  B.a00 = A.a00;
  B.a01 = A.a10;
  B.a02 = A.a20;
  B.a10 = A.a01;
  B.a11 = A.a11;
  B.a12 = A.a21;
  B.a20 = A.a02;
  B.a21 = A.a12;
  B.a22 = A.a22;
  return B;
}

/*
    calculate the determinent of a 2x2 matrix in the from

    | a1 a2 |
    | b1 b2 |

*/
template <class T>
inline T det2x2(T a1, T a2, T b1, T b2) {
  return a1 * b2 - b1 * a2;
}

/*
    calculate the determinent of a 3x3 matrix in the from

    | a1 a2 a3 |
    | b1 b2 b3 |
    | c1 c2 c3 |

*/
template <class T>
inline T det3x3(T a1, T a2, T a3, T b1, T b2, T b3, T c1, T c2, T c3) {
  return a1 * det2x2(b2, b3, c2, c3) - b1 * det2x2(a2, a3, c2, c3) +
         c1 * det2x2(a2, a3, b2, b3);
}

template <class T>
inline matrix4<T> invert(const matrix4<T>& A) {
  matrix4<T> B;
  T det, oodet;

  B.a00 = det3x3(A.a11, A.a21, A.a31, A.a12, A.a22, A.a32, A.a13, A.a23, A.a33);
  B.a10 =
      -det3x3(A.a10, A.a20, A.a30, A.a12, A.a22, A.a32, A.a13, A.a23, A.a33);
  B.a20 = det3x3(A.a10, A.a20, A.a30, A.a11, A.a21, A.a31, A.a13, A.a23, A.a33);
  B.a30 =
      -det3x3(A.a10, A.a20, A.a30, A.a11, A.a21, A.a31, A.a12, A.a22, A.a32);

  B.a01 =
      -det3x3(A.a01, A.a21, A.a31, A.a02, A.a22, A.a32, A.a03, A.a23, A.a33);
  B.a11 = det3x3(A.a00, A.a20, A.a30, A.a02, A.a22, A.a32, A.a03, A.a23, A.a33);
  B.a21 =
      -det3x3(A.a00, A.a20, A.a30, A.a01, A.a21, A.a31, A.a03, A.a23, A.a33);
  B.a31 = det3x3(A.a00, A.a20, A.a30, A.a01, A.a21, A.a31, A.a02, A.a22, A.a32);

  B.a02 = det3x3(A.a01, A.a11, A.a31, A.a02, A.a12, A.a32, A.a03, A.a13, A.a33);
  B.a12 =
      -det3x3(A.a00, A.a10, A.a30, A.a02, A.a12, A.a32, A.a03, A.a13, A.a33);
  B.a22 = det3x3(A.a00, A.a10, A.a30, A.a01, A.a11, A.a31, A.a03, A.a13, A.a33);
  B.a32 =
      -det3x3(A.a00, A.a10, A.a30, A.a01, A.a11, A.a31, A.a02, A.a12, A.a32);

  B.a03 =
      -det3x3(A.a01, A.a11, A.a21, A.a02, A.a12, A.a22, A.a03, A.a13, A.a23);
  B.a13 = det3x3(A.a00, A.a10, A.a20, A.a02, A.a12, A.a22, A.a03, A.a13, A.a23);
  B.a23 =
      -det3x3(A.a00, A.a10, A.a20, A.a01, A.a11, A.a21, A.a03, A.a13, A.a23);
  B.a33 = det3x3(A.a00, A.a10, A.a20, A.a01, A.a11, A.a21, A.a02, A.a12, A.a22);

  det = (A.a00 * B.a00) + (A.a01 * B.a10) + (A.a02 * B.a20) + (A.a03 * B.a30);

  // The following divions goes unchecked for division
  // by zero. We should consider throwing an exception
  // if det < eps.
  oodet = T(1) / det;

  B.a00 *= oodet;
  B.a10 *= oodet;
  B.a20 *= oodet;
  B.a30 *= oodet;

  B.a01 *= oodet;
  B.a11 *= oodet;
  B.a21 *= oodet;
  B.a31 *= oodet;

  B.a02 *= oodet;
  B.a12 *= oodet;
  B.a22 *= oodet;
  B.a32 *= oodet;

  B.a03 *= oodet;
  B.a13 *= oodet;
  B.a23 *= oodet;
  B.a33 *= oodet;

  return B;
}

template <class T>
inline matrix4<T> inverse(const matrix4<T>& A) {
  return invert(A);
}

template <class T>
inline matrix4<T> invert(const matrix4<T>& A, bool& valid) {
  matrix4<T> B;
  T det, oodet;

  B.a00 = det3x3(A.a11, A.a21, A.a31, A.a12, A.a22, A.a32, A.a13, A.a23, A.a33);
  B.a10 =
      -det3x3(A.a10, A.a20, A.a30, A.a12, A.a22, A.a32, A.a13, A.a23, A.a33);
  B.a20 = det3x3(A.a10, A.a20, A.a30, A.a11, A.a21, A.a31, A.a13, A.a23, A.a33);
  B.a30 =
      -det3x3(A.a10, A.a20, A.a30, A.a11, A.a21, A.a31, A.a12, A.a22, A.a32);

  B.a01 =
      -det3x3(A.a01, A.a21, A.a31, A.a02, A.a22, A.a32, A.a03, A.a23, A.a33);
  B.a11 = det3x3(A.a00, A.a20, A.a30, A.a02, A.a22, A.a32, A.a03, A.a23, A.a33);
  B.a21 =
      -det3x3(A.a00, A.a20, A.a30, A.a01, A.a21, A.a31, A.a03, A.a23, A.a33);
  B.a31 = det3x3(A.a00, A.a20, A.a30, A.a01, A.a21, A.a31, A.a02, A.a22, A.a32);

  B.a02 = det3x3(A.a01, A.a11, A.a31, A.a02, A.a12, A.a32, A.a03, A.a13, A.a33);
  B.a12 =
      -det3x3(A.a00, A.a10, A.a30, A.a02, A.a12, A.a32, A.a03, A.a13, A.a33);
  B.a22 = det3x3(A.a00, A.a10, A.a30, A.a01, A.a11, A.a31, A.a03, A.a13, A.a33);
  B.a32 =
      -det3x3(A.a00, A.a10, A.a30, A.a01, A.a11, A.a31, A.a02, A.a12, A.a32);

  B.a03 =
      -det3x3(A.a01, A.a11, A.a21, A.a02, A.a12, A.a22, A.a03, A.a13, A.a23);
  B.a13 = det3x3(A.a00, A.a10, A.a20, A.a02, A.a12, A.a22, A.a03, A.a13, A.a23);
  B.a23 =
      -det3x3(A.a00, A.a10, A.a20, A.a01, A.a11, A.a21, A.a03, A.a13, A.a23);
  B.a33 = det3x3(A.a00, A.a10, A.a20, A.a01, A.a11, A.a21, A.a02, A.a12, A.a22);

  det = (A.a00 * B.a00) + (A.a01 * B.a10) + (A.a02 * B.a20) + (A.a03 * B.a30);

  valid = det >= FLT_MIN;

  // The following divions goes unchecked for division
  // by zero. We should consider throwing an exception
  // if det < eps.
  oodet = T(1) / det;

  B.a00 *= oodet;
  B.a10 *= oodet;
  B.a20 *= oodet;
  B.a30 *= oodet;

  B.a01 *= oodet;
  B.a11 *= oodet;
  B.a21 *= oodet;
  B.a31 *= oodet;

  B.a02 *= oodet;
  B.a12 *= oodet;
  B.a22 *= oodet;
  B.a32 *= oodet;

  B.a03 *= oodet;
  B.a13 *= oodet;
  B.a23 *= oodet;
  B.a33 *= oodet;

  return B;
}

template <class T>
inline matrix4<T> invert_rot_trans(const matrix4<T>& A) {
  matrix4<T> B;
  B.a00 = A.a00;
  B.a10 = A.a01;
  B.a20 = A.a02;
  B.a30 = A.a30;
  B.a01 = A.a10;
  B.a11 = A.a11;
  B.a21 = A.a12;
  B.a31 = A.a31;
  B.a02 = A.a20;
  B.a12 = A.a21;
  B.a22 = A.a22;
  B.a32 = A.a32;
  B.a03 = -(A.a00 * A.a03 + A.a10 * A.a13 + A.a20 * A.a23);
  B.a13 = -(A.a01 * A.a03 + A.a11 * A.a13 + A.a21 * A.a23);
  B.a23 = -(A.a02 * A.a03 + A.a12 * A.a13 + A.a22 * A.a23);
  B.a33 = A.a33;
  return B;
}

template <class T>
inline T det(const matrix3<T>& A) {
  return det3x3(A.a00, A.a01, A.a02, A.a10, A.a11, A.a12, A.a20, A.a21, A.a22);
}

template <class T>
inline T det(const matrix4<T>& A) {
  return det3x3(A.a00, A.a01, A.a02, A.a10, A.a11, A.a12, A.a20, A.a21, A.a22);
}

template <class T>
inline matrix3<T> invert(const matrix3<T>& A) {
  matrix3<T> B;
  T det, oodet;

  B.a00 = (A.a11 * A.a22 - A.a21 * A.a12);
  B.a10 = -(A.a10 * A.a22 - A.a20 * A.a12);
  B.a20 = (A.a10 * A.a21 - A.a20 * A.a11);
  B.a01 = -(A.a01 * A.a22 - A.a21 * A.a02);
  B.a11 = (A.a00 * A.a22 - A.a20 * A.a02);
  B.a21 = -(A.a00 * A.a21 - A.a20 * A.a01);
  B.a02 = (A.a01 * A.a12 - A.a11 * A.a02);
  B.a12 = -(A.a00 * A.a12 - A.a10 * A.a02);
  B.a22 = (A.a00 * A.a11 - A.a10 * A.a01);

  det = (A.a00 * B.a00) + (A.a01 * B.a10) + (A.a02 * B.a20);

  oodet = T(1) / det;

  B.a00 *= oodet;
  B.a01 *= oodet;
  B.a02 *= oodet;
  B.a10 *= oodet;
  B.a11 *= oodet;
  B.a12 *= oodet;
  B.a20 *= oodet;
  B.a21 *= oodet;
  B.a22 *= oodet;
  return B;
}

template <class T>
inline matrix3<T> inverse(const matrix3<T>& A) {
  return invert(A);
}

template <class T>
inline matrix4<T> look_at(const vector3<T>& eye,
                          const vector3<T>& center,
                          const vector3<T>& up) {
  matrix4<T> M;
  vector3<T> x, y, z;

  // make rotation matrix

  // Z vector
  z.x = eye.x - center.x;
  z.y = eye.y - center.y;
  z.z = eye.z - center.z;
  z.normalize();

  // Y vector
  y.x = up.x;
  y.y = up.y;
  y.z = up.z;

  // X vector = Y cross Z
  x = cross(y, z);

  // Recompute Y = Z cross X
  y = cross(z, x);

  // cross product gives area of parallelogram, which is < 1.0 for
  // non-perpendicular unit-length vectors; so normalize x, y here
  x.normalize();
  y.normalize();

  M.a00 = x.x;
  M.a01 = x.y;
  M.a02 = x.z;
  M.a03 = -x.x * eye.x - x.y * eye.y - x.z * eye.z;
  M.a10 = y.x;
  M.a11 = y.y;
  M.a12 = y.z;
  M.a13 = -y.x * eye.x - y.y * eye.y - y.z * eye.z;
  M.a20 = z.x;
  M.a21 = z.y;
  M.a22 = z.z;
  M.a23 = -z.x * eye.x - z.y * eye.y - z.z * eye.z;
  M.a30 = T(0);
  M.a31 = T(0);
  M.a32 = T(0);
  M.a33 = T(1);
  return M;
}

template <class T>
inline matrix4<T> frustum(const T l,
                          const T r,
                          const T b,
                          const T t,
                          const T n,
                          const T f) {
  matrix4<T> M;
  M.a00 = (nv_two * n) / (r - l);
  M.a10 = 0.0;
  M.a20 = 0.0;
  M.a30 = 0.0;

  M.a01 = 0.0;
  M.a11 = (nv_two * n) / (t - b);
  M.a21 = 0.0;
  M.a31 = 0.0;

  M.a02 = (r + l) / (r - l);
  M.a12 = (t + b) / (t - b);
  M.a22 = -(f + n) / (f - n);
  M.a32 = -T(1);

  M.a03 = 0.0;
  M.a13 = 0.0;
  M.a23 = -(nv_two * f * n) / (f - n);
  M.a33 = 0.0;
  return M;
}

template <class T>
inline matrix4<T> frustum01(const T l,
                            const T r,
                            const T b,
                            const T t,
                            const T n,
                            const T f) {
  matrix4<T> M;
  M.a00 = (nv_two * n) / (r - l);
  M.a10 = 0.0;
  M.a20 = 0.0;
  M.a30 = 0.0;

  M.a01 = 0.0;
  M.a11 = (nv_two * n) / (t - b);
  M.a21 = 0.0;
  M.a31 = 0.0;

  M.a02 = (r + l) / (r - l);
  M.a12 = (t + b) / (t - b);
  M.a22 = (f) / (n - f);
  M.a32 = -T(1);

  M.a03 = 0.0;
  M.a13 = 0.0;
  M.a23 = (f * n) / (n - f);
  M.a33 = 0.0;
  return M;
}

template <class T>
inline matrix4<T> frustum01Rev(const T l,
                               const T r,
                               const T b,
                               const T t,
                               const T n,
                               const T f) {
  matrix4<T> M;
  M.a00 = (nv_two * n) / (r - l);
  M.a10 = 0.0;
  M.a20 = 0.0;
  M.a30 = 0.0;

  M.a01 = 0.0;
  M.a11 = (nv_two * n) / (t - b);
  M.a21 = 0.0;
  M.a31 = 0.0;

  M.a02 = (r + l) / (r - l);
  M.a12 = (t + b) / (t - b);
  M.a22 = -((f) / (n - f)) - T(1);
  M.a32 = -T(1);

  M.a03 = 0.0;
  M.a13 = 0.0;
  M.a23 = -(f * n) / (n - f);
  M.a33 = 0.0;
  return M;
}

template <class T>
inline matrix4<T> perspective(const T fovy,
                              const T aspect,
                              const T n,
                              const T f) {
  T xmin, xmax, ymin, ymax;

  ymax = n * tanf(fovy * nv_to_rad * T(0.5));
  ymin = -ymax;

  xmin = ymin * aspect;
  xmax = ymax * aspect;

  return frustum(xmin, xmax, ymin, ymax, n, f);
}

template <class T>
inline matrix4<T> perspective01(const T fovy,
                                const T aspect,
                                const T n,
                                const T f) {
  T xmin, xmax, ymin, ymax;

  ymax = n * tanf(fovy * nv_to_rad * T(0.5));
  ymin = -ymax;

  xmin = ymin * aspect;
  xmax = ymax * aspect;

  return frustum01(xmin, xmax, ymin, ymax, n, f);
}

template <class T>
inline matrix4<T> perspective01Rev(const T fovy,
                                   const T aspect,
                                   const T n,
                                   const T f) {
  T xmin, xmax, ymin, ymax;

  ymax = n * tanf(fovy * nv_to_rad * T(0.5));
  ymin = -ymax;

  xmin = ymin * aspect;
  xmax = ymax * aspect;

  return frustum01Rev(xmin, xmax, ymin, ymax, n, f);
}

// Vulkan uses DX style 0,1 z clip space and inversed Y
template <class T>
inline matrix4<T> perspectiveVK(T fovy, T aspect, T nearPlane, T farPlane) {
  matrix4<T> M;
  float r, l, b, t;
  float f = farPlane;
  float n = nearPlane;

  t = n * tanf(fovy * nv_to_rad * T(0.5));
  b = -t;

  l = b * aspect;
  r = t * aspect;

  M.a00 = (2 * n) / (r - l);
  M.a10 = 0;
  M.a20 = 0;
  M.a30 = 0;

  M.a01 = 0;
  M.a11 = -(2 * n) / (t - b);
  M.a21 = 0;
  M.a31 = 0;

  M.a02 = (r + l) / (r - l);
  M.a12 = (t + b) / (t - b);
  M.a22 = -(f) / (f - n);
  M.a32 = -1;

  M.a03 = 0;
  M.a13 = 0;
  M.a23 = (f * n) / (n - f);
  M.a33 = 0;

  return M;
}

template <class T>
inline matrix4<T> ortho(const T left,
                        const T right,
                        const T bottom,
                        const T top,
                        const T n,
                        const T f) {
  matrix4<T> M;
  M.a00 = nv_two / (right - left);
  M.a01 = T(0);
  M.a02 = T(0);
  M.a03 = -(right + left) / (right - left);
  M.a10 = T(0);
  M.a11 = nv_two / (top - bottom);
  M.a12 = T(0);
  M.a13 = -(top + bottom) / (top - bottom);
  M.a20 = T(0);
  M.a21 = T(0);
  M.a22 = -nv_two / (f - n);
  M.a23 = -(f + n) / (f - n);
  M.a30 = T(0);
  M.a31 = T(0);
  M.a32 = T(0);
  M.a33 = T(1);
  return M;
}

template <class T>
inline vector2<T> normalize(const vector2<T>& v) {
  vector2<T> u(v);
  T norm = sqrtf(u.x * u.x + u.y * u.y);
  if (norm > nv_eps)
    norm = T(1) / norm;
  else
    norm = T(0);
  return scale(u, norm);
}

template <class T>
inline vector3<T> normalize(const vector3<T>& v) {
  vector3<T> u(v);
  T norm = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z);
  if (norm > nv_eps)
    norm = T(1) / norm;
  else
    norm = T(0);
  return scale(u, norm);
}

template <class T>
inline vector4<T> normalize(const vector4<T>& v) {
  vector4<T> u(v);
  T norm = sqrtf(u.x * u.x + u.y * u.y + u.z * u.z + u.w * u.w);
  if (norm > nv_eps)
    norm = T(1) / norm;
  else
    norm = T(0);
  return scale(u, norm);
}

template <class T>
inline quaternion<T> normalize(const quaternion<T>& q) {
  quaternion<T> p(q);
  T norm = sqrtf(p.x * p.x + p.y * p.y + p.z * p.z + p.w * p.w);
  if (norm > nv_eps)
    norm = T(1) / norm;
  else
    norm = T(0);
  p.x *= norm;
  p.y *= norm;
  p.z *= norm;
  p.w *= norm;
  return p;
}

template <class T>
inline quaternion<T>::quaternion(const vector3<T>& axis, T angle) {
  T len = axis.norm();
  if (len) {
    T invLen = 1 / len;
    T angle2 = angle / 2;
    T scale = sinf(angle2) * invLen;
    x = scale * axis[0];
    y = scale * axis[1];
    z = scale * axis[2];
    w = cosf(angle2);
  }
}

template <class T>
inline quaternion<T>::quaternion(const matrix3<T>& rot) {
  from_matrix(rot);
}
template <class T>
inline quaternion<T>::quaternion(const matrix4<T>& rot) {
  from_matrix(rot);
}

template <class T>
inline quaternion<T>& quaternion<T>::operator=(
    const quaternion<T>& quaternion) {
  x = quaternion.x;
  y = quaternion.y;
  z = quaternion.z;
  w = quaternion.w;
  return *this;
}

template <class T>

inline quaternion<T> quaternion<T>::inverse() {
  return quaternion<T>(-x, -y, -z, w);
}
template <class T>
inline quaternion<T> quaternion<T>::conjugate() {
  return quaternion<T>(-x, -y, -z, w);
}

template <class T>
inline void quaternion<T>::normalize() {
  T len = sqrtf(x * x + y * y + z * z + w * w);
  if (len > 0) {
    T invLen = 1 / len;
    x *= invLen;
    y *= invLen;
    z *= invLen;
    w *= invLen;
  }
}

template <class T>
inline void quaternion<T>::from_matrix(const matrix3<T>& mat) {
  T trace = mat(0, 0) + mat(1, 1) + mat(2, 2);
  if (trace > T(0)) {
    T scale = sqrtf(trace + T(1));
    w = T(0.5) * scale;
    scale = T(0.5) / scale;
    x = scale * (mat(2, 1) - mat(1, 2));
    y = scale * (mat(0, 2) - mat(2, 0));
    z = scale * (mat(1, 0) - mat(0, 1));
  } else {
    static int next[] = {1, 2, 0};
    int i = 0;
    if (mat(1, 1) > mat(0, 0))
      i = 1;
    if (mat(2, 2) > mat(i, i))
      i = 2;
    int j = next[i];
    int k = next[j];
    T scale = sqrtf(mat(i, i) - mat(j, j) - mat(k, k) + 1);
    T* q[] = {&x, &y, &z};
    *q[i] = 0.5f * scale;
    scale = 0.5f / scale;
    w = scale * (mat(k, j) - mat(j, k));
    *q[j] = scale * (mat(j, i) + mat(i, j));
    *q[k] = scale * (mat(k, i) + mat(i, k));
  }
}

template <class T>
inline void quaternion<T>::from_matrix(const matrix4<T>& mat) {
  T trace = mat(0, 0) + mat(1, 1) + mat(2, 2);
  if (trace > T(0)) {
    T scale = sqrtf(trace + T(1));
    w = T(0.5) * scale;
    scale = T(0.5) / scale;
    x = scale * (mat(2, 1) - mat(1, 2));
    y = scale * (mat(0, 2) - mat(2, 0));
    z = scale * (mat(1, 0) - mat(0, 1));
  } else {
    static int next[] = {1, 2, 0};
    int i = 0;
    if (mat(1, 1) > mat(0, 0))
      i = 1;
    if (mat(2, 2) > mat(i, i))
      i = 2;
    int j = next[i];
    int k = next[j];
    T scale = sqrtf(mat(i, i) - mat(j, j) - mat(k, k) + 1);
    T* q[] = {&x, &y, &z};
    *q[i] = 0.5f * scale;
    scale = 0.5f / scale;
    w = scale * (mat(k, j) - mat(j, k));
    *q[j] = scale * (mat(j, i) + mat(i, j));
    *q[k] = scale * (mat(k, i) + mat(i, k));
  }
}

template <class T>
inline void quaternion<T>::to_matrix(matrix3<T>& mat) const {
  T x2 = x * 2;
  T y2 = y * 2;
  T z2 = z * 2;
  T wx = x2 * w;
  T wy = y2 * w;
  T wz = z2 * w;
  T xx = x2 * x;
  T xy = y2 * x;
  T xz = z2 * x;
  T yy = y2 * y;
  T yz = z2 * y;
  T zz = z2 * z;
  mat(0, 0) = 1 - (yy + zz);
  mat(0, 1) = xy - wz;
  mat(0, 2) = xz + wy;
  mat(1, 0) = xy + wz;
  mat(1, 1) = 1 - (xx + zz);
  mat(1, 2) = yz - wx;
  mat(2, 0) = xz - wy;
  mat(2, 1) = yz + wx;
  mat(2, 2) = 1 - (xx + yy);
}

template <class T>
inline void quaternion<T>::to_matrix(matrix4<T>& mat) const {
  T x2 = x * 2;
  T y2 = y * 2;
  T z2 = z * 2;
  T wx = x2 * w;
  T wy = y2 * w;
  T wz = z2 * w;
  T xx = x2 * x;
  T xy = y2 * x;
  T xz = z2 * x;
  T yy = y2 * y;
  T yz = z2 * y;
  T zz = z2 * z;
  mat(0, 0) = 1 - (yy + zz);
  mat(0, 1) = xy - wz;
  mat(0, 2) = xz + wy;
  mat(0, 3) = 0.0f;
  mat(1, 0) = xy + wz;
  mat(1, 1) = 1 - (xx + zz);
  mat(1, 2) = yz - wx;
  mat(1, 3) = 0.0f;
  mat(2, 0) = xz - wy;
  mat(2, 1) = yz + wx;
  mat(2, 2) = 1 - (xx + yy);
  mat(2, 3) = 0.0f;
  mat(3, 0) = 0.0f;
  mat(3, 1) = 0.0f;
  mat(3, 2) = 0.0f;
  mat(3, 3) = 1.0f;
}

template <class T>
inline const quaternion<T> operator*(const quaternion<T>& p,
                                     const quaternion<T>& q) {
  return quaternion<T>(p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
                       p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z,
                       p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x,
                       p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z);
}

template <class T>
inline const quaternion<T> mul(const quaternion<T>& p, const quaternion<T>& q) {
  return quaternion<T>(p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y,
                       p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z,
                       p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x,
                       p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z);
}

template <class T>
inline matrix3<T> quat_2_mat(const quaternion<T>& q) {
  matrix3<T> M;
  q.to_matrix(M);
  return M;
}

template <class T>
inline quaternion<T> mat_2_quat(const matrix3<T>& M) {
  quaternion<T> q;
  q.from_matrix(M);
  return q;
}

template <class T>
inline quaternion<T> mat_2_quat(const matrix4<T>& M) {
  quaternion<T> q;
  matrix3<T> m;
  m = M.get_rot_mat3();
  q.from_matrix(m);
  return q;
}

/*
    Given an axis and angle, compute quaternion.
 */
template <class T>
inline quaternion<T> axis_to_quat(const vector3<T>& a, const T phi) {
  quaternion<T> q;
  vector3<T> tmp(a.x, a.y, a.z);

  tmp.normalize();
  T s = sinf(phi / nv_two);
  q.x = s * tmp.x;
  q.y = s * tmp.y;
  q.z = s * tmp.z;
  q.w = cosf(phi / nv_two);
  return q;
}

template <class T>
inline quaternion<T> conj(const quaternion<T>& q) {
  quaternion<T> p(q);
  p.x = -p.x;
  p.y = -p.y;
  p.z = -p.z;
  return p;
}

template <class T>
inline quaternion<T> add_quats(const quaternion<T>& q1,
                               const quaternion<T>& q2) {
  quaternion<T> p;
  quaternion<T> t1, t2;

  t1 = q1;
  t1.x *= q2.w;
  t1.y *= q2.w;
  t1.z *= q2.w;

  t2 = q2;
  t2.x *= q1.w;
  t2.y *= q1.w;
  t2.z *= q1.w;

  p.x = (q2.y * q1.z) - (q2.z * q1.y) + t1.x + t2.x;
  p.y = (q2.z * q1.x) - (q2.x * q1.z) + t1.y + t2.y;
  p.z = (q2.x * q1.y) - (q2.y * q1.x) + t1.z + t2.z;
  p.w = q1.w * q2.w - (q1.x * q2.x + q1.y * q2.y + q1.z * q2.z);

  return p;
}

template <class T>
inline T dot(const quaternion<T>& q1, const quaternion<T>& q2) {
  return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
}

#if !defined(_MSC_VER) || (defined(_MSC_VER) && (_MSC_VER < 1920))
#ifndef acosf
#define acosf acos
#endif
#endif

template <class T>
inline quaternion<T> slerp_quats(T s,
                                 const quaternion<T>& q1,
                                 const quaternion<T>& q2) {
  quaternion<T> p;
  T cosine = dot(q1, q2);
  if (cosine < -1)
    cosine = -1;
  else if (cosine > 1)
    cosine = 1;
  T angle = (T)acosf(cosine);
  if (fabs(angle) < nv_eps) {
    p = q1;
    return p;
  }
  T sine = sinf(angle);
  T sineInv = 1.0f / sine;
  T c1 = sinf((1.0f - s) * angle) * sineInv;
  T c2 = sinf(s * angle) * sineInv;
  p.x = c1 * q1.x + c2 * q2.x;
  p.y = c1 * q1.y + c2 * q2.y;
  p.z = c1 * q1.z + c2 * q2.z;
  p.w = c1 * q1.w + c2 * q2.w;
  return p;
}

const int HALF_RAND = (RAND_MAX / 2);

template <class T>
inline T nv_random() {
  return ((T)(rand() - HALF_RAND) / (T)HALF_RAND);
}

// v is normalized
// theta in radians
template <class T>
inline matrix3<T>& matrix3<T>::set_rot(const T& theta, const vector3<T>& v) {
  T ct = T(::cos(theta));
  T st = T(::sin(theta));

  T xx = v.x * v.x;
  T yy = v.y * v.y;
  T zz = v.z * v.z;
  T xy = v.x * v.y;
  T xz = v.x * v.z;
  T yz = v.y * v.z;

  a00 = xx + ct * (1 - xx);
  a01 = xy + ct * (-xy) + st * -v.z;
  a02 = xz + ct * (-xz) + st * v.y;

  a10 = xy + ct * (-xy) + st * v.z;
  a11 = yy + ct * (1 - yy);
  a12 = yz + ct * (-yz) + st * -v.x;

  a20 = xz + ct * (-xz) + st * -v.y;
  a21 = yz + ct * (-yz) + st * v.x;
  a22 = zz + ct * (1 - zz);
  return *this;
}

template <class T>
inline matrix3<T>& matrix3<T>::set_rot(const vector3<T>& u,
                                       const vector3<T>& v) {
  T phi;
  T h;
  T lambda;
  vector3<T> w;

  w = cross(u, v);
  phi = dot(u, v);
  lambda = dot(w, w);
  if (lambda > nv_eps)
    h = (T(1) - phi) / lambda;
  else
    h = lambda;

  T hxy = w.x * w.y * h;
  T hxz = w.x * w.z * h;
  T hyz = w.y * w.z * h;

  a00 = phi + w.x * w.x * h;
  a01 = hxy - w.z;
  a02 = hxz + w.y;

  a10 = hxy + w.z;
  a11 = phi + w.y * w.y * h;
  a12 = hyz - w.x;

  a20 = hxz - w.y;
  a21 = hyz + w.x;
  a22 = phi + w.z * w.z * h;
  return *this;
}

template <class T>
inline T matrix3<T>::norm_one() {
  T sum, max = T(0);
  sum = fabs(a00) + fabs(a10) + fabs(a20);
  if (max < sum)
    max = sum;
  sum = fabs(a01) + fabs(a11) + fabs(a21);
  if (max < sum)
    max = sum;
  sum = fabs(a02) + fabs(a12) + fabs(a22);
  if (max < sum)
    max = sum;
  return max;
}

template <class T>
inline T matrix3<T>::norm_inf() {
  T sum, max = T(0);
  sum = fabs(a00) + fabs(a01) + fabs(a02);
  if (max < sum)
    max = sum;
  sum = fabs(a10) + fabs(a11) + fabs(a12);
  if (max < sum)
    max = sum;
  sum = fabs(a20) + fabs(a21) + fabs(a22);
  if (max < sum)
    max = sum;
  return max;
}

template <class T>
inline matrix4<T>& matrix4<T>::set_rot(const quaternion<T>& q) {
  matrix3<T> m;
  q.to_matrix(m);
  set_rot(m);
  return *this;
}

// v is normalized
// theta in radians
template <class T>
inline matrix4<T>& matrix4<T>::set_rot(const T& theta, const vector3<T>& v) {
  T ct = T(::cos(theta));
  T st = T(::sin(theta));

  T xx = v.x * v.x;
  T yy = v.y * v.y;
  T zz = v.z * v.z;
  T xy = v.x * v.y;
  T xz = v.x * v.z;
  T yz = v.y * v.z;

  a00 = xx + ct * (1 - xx);
  a01 = xy + ct * (-xy) + st * -v.z;
  a02 = xz + ct * (-xz) + st * v.y;

  a10 = xy + ct * (-xy) + st * v.z;
  a11 = yy + ct * (1 - yy);
  a12 = yz + ct * (-yz) + st * -v.x;

  a20 = xz + ct * (-xz) + st * -v.y;
  a21 = yz + ct * (-yz) + st * v.x;
  a22 = zz + ct * (1 - zz);
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::set_rot(const vector3<T>& u,
                                       const vector3<T>& v) {
  T phi;
  T h;
  T lambda;
  vector3<T> w;

  w = cross(u, v);
  phi = dot(u, v);
  lambda = dot(w, w);
  if (lambda > nv_eps)
    h = (T(1) - phi) / lambda;
  else
    h = lambda;

  T hxy = w.x * w.y * h;
  T hxz = w.x * w.z * h;
  T hyz = w.y * w.z * h;

  a00 = phi + w.x * w.x * h;
  a01 = hxy - w.z;
  a02 = hxz + w.y;

  a10 = hxy + w.z;
  a11 = phi + w.y * w.y * h;
  a12 = hyz - w.x;

  a20 = hxz - w.y;
  a21 = hyz + w.x;
  a22 = phi + w.z * w.z * h;
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::set_rot(const matrix3<T>& M) {
  // copy the 3x3 rotation block
  a00 = M.a00;
  a10 = M.a10;
  a20 = M.a20;
  a01 = M.a01;
  a11 = M.a11;
  a21 = M.a21;
  a02 = M.a02;
  a12 = M.a12;
  a22 = M.a22;
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::set_scale(const vector3<T>& s) {
  a00 = s.x;
  a11 = s.y;
  a22 = s.z;
  return *this;
}

template <class T>
inline vector3<T>& matrix4<T>::get_scale(vector3<T>& s) const {
  s.x = a00;
  s.y = a11;
  s.z = a22;
  return s;
}

template <class T>
inline matrix4<T>& matrix4<T>::as_scale(const vector3<T>& s) {
  identity();
  a00 = s.x;
  a11 = s.y;
  a22 = s.z;
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::as_scale(const T& s) {
  identity();
  a00 = s;
  a11 = s;
  a22 = s;
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::set_translation(const vector3<T>& t) {
  a03 = t.x;
  a13 = t.y;
  a23 = t.z;
  return *this;
}

template <class T>
inline matrix4<T>& matrix4<T>::as_translation(const vector3<T>& t) {
  identity();
  a03 = t.x;
  a13 = t.y;
  a23 = t.z;
  return *this;
}

template <class T>
inline vector3<T>& matrix4<T>::get_translation(vector3<T>& t) const {
  t.x = a03;
  t.y = a13;
  t.z = a23;
  return t;
}

template <class T>
inline matrix3<T> matrix4<T>::get_rot_mat3() const {
  matrix3<T> M;
  // assign the 3x3 rotation block
  M.a00 = a00;
  M.a10 = a10;
  M.a20 = a20;
  M.a01 = a01;
  M.a11 = a11;
  M.a21 = a21;
  M.a02 = a02;
  M.a12 = a12;
  M.a22 = a22;
  return M;
}

template <class T>
inline quaternion<T> matrix4<T>::get_rot_quat() const {
  quaternion<T> q;
  matrix3<T> m = this->get_rot_mat3();
  q.from_matrix(m);
  return q;
}

template <class T>
inline matrix4<T> negate(const matrix4<T>& M) {
  matrix4<T> N;
  for (int i = 0; i < 16; ++i)
    N.mat_array[i] = -M.mat_array[i];
  return N;
}

template <class T>
inline matrix3<T> negate(const matrix3<T>& M) {
  matrix3<T> N;
  for (int i = 0; i < 9; ++i)
    N.mat_array[i] = -M.mat_array[i];
  return N;
}

template <class T>
inline matrix3<T> tangent_basis(const vector3<T>& v0,
                                const vector3<T>& v1,
                                const vector3<T>& v2,
                                const vector2<T>& t0,
                                const vector2<T>& t1,
                                const vector2<T>& t2,
                                const vector3<T>& n) {
  matrix3<T> basis;
  vector3<T> cp;
  vector3<T> e0(v1.x - v0.x, t1.s - t0.s, t1.t - t0.t);
  vector3<T> e1(v2.x - v0.x, t2.s - t0.s, t2.t - t0.t);

  cp = cross(e0, e1);
  if (fabs(cp.x) > nv_eps) {
    basis.a00 = -cp.y / cp.x;
    basis.a10 = -cp.z / cp.x;
  }

  e0.x = v1.y - v0.y;
  e1.x = v2.y - v0.y;

  cp = cross(e0, e1);
  if (fabs(cp.x) > nv_eps) {
    basis.a01 = -cp.y / cp.x;
    basis.a11 = -cp.z / cp.x;
  }

  e0.x = v1.z - v0.z;
  e1.x = v2.z - v0.z;

  cp = cross(e0, e1);
  if (fabs(cp.x) > nv_eps) {
    basis.a02 = -cp.y / cp.x;
    basis.a12 = -cp.z / cp.x;
  }

  // tangent...
  T oonorm = T(1) / sqrtf(basis.a00 * basis.a00 + basis.a01 * basis.a01 +
                          basis.a02 * basis.a02);
  basis.a00 *= oonorm;
  basis.a01 *= oonorm;
  basis.a02 *= oonorm;

  // binormal...
  oonorm = T(1) / sqrtf(basis.a10 * basis.a10 + basis.a11 * basis.a11 +
                        basis.a12 * basis.a12);
  basis.a10 *= oonorm;
  basis.a11 *= oonorm;
  basis.a12 *= oonorm;

  // normal...
  // compute the cross product TxB
  basis.a20 = basis.a01 * basis.a12 - basis.a02 * basis.a11;
  basis.a21 = basis.a02 * basis.a10 - basis.a00 * basis.a12;
  basis.a22 = basis.a00 * basis.a11 - basis.a01 * basis.a10;

  oonorm = T(1) / sqrtf(basis.a20 * basis.a20 + basis.a21 * basis.a21 +
                        basis.a22 * basis.a22);
  basis.a20 *= oonorm;
  basis.a21 *= oonorm;
  basis.a22 *= oonorm;

  // Gram-Schmidt orthogonalization process for B
  // compute the cross product B=NxT to obtain
  // an orthogonal basis
  basis.a10 = basis.a21 * basis.a02 - basis.a22 * basis.a01;
  basis.a11 = basis.a22 * basis.a00 - basis.a20 * basis.a02;
  basis.a12 = basis.a20 * basis.a01 - basis.a21 * basis.a00;

  if (basis.a20 * n.x + basis.a21 * n.y + basis.a22 * n.z < T(0)) {
    basis.a20 = -basis.a20;
    basis.a21 = -basis.a21;
    basis.a22 = -basis.a22;
  }
  return basis;
}

/*
 * Project an x,y pair onto a sphere of radius r OR a hyperbolic sheet
 * if we are away from the center of the sphere.
 */
template <class T>
inline T tb_project_to_sphere(T r, T x, T y) {
  T d, t, z;

  d = sqrtf(x * x + y * y);
  if (d < r * 0.70710678118654752440) { /* Inside sphere */
    z = sqrtf(r * r - d * d);
  } else { /* On hyperbola */
    t = r / (T)1.41421356237309504880;
    z = t * t / d;
  }
  return z;
}

/*
 * Ok, simulate a track-ball.  Project the points onto the virtual
 * trackball, then figure out the axis of rotation, which is the cross
 * product of P1 P2 and O P1 (O is the center of the ball, 0,0,0)
 * Note:  This is a deformed trackball-- is a trackball in the center,
 * but is deformed into a hyperbolic sheet of rotation away from the
 * center.  This particular function was chosen after trying out
 * several variations.
 *
 * It is assumed that the arguments to this routine are in the range
 * (-1.0 ... 1.0)
 */
template <class T>
inline quaternion<T> trackball(vector2<T>& pt1,
                               vector2<T>& pt2,
                               T trackballsize) {
  quaternion<T> q;
  vector3<T> a;  // Axis of rotation
  float phi;     // how much to rotate about axis
  vector3<T> d;
  float t;

  if (pt1.x == pt2.x && pt1.y == pt2.y) {
    // Zero rotation
    q = quat_id;
    return q;
  }

  // First, figure out z-coordinates for projection of P1 and P2 to
  // deformed sphere
  vector3<T> p1(pt1.x, pt1.y,
                tb_project_to_sphere(trackballsize, pt1.x, pt1.y));
  vector3<T> p2(pt2.x, pt2.y,
                tb_project_to_sphere(trackballsize, pt2.x, pt2.y));

  //  Now, we want the cross product of P1 and P2
  a = cross(p1, p2);

  //  Figure out how much to rotate around that axis.
  d.x = p1.x - p2.x;
  d.y = p1.y - p2.y;
  d.z = p1.z - p2.z;
  t = sqrtf(d.x * d.x + d.y * d.y + d.z * d.z) / (trackballsize);

  // Avoid problems with out-of-control values...

  if (t > T(1))
    t = T(1);
  if (t < -T(1))
    t = -T(1);
  phi = nv_two * T(asin(t));
  axis_to_quat(q, a, phi);
  return q;
}

template <class T>
inline vector3<T> cube_map_normal(int i, int x, int y, int cubesize) {
  vector3<T> v;
  T s, t, sc, tc;
  s = (T(x) + T(0.5)) / T(cubesize);
  t = (T(y) + T(0.5)) / T(cubesize);
  sc = s * nv_two - T(1);
  tc = t * nv_two - T(1);

  switch (i) {
    case 0:
      v.x = T(1);
      v.y = -tc;
      v.z = -sc;
      break;
    case 1:
      v.x = -T(1);
      v.y = -tc;
      v.z = sc;
      break;
    case 2:
      v.x = sc;
      v.y = T(1);
      v.z = tc;
      break;
    case 3:
      v.x = sc;
      v.y = -T(1);
      v.z = -tc;
      break;
    case 4:
      v.x = sc;
      v.y = -tc;
      v.z = T(1);
      break;
    case 5:
      v.x = -sc;
      v.y = -tc;
      v.z = -T(1);
      break;
  }
  v.normalize();
  return v;
}

// computes the area of a triangle
template <class T>
inline T nv_area(const vector3<T>& v1,
                 const vector3<T>& v2,
                 const vector3<T>& v3) {
  vector3<T> cp_sum;
  vector3<T> cp;
  cp_sum = cross(v1, v2);
  cp_sum += cross(v2, v3);
  cp_sum += cross(v3, v1);
  return nv_norm(cp_sum) * T(0.5);
}

// computes the perimeter of a triangle
template <class T>
inline T nv_perimeter(const vector3<T>& v1,
                      const vector3<T>& v2,
                      const vector3<T>& v3) {
  T perim;
  vector3<T> diff;
  diff = sub(v1, v2);
  perim = nv_norm(diff);
  diff = sub(v2, v3);
  perim += nv_norm(diff);
  diff = sub(v3, v1);
  perim += nv_norm(diff);
  return perim;
}

// compute the center and radius of the inscribed circle defined by the three
// vertices
template <class T>
inline T nv_find_in_circle(vector3<T>& center,
                           const vector3<T>& v1,
                           const vector3<T>& v2,
                           const vector3<T>& v3) {
  T area = nv_area(v1, v2, v3);
  // if the area is null
  if (area < nv_eps) {
    center = v1;
    return T(0);
  }

  T oo_perim = T(1) / nv_perimeter(v1, v2, v3);

  vector3<T> diff;

  diff = sub(v2, v3);
  mult<T>(center, v1, nv_norm(diff));

  diff = sub(v3, v1);
  center = madd(v2, nv_norm(diff));

  diff = sub(v1, v2);
  center = madd(v3, nv_norm(diff));

  center *= oo_perim;

  return nv_two * area * oo_perim;
}

// compute the center and radius of the circumscribed circle defined by the
// three vertices i.e. the osculating circle of the three vertices
template <class T>
inline T nv_find_circ_circle(vector3<T>& center,
                             const vector3<T>& v1,
                             const vector3<T>& v2,
                             const vector3<T>& v3) {
  vector3<T> e0;
  vector3<T> e1;
  T d1, d2, d3;
  T c1, c2, c3, oo_c;

  e0 = sub(v3, v1);
  e1 = sub(v2, v1);
  d1 = dot(e0, e1);

  e0 = sub(v3, v2);
  e1 = sub(v1, v2);
  d2 = dot(e0, e1);

  e0 = sub(v1, v3);
  e1 = sub(v2, v3);
  d3 = dot(e0, e1);

  c1 = d2 * d3;
  c2 = d3 * d1;
  c3 = d1 * d2;
  oo_c = T(1) / (c1 + c2 + c3);

  mult<T>(center, v1, c2 + c3);
  center = madd(v2, c3 + c1);
  center = madd(v3, c1 + c2);
  center *= oo_c * T(0.5);

  return T(0.5) * sqrtf((d1 + d2) * (d2 + d3) * (d3 + d1) * oo_c);
}

template <class T>
inline T ffast_cos(const T x) {
  // assert:  0 <= fT <= PI/2
  // maximum absolute error = 1.1880e-03
  // speedup = 2.14

  T x_sqr = x * x;
  T res = T(3.705e-02);
  res *= x_sqr;
  res -= T(4.967e-01);
  res *= x_sqr;
  res += T(1);
  return res;
}

template <class T>
inline T fast_cos(const T x) {
  // assert:  0 <= fT <= PI/2
  // maximum absolute error = 2.3082e-09
  // speedup = 1.47

  T x_sqr = x * x;
  T res = T(-2.605e-07);
  res *= x_sqr;
  res += T(2.47609e-05);
  res *= x_sqr;
  res -= T(1.3888397e-03);
  res *= x_sqr;
  res += T(4.16666418e-02);
  res *= x_sqr;
  res -= T(4.999999963e-01);
  res *= x_sqr;
  res += T(1);
  return res;
}

template <class T>
inline void nv_is_valid(const vector3<T>& v) {
#ifdef WIN32
  assert(!_isnan(v.x) && !_isnan(v.y) && !_isnan(v.z) && _finite(v.x) &&
         _finite(v.y) && _finite(v.z));
#else
  assert(!isnan(v.x) && !isnan(v.y) && !isnan(v.z));
#endif
}

template <class T>
void nv_is_valid(T lambda) {
#ifndef WIN32

  assert(!_isnan(lambda));
#else
  assert(!_isnan(lambda) && _finite(lambda));
#endif
}

// TL
template <class T>
inline T get_angle(const vector3<T>& v1, const vector3<T>& v2) {
  float dp = dot(v1, v2);
  if (dp > 1.0f)
    dp = 1.0f;
  else if (dp < -1.0f)
    dp = -1.0f;
  return acosf(dp);
}

template <class T>
inline vector3<T> rotate_by(const vector3<T>& src, const quaternion<T>& q) {
  return mult<T>(quat_2_mat(q), src);
}

// TL
template <class T>
inline void quaternion<T>::to_euler_xyz(vector3<T>& r) {
  double a = 2.0f * (w * x + y * z);
  double b = 1.0 - 2.0f * (x * x + y * y);
  r.x = (T)atan2(a, b);

  a = 2.0 * (w * y - z * x);
  r.y = (T)asin(a);

  a = 2.0 * (w * z + x * y);
  b = 1.0 - 2.0 * (y * y + z * z);
  r.z = (T)atan2(a, b);
}

template <class T>
inline void quaternion<T>::to_euler_xyz(T* r) {
  double a = 2.0f * (w * x + y * z);
  double b = 1.0 - 2.0f * (x * x + y * y);
  r[0] = (T)atan2(a, b);

  a = 2.0 * (w * y - z * x);
  r[1] = (T)asin(a);

  a = 2.0 * (w * z + x * y);
  b = 1.0 - 2.0 * (y * y + z * z);
  r[2] = (T)atan2(a, b);
}

template <class T>
inline quaternion<T>::quaternion(const vector3<T>& eulerXYZ) {
  from_euler_xyz(eulerXYZ);
}

template <class T>
inline void quaternion<T>::from_euler_xyz(vector3<T> r) {
  r *= 0.5f;
  w = cosf(r.x) * cosf(r.y) * cosf(r.z) + sinf(r.x) * sinf(r.y) * sinf(r.z);
  x = sinf(r.x) * cosf(r.y) * cosf(r.z) - cosf(r.x) * sinf(r.y) * sinf(r.z);
  y = cosf(r.x) * sinf(r.y) * cosf(r.z) + sinf(r.x) * cosf(r.y) * sinf(r.z);
  z = cosf(r.x) * cosf(r.y) * sinf(r.z) - sinf(r.x) * sinf(r.y) * cosf(r.z);
}

template <class T>
inline matrix4<T> rotation_yaw_pitch_roll(const T yaw,
                                          const T pitch,
                                          const T roll) {
  matrix4<T> M;
  M.identity();
  matrix4<T> rot;

  if (roll) {
    M.rotate(roll, vector3<T>(0, 0, 1));
  }
  if (pitch) {
    M.rotate(pitch, vector3<T>(1, 0, 0));
  }
  if (yaw) {
    M.rotate(yaw, vector3<T>(0, 1, 0));
  }

  return M;
}

template <class T>
inline vector2<T> nv_max(const vector2<T>& vFirst, const vector2<T>& vSecond) {
  vector2<T> vOut;
  vOut.x = nv_max(vFirst.x, vSecond.x);
  vOut.y = nv_max(vFirst.y, vSecond.y);
  return vOut;
}

template <class T>
inline vector2<T> nv_min(const vector2<T>& vFirst, const vector2<T>& vSecond) {
  vector2<T> vOut;
  vOut.x = nv_min(vFirst.x, vSecond.x);
  vOut.y = nv_min(vFirst.y, vSecond.y);
  return vOut;
}

template <class T>
inline vector3<T> nv_max(const vector3<T>& vFirst, const vector3<T>& vSecond) {
  vector3<T> vOut;
  vOut.x = nv_max(vFirst.x, vSecond.x);
  vOut.y = nv_max(vFirst.y, vSecond.y);
  vOut.z = nv_max(vFirst.z, vSecond.z);
  return vOut;
}

template <class T>
inline vector3<T> nv_min(const vector3<T>& vFirst, const vector3<T>& vSecond) {
  vector3<T> vOut;
  vOut.x = nv_min(vFirst.x, vSecond.x);
  vOut.y = nv_min(vFirst.y, vSecond.y);
  vOut.z = nv_min(vFirst.z, vSecond.z);
  return vOut;
}

template <class T>
inline vector4<T> nv_max(const vector4<T>& vFirst, const vector4<T>& vSecond) {
  vector4<T> vOut;
  vOut.x = nv_max(vFirst.x, vSecond.x);
  vOut.y = nv_max(vFirst.y, vSecond.y);
  vOut.z = nv_max(vFirst.z, vSecond.z);
  vOut.w = nv_max(vFirst.w, vSecond.w);
  return vOut;
}

template <class T>
inline vector4<T> nv_min(const vector4<T>& vFirst, const vector4<T>& vSecond) {
  vector4<T> vOut;
  vOut.x = nv_min(vFirst.x, vSecond.x);
  vOut.y = nv_min(vFirst.y, vSecond.y);
  vOut.z = nv_min(vFirst.z, vSecond.z);
  vOut.w = nv_min(vFirst.w, vSecond.w);
  return vOut;
}

template <class T>
inline vector2<T> nv_clamp(const vector2<T>& u, const T min, const T max) {
  vector2<T> vOut;
  vOut.x = nv_clamp(u.x, min, max);
  vOut.y = nv_clamp(u.y, min, max);
  return vOut;
}

template <class T>
inline vector3<T> nv_clamp(const vector3<T>& u, const T min, const T max) {
  vector3<T> vOut;
  vOut.x = nv_clamp(u.x, min, max);
  vOut.y = nv_clamp(u.y, min, max);
  vOut.z = nv_clamp(u.z, min, max);
  return vOut;
}

template <class T>
inline vector4<T> nv_clamp(const vector4<T>& u, const T min, const T max) {
  vector4<T> vOut;
  vOut.x = nv_clamp(u.x, min, max);
  vOut.y = nv_clamp(u.y, min, max);
  vOut.z = nv_clamp(u.z, min, max);
  vOut.w = nv_clamp(u.w, min, max);
  return vOut;
}

template <class T>
inline T nv_floor(T u) {
  return std::floor(u);
}

template <class T>
inline vector2<T> nv_floor(const vector2<T>& u) {
  vector2<T> vOut;
  vOut.x = std::floor(u.x);
  vOut.y = std::floor(u.y);
  return vOut;
}

template <class T>
inline vector3<T> nv_floor(const vector3<T>& u) {
  vector3<T> vOut;
  vOut.x = std::floor(u.x);
  vOut.y = std::floor(u.y);
  vOut.z = std::floor(u.z);
  return vOut;
}

template <class T>
inline vector4<T> nv_floor(const vector4<T>& u) {
  vector4<T> vOut;
  vOut.x = std::floor(u.x);
  vOut.y = std::floor(u.y);
  vOut.z = std::floor(u.z);
  vOut.w = std::floor(u.w);
  return vOut;
}

template <class T>
inline T nv_min(const T& lambda, const T& n) {
  return (lambda < n) ? lambda : n;
}

template <class T>
inline T nv_max(const T& lambda, const T& n) {
  return (lambda > n) ? lambda : n;
}

template <class T>
inline T nv_clamp(const T u, const T min, const T max) {
  T o = (u < min) ? min : u;
  o = (o > max) ? max : o;
  return o;
}

#ifdef NVP_SUPPORTS_OPTIX
#pragma message( \
    "**WARNING** nvmath.h : Canceling the lerp() function here : already declared in OptiX")
#else
template <class T>
inline T lerp(T t, T a, T b) {
  return a * (T(1) - t) + t * b;
}

template <class T>
inline vector2<T> lerp(const T& t, const vector2<T>& u, const vector2<T>& v) {
  vector2<T> w;
  w.x = lerp(t, u.x, v.x);
  w.y = lerp(t, u.y, v.y);
  return w;
}
template <class T>
inline vector3<T> lerp(const T& t, const vector3<T>& u, const vector3<T>& v) {
  vector3<T> w;
  w.x = lerp(t, u.x, v.x);
  w.y = lerp(t, u.y, v.y);
  w.z = lerp(t, u.z, v.z);
  return w;
}

template <class T>
inline vector4<T> lerp(const T& t, const vector4<T>& u, const vector4<T>& v) {
  vector4<T> w;
  w.x = lerp(t, u.x, v.x);
  w.y = lerp(t, u.y, v.y);
  w.z = lerp(t, u.z, v.z);
  w.w = lerp(t, u.w, v.w);
  return w;
}
#endif

// Computes the squared magnitude
template <class T>
inline T nv_sq_norm(const vector2<T>& n) {
  return n.x * n.x + n.y * n.y;
}

template <class T>
inline T nv_sq_norm(const vector3<T>& n) {
  return n.x * n.x + n.y * n.y + n.z * n.z;
}

template <class T>
inline T nv_sq_norm(const vector4<T>& n) {
  return n.x * n.x + n.y * n.y + n.z * n.z + n.w * n.w;
}

// Computes the magnitude
template <class T>
inline T nv_norm(const vector2<T>& n) {
  return sqrtf(nv_sq_norm(n));
}

template <class T>
inline T nv_norm(const vector3<T>& n) {
  return sqrtf(nv_sq_norm(n));
}

template <class T>
inline T nv_norm(const vector4<T>& n) {
  return sqrtf(nv_sq_norm(n));
}

template <class T>
inline T length(const vector2<T>& n) {
  return n.norm();
}

template <class T>
inline T length(const vector3<T>& n) {
  return n.norm();
}

template <class T>
inline T length(const vector4<T>& n) {
  return n.norm();
}

template <class T>
inline T nv_abs(const T u) {
  return T(std::abs(u));
}
template <class T>
inline vector2<T> nv_abs(const vector2<T>& u) {
  return vector2<T>(T(std::abs(u.x)), T(std::abs(u.y)));
}
template <class T>
inline vector3<T> nv_abs(const vector3<T>& u) {
  return vector3<T>(T(std::abs(u.x)), T(std::abs(u.y)), T(std::abs(u.z)));
}
template <class T>
inline vector4<T> nv_abs(const vector4<T>& u) {
  return vector4<T>(T(std::abs(u.x)), T(std::abs(u.y)), T(std::abs(u.z)),
                    T(std::abs(u.w)));
}

template <class T>
T smoothstep(T edge0, T edge1, T x) {
  T const tmp(nv_clamp((x - edge0) / (edge1 - edge0), T(0), T(1)));
  return tmp * tmp * (T(3) - T(2) * tmp);
}

template <typename T>
vector2<T> make_vec2(T const* const ptr) {
  vector2<T> Result;
  memcpy(&Result.x, ptr, sizeof(vector2<T>));
  return Result;
}
template <typename T>
vector3<T> make_vec3(T const* const ptr) {
  vector3<T> Result;
  memcpy(&Result.x, ptr, sizeof(vector3<T>));
  return Result;
}
template <typename T>
vector4<T> make_vec4(T const* const ptr) {
  vector4<T> Result;
  memcpy(&Result.x, ptr, sizeof(vector4<T>));
  return Result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
inline PlaneSide point_on_planeside(const vector3<T>& vec, const plane<T>& p) {
  float d = dot(vec, p.normal()) + p.w;
  if (d > nv_eps)
    return PLANESIDE_FRONT;
  if (d < -nv_eps)
    return PLANESIDE_BACK;
  return PLANESIDE_ON_PLANE;
}

template <typename T>
inline float point_distance(const vector3<T>& vec, const plane<T>& p) {
  return dot(vector4<T>(vec, T(1.0)), p);
}

template <typename T>
inline vector3<T> project_point_on_plane(const vector3<T>& point,
                                         const plane<T>& p) {
  return point - p.normal() * point_distance(point, p);
}

template <typename T>
inline void normalize_plane(plane<T>& p) {
  float inv_length = 1.0f / length(p.normal());
  p *= inv_length;
}

template <typename T>
inline plane<T>::plane(const vector4<T>& v,
                       NormalizeInConstruction normalizePlane)
    : vector4<T>(v) {
  if (normalizePlane) {
    normalize_plane(*this);
  }
};

template <typename T>
inline plane<T>::plane(const vector3<T>& normal,
                       const vector3<T>& point,
                       NormalizeInConstruction normalizePlane) {
  vector3<T> n = normalizePlane ? normalize(normal) : normal;

  this->x = n.x;
  this->y = n.y;
  this->z = n.z;
  this->w = -(dot(n, point));
};

template <typename T>
inline plane<T>::plane(const vector3<T>& normal,
                       const float dist,
                       NormalizeInConstruction normalizePlane)
    : vector4<T>(normal, -dist) {
  // re-normalize immediately
  if (normalizePlane) {
    normalize_plane(*this);
  }
};

template <typename T>
inline plane<T>::plane(const vector3<T>& v1,
                       const vector3<T>& v2,
                       const vector3<T>& v3,
                       NormalizeInConstruction normalizePlane)
    : plane<T>(cross((v2 - v1), (v3 - v1)), v1, normalizePlane) {}

template <typename T>
inline plane<T>::plane(T a,
                       T b,
                       T c,
                       T d,
                       NormalizeInConstruction normalizePlane)
    : vector4<T>(a, b, c, d) {
  if (normalizePlane) {
    normalize_plane(*this);
  }
}

template <typename T>
inline plane<T> plane<T>::operator-() {
  return plane<T>(vector4<T>::operator-());
}

template <typename T>
vector3<T> get_perpendicular_vec(const vector3<T>& vec) {
  vector3<T> perp;

  perp.x = std::abs(vec.x);
  perp.y = std::abs(vec.y);
  perp.z = std::abs(vec.z);

  // choose a basis vector roughly along the smallest component of the vector
  if (perp.x <= perp.y && perp.x <= perp.z) {
    perp = vector3<T>(1.0f, 0, 0);
  } else {
    if (perp.y <= perp.x && perp.y <= perp.z) {
      perp = vector3<T>(0, 1.0f, 0);
    } else {
      perp = vector3<T>(0, 0, 1.0f);
    }
  }

  perp =
      perp -
      (vec * (dot(perp, vec)));  // lay perp into the plane perpendicular to vec
  return perp;
}

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

}  // namespace nvmath
