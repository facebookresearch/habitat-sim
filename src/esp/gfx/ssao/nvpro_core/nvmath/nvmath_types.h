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

#ifndef _nvmathtypes_h_
#define _nvmathtypes_h_

#include <assert.h>
#include <cmath>

#ifdef _WIN32
#include <limits>
#else
#include <limits.h>
#endif

#ifdef MACOS
#define sqrtf sqrt
#define sinf sin
#define cosf cos
#define tanf tan
#endif

#include <float.h>
#include <memory.h>
#include <stdlib.h>

#ifdef NVMATH_SUPPORTS_GLM
#include <glm/glm.hpp>
#endif

namespace nvmath {

typedef float nv_scalar;

#define nv_zero nvmath::nv_scalar(0)
#define nv_one_half nvmath::nv_scalar(0.5)
#define nv_one nvmath::nv_scalar(1.0)
#define nv_two nvmath::nv_scalar(2)
#define nv_half_pi                                                            \
  nvmath::nv_scalar(3.14159265358979323846264338327950288419716939937510582 * \
                    0.5)
#define nv_quarter_pi                                                         \
  nvmath::nv_scalar(3.14159265358979323846264338327950288419716939937510582 * \
                    0.25)
#define nv_pi \
  nvmath::nv_scalar(3.14159265358979323846264338327950288419716939937510582)
#define nv_two_pi                                                             \
  nvmath::nv_scalar(3.14159265358979323846264338327950288419716939937510582 * \
                    2.0)
#define nv_oo_pi (nv_one / nv_pi)
#define nv_oo_two_pi (nv_one / nv_two_pi)
#define nv_oo_255 (nv_one / nvmath::nv_scalar(255))
#define nv_oo_128 (nv_one / nvmath::nv_scalar(128))
#define nv_to_rad (nv_pi / nvmath::nv_scalar(180))
#define nv_to_deg (nvmath::nv_scalar(180) / nv_pi)
#define rad2deg(a) ((a)*nv_to_deg)
#define deg2rad(a) ((a)*nv_to_rad)
#define nv_eps nvmath::nv_scalar(10e-6)
#define nv_double_eps (nvmath::nv_scalar(10e-6) * nv_two)
#define nv_big_eps nvmath::nv_scalar(10e-2)
#define nv_small_eps nvmath::nv_scalar(10e-6)
#define nv_sqrthalf nvmath::nv_scalar(0.7071067811865475244)

#define nv_scalar_max nvmath::nv_scalar(FLT_MAX)
#define nv_scalar_min nvmath::nv_scalar(FLT_MIN)

template <class T>
struct vector2;
template <class T>
struct vector3;
template <class T>
struct vector4;
template <class T>
struct matrix3;
template <class T>
struct matrix4;
template <class T>
struct quaternion;

template <class T>
struct vector2 {
  vector2() : vector2(0, 0) {}
  vector2(T x) : x(T(x)), y(T(x)) {}
  template <typename T0, typename T1>
  vector2(T0 x, T1 y) : x(T(x)), y(T(y)) {}
  vector2(const T* xy) : x(xy[0]), y(xy[1]) {}
  vector2(const vector2& u) = default;
  explicit vector2(const vector3<T>&);
  explicit vector2(const vector4<T>&);

  bool operator==(const vector2& u) const {
    return (u.x == x && u.y == y) ? true : false;
  }

  bool operator!=(const vector2& u) const { return !(*this == u); }

  vector2& operator*=(const T& lambda) {
    x *= lambda;
    y *= lambda;
    return *this;
  }
  // TL
  vector2& operator/=(const T& lambda) {
    x /= lambda;
    y /= lambda;
    return *this;
  }

  vector2& operator-=(const vector2& u) {
    x -= u.x;
    y -= u.y;
    return *this;
  }

  vector2& operator+=(const vector2& u) {
    x += u.x;
    y += u.y;
    return *this;
  }

  T& operator[](int i) { return vec_array[i]; }

  const T& operator[](int i) const { return vec_array[i]; }

  T sq_norm() const { return x * x + y * y; }
  T norm() const { return sqrtf(sq_norm()); }

  union {
    struct {
      T x, y;  // standard names for components
    };
    T vec_array[2];  // array access
  };

  T* get_value() { return vec_array; }
  const T* get_value() const { return vec_array; }

#ifdef NVMATH_SUPPORTS_GLM
  vector2(const glm::vec2& f) {
    x = f.x;
    y = f.y;
  }
  operator glm::vec2() const { return glm::vec2(x, y); }
#endif
};

template <class T>
struct vector3 {
  vector3() : x(T(0)), y(T(0)), z(T(0)) {}
  vector3(T x) : x(T(x)), y(T(x)), z(T(x)) {}
  template <typename T0, typename T1, typename T2>
  vector3(T0 x, T1 y, T2 z) : x(T(x)), y(T(y)), z(T(z)) {}
  vector3(const T* xyz) : x(xyz[0]), y(xyz[1]), z(xyz[2]) {}
  explicit vector3(const vector2<T>& u) : x(u.x), y(u.y), z(1.0f) {}
  vector3(const vector2<T>& u, T v) : x(u.x), y(u.y), z(v) {}
  vector3(const vector3<T>& u) = default;

  template <typename T2>
  explicit vector3(const vector3<T2>& u) : x(u.x), y(u.y), z(u.z) {}

  explicit vector3(const vector4<T>&);

  bool operator==(const vector3<T>& u) const {
    return (u.x == x && u.y == y && u.z == z) ? true : false;
  }

  bool operator!=(const vector3<T>& rhs) const { return !(*this == rhs); }

  // TL
  vector3<T>& operator*=(const matrix3<T>& M) {
    vector3<T> dst;
    mult(dst, M, *this);
    x = dst.x;
    y = dst.y;
    z = dst.z;
    return (*this);
  }
  // TL
  vector3<T>& operator*=(const matrix4<T>& M) {
    vector3<T> dst;
    mult(dst, M, *this);
    x = dst.x;
    y = dst.y;
    z = dst.z;
    return (*this);
  }
  // TL
  vector3<T>& operator/=(const vector3<T>& d) {
    x /= d.x;
    y /= d.y;
    z /= d.z;
    return *this;
  }
  vector3<T>& operator/=(const T& lambda) {
    x /= lambda;
    y /= lambda;
    z /= lambda;
    return *this;
  }

  vector3<T>& operator*=(const T& lambda) {
    x *= lambda;
    y *= lambda;
    z *= lambda;
    return *this;
  }
  // TL
  vector3<T>& operator*=(const vector3<T>& v) {
    x *= v.x;
    y *= v.y;
    z *= v.z;
    return *this;
  }

  vector3<T> operator-() const { return vector3<T>(-x, -y, -z); }

  vector3<T>& operator-=(const vector3<T>& u) {
    x -= u.x;
    y -= u.y;
    z -= u.z;
    return *this;
  }

  vector3<T>& operator+=(const vector3<T>& u) {
    x += u.x;
    y += u.y;
    z += u.z;
    return *this;
  }
  vector3<T>& rotateBy(const quaternion<T>& q);
  T normalize();
  void orthogonalize(const vector3<T>& v);
  void orthonormalize(const vector3<T>& v) {
    orthogonalize(v);  //  just orthogonalize...
    normalize();       //  ...and normalize it
  }

  T sq_norm() const { return x * x + y * y + z * z; }
  T norm() const { return sqrtf(sq_norm()); }

  T& operator[](int i) { return vec_array[i]; }

  const T& operator[](int i) const { return vec_array[i]; }

  union {
    struct {
      T x, y, z;  // standard names for components
    };
    T vec_array[3];  // array access
  };

  T* get_value() { return vec_array; }
  const T* get_value() const { return vec_array; }

#ifdef NVMATH_SUPPORTS_GLM
  vector3(const glm::vec3& f) {
    x = f.x;
    y = f.y;
    z = f.z;
  }
  operator glm::vec3() const { return glm::vec3(x, y, z); }
#endif
};

template <class T>
inline vector2<T>::vector2(const vector3<T>& u) {
  x = u.x;
  y = u.y;
}

template <class T>
struct vector4 {
  vector4() : x(T(0)), y(T(0)), z(T(0)), w(T(0)) {}
  vector4(T x) : x(T(x)), y(T(x)), z(T(x)), w(T(x)) {}
  template <typename T0, typename T1, typename T2, typename T3>
  vector4(T0 x, T1 y, T2 z, T3 w) : x(T(x)), y(T(y)), z(T(z)), w(T(w)) {}
  vector4(const T* xyzw) : x(xyzw[0]), y(xyzw[1]), z(xyzw[2]), w(xyzw[3]) {}
  explicit vector4(const vector2<T>& u) : x(u.x), y(u.y), z(0.0f), w(1.0f) {}
  explicit vector4(const vector2<T>& u, const T zz)
      : x(u.x), y(u.y), z(zz), w(1.0f) {}
  vector4(const vector2<T>& u, const T zz, const T ww)
      : x(u.x), y(u.y), z(zz), w(ww) {}
  explicit vector4(const vector3<T>& u) : x(u.x), y(u.y), z(u.z), w(1.0f) {}
  vector4(const vector3<T>& u, const T w) : x(u.x), y(u.y), z(u.z), w(w) {}
  vector4(const vector4<T>& u) = default;

  bool operator==(const vector4<T>& u) const {
    return (u.x == x && u.y == y && u.z == z && u.w == w) ? true : false;
  }

  bool operator!=(const vector4<T>& rhs) const { return !(*this == rhs); }

  T sq_norm() const { return x * x + y * y + z * z + w * w; }
  T norm() const { return sqrtf(sq_norm()); }

  // TL
  vector4<T>& operator*=(const matrix3<T>& M) {
    vector4<T> dst;
    mult(dst, M, *this);
    x = dst.x;
    y = dst.y;
    z = dst.z;
    w = dst.z;
    return (*this);
  }
  // TL
  vector4<T>& operator*=(const matrix4<T>& M) {
    vector4<T> dst;
    mult(dst, M, *this);
    x = dst.x;
    y = dst.y;
    z = dst.z;
    w = dst.w;
    return (*this);
  }
  // TL
  vector4<T>& operator/=(const T& lambda) {
    x /= lambda;
    y /= lambda;
    z /= lambda;
    w /= lambda;
    return *this;
  }

  vector4<T>& operator*=(const T& lambda) {
    x *= lambda;
    y *= lambda;
    z *= lambda;
    w *= lambda;
    return *this;
  }

  vector4<T>& operator-=(const vector4<T>& u) {
    x -= u.x;
    y -= u.y;
    z -= u.z;
    w -= u.w;
    return *this;
  }

  vector4<T>& operator+=(const vector4<T>& u) {
    x += u.x;
    y += u.y;
    z += u.z;
    w += u.w;
    return *this;
  }

  vector4<T> operator-() const { return vector4<T>(-x, -y, -z, -w); }

  T& operator[](int i) { return vec_array[i]; }

  const T& operator[](int i) const { return vec_array[i]; }

  union {
    struct {
      T x, y, z, w;  // standard names for components
    };
    T vec_array[4];  // array access
  };

  T* get_value() { return vec_array; }
  const T* get_value() const { return vec_array; }

#ifdef NVMATH_SUPPORTS_GLM
  vector4(const glm::vec4& f) {
    x = f.x;
    y = f.y;
    z = f.z;
    w = f.w;
  }
  operator glm::vec4() const { return glm::vec4(x, y, z, w); }
#endif
};  // struct vector4

template <class T>
inline vector2<T>::vector2(const vector4<T>& u) {
  x = u.x;
  y = u.y;
}

template <class T>
inline vector3<T>::vector3(const vector4<T>& u) {
  x = u.x;
  y = u.y;
  z = u.z;
}

// quaternion
template <class T>
struct quaternion;

/*
    for all the matrices...a<x><y> indicates the element at row x, col y

    For example:
    a01 <-> row 0, col 1
*/

template <class T>
struct matrix3 {
  matrix3() : matrix3(0, 0, 0, 0, 0, 0, 0, 0, 0) {}
  matrix3(int one) { identity(); }
  matrix3(const T* array) { memcpy(mat_array, array, sizeof(T) * 9); }
  matrix3(const matrix3<T>& M) = default;
  matrix3(const T& f0,
          const T& f1,
          const T& f2,
          const T& f3,
          const T& f4,
          const T& f5,
          const T& f6,
          const T& f7,
          const T& f8)
      : a00(f0),
        a10(f1),
        a20(f2),
        a01(f3),
        a11(f4),
        a21(f5),
        a02(f6),
        a12(f7),
        a22(f8) {}

  matrix3<T>& identity() {
    mat_array[0] = T(1);
    mat_array[1] = T(0);
    mat_array[2] = T(0);
    mat_array[3] = T(0);
    mat_array[4] = T(1);
    mat_array[5] = T(0);
    mat_array[6] = T(0);
    mat_array[7] = T(0);
    mat_array[8] = T(1);

    return *this;
  }

  const vector3<T> col(const int i) const {
    return vector3<T>(&mat_array[i * 3]);
  }

  const vector3<T> row(const int i) const {
    return vector3<T>(mat_array[i], mat_array[i + 3], mat_array[i + 6]);
  }

  const vector3<T> operator[](int i) const {
    return vector3<T>(mat_array[i], mat_array[i + 3], mat_array[i + 6]);
  }

  const T& operator()(const int& i, const int& j) const {
    return mat_array[j * 3 + i];
  }

  T& operator()(const int& i, const int& j) { return mat_array[j * 3 + i]; }

  matrix3<T>& operator*=(const T& lambda) {
    for (int i = 0; i < 9; ++i)
      mat_array[i] *= lambda;
    return *this;
  }

  matrix3<T> operator*(const matrix3<T>&) const;

  matrix3<T>& operator*=(const matrix3<T>& M) {
    *this = mult(*this, M);
    return *this;
  }

  matrix3<T>& operator-=(const matrix3<T>& M) {
    for (int i = 0; i < 9; ++i)
      mat_array[i] -= M.mat_array[i];
    return *this;
  }

  matrix3<T>& set_row(int i, const vector3<T>& v) {
    mat_array[i] = v.x;
    mat_array[i + 3] = v.y;
    mat_array[i + 6] = v.z;
    return *this;
  }

  matrix3<T>& set_col(int i, const vector3<T>& v) {
    mat_array[i * 3] = v.x;
    mat_array[i * 3 + 1] = v.y;
    mat_array[i * 3 + 2] = v.z;
    return *this;
  }

  matrix3<T>& set_rot(const T& theta, const vector3<T>& v);
  matrix3<T>& set_rot(const vector3<T>& u, const vector3<T>& v);

  // Matrix norms...
  // Compute || M ||
  //                1
  T norm_one();

  // Compute || M ||
  //                +inf
  T norm_inf();

  union {
    struct {
      T a00, a10, a20;  // standard names for components
      T a01, a11, a21;  // standard names for components
      T a02, a12, a22;  // standard names for components
    };
    T mat_array[9];  // array access
  };

  T* get_value() { return mat_array; }
  const T* get_value() const { return mat_array; }

#ifdef NVMATH_SUPPORTS_GLM
  matrix3(const glm::mat3& f) {
    mat_array[0] = f[0].x;
    mat_array[1] = f[0].y;
    mat_array[2] = f[0].z;
    mat_array[3] = f[1].x;
    mat_array[4] = f[1].y;
    mat_array[5] = f[1].z;
    mat_array[6] = f[2].x;
    mat_array[7] = f[2].y;
    mat_array[8] = f[2].z;
  }
  operator glm::mat3() const {
    return glm::mat3(mat_array[0], mat_array[1], mat_array[2], mat_array[3],
                     mat_array[4], mat_array[5], mat_array[6], mat_array[7],
                     mat_array[8]);
  }
#endif
};  // struct matrix3

//
// Note : as_...() means that the whole matrix is being modified
// set_...() only changes the concerned fields of the matrix
//
// translate()/scale()/identity()/rotate() : act as OpenGL functions. Example :
// M.identity()
// M.translate(t)
// M.scale(s)
// drawMyVertex(vtx)
//
// is like : newVtx = Midentiry * Mt * Ms * vtx
//
template <class T>
struct matrix4 {
  matrix4() { memset(mat_array, 0, sizeof(T) * 16); }
  matrix4(int one) { identity(); }

  matrix4(const T* array) { memcpy(mat_array, array, sizeof(T) * 16); }
  explicit matrix4(const matrix3<T>& M) {
    memcpy(mat_array, M.mat_array, sizeof(T) * 3);
    mat_array[3] = 0.0;
    memcpy(mat_array + 4, M.mat_array + 3, sizeof(T) * 3);
    mat_array[7] = 0.0;
    memcpy(mat_array + 8, M.mat_array + 6, sizeof(T) * 3);
    mat_array[11] = 0.0f;
    mat_array[12] = 0.0f;
    mat_array[13] = 0.0f;
    mat_array[14] = 0.0f;
    mat_array[15] = 1.0f;
  }
  matrix4(const matrix4<T>& M) = default;

  matrix4(const T& f0,
          const T& f1,
          const T& f2,
          const T& f3,
          const T& f4,
          const T& f5,
          const T& f6,
          const T& f7,
          const T& f8,
          const T& f9,
          const T& f10,
          const T& f11,
          const T& f12,
          const T& f13,
          const T& f14,
          const T& f15)
      : a00(f0),
        a10(f1),
        a20(f2),
        a30(f3),
        a01(f4),
        a11(f5),
        a21(f6),
        a31(f7),
        a02(f8),
        a12(f9),
        a22(f10),
        a32(f11),
        a03(f12),
        a13(f13),
        a23(f14),
        a33(f15) {}

  template <typename T2>
  explicit matrix4(const matrix4<T2>& M) {
    for (int i = 0; i < 16; ++i) {
      mat_array[i] = static_cast<T>(M.mat_array[i]);
    }
  }

  const vector4<T> col(const int i) const {
    return vector4<T>(&mat_array[i * 4]);
  }

  const vector4<T> row(const int i) const {
    return vector4<T>(mat_array[i], mat_array[i + 4], mat_array[i + 8],
                      mat_array[i + 12]);
  }

  const vector4<T> operator[](const int& i) const {
    return vector4<T>(mat_array[i], mat_array[i + 4], mat_array[i + 8],
                      mat_array[i + 12]);
  }

  const T& operator()(const int& i, const int& j) const {
    return mat_array[j * 4 + i];
  }

  T& operator()(const int& i, const int& j) { return mat_array[j * 4 + i]; }

  matrix4<T>& set_col(int i, const vector4<T>& v) {
    mat_array[i * 4] = v.x;
    mat_array[i * 4 + 1] = v.y;
    mat_array[i * 4 + 2] = v.z;
    mat_array[i * 4 + 3] = v.w;
    return *this;
  }

  matrix4<T>& set_row(int i, const vector4<T>& v) {
    mat_array[i] = v.x;
    mat_array[i + 4] = v.y;
    mat_array[i + 8] = v.z;
    mat_array[i + 12] = v.w;
    return *this;
  }

  matrix3<T> get_rot_mat3() const;
  quaternion<T> get_rot_quat() const;
  matrix4<T>& set_rot(const quaternion<T>& q);
  matrix4<T>& set_rot(const matrix3<T>& M);
  matrix4<T>& set_rot(const T& theta, const vector3<T>& v);
  matrix4<T>& set_rot(const vector3<T>& u, const vector3<T>& v);

  matrix4<T>& as_rot(const quaternion<T>& q) {
    a30 = a31 = a32 = 0.0;
    a33 = 1.0;
    a03 = a13 = a23 = 0.0;
    set_rot(q);
    return *this;
  }
  matrix4<T>& as_rot(const matrix3<T>& M) {
    a30 = a31 = a32 = 0.0;
    a33 = 1.0;
    a03 = a13 = a23 = 0.0;
    set_rot(M);
    return *this;
  }
  matrix4<T>& as_rot(const T& theta, const vector3<T>& v) {
    set_rot(theta, v);
    a30 = a31 = a32 = 0.0;
    a33 = 1.0;
    a03 = a13 = a23 = 0.0;
    return *this;
  }
  matrix4<T>& as_rot(const vector3<T>& u, const vector3<T>& v) {
    a30 = a31 = a32 = 0.0;
    a33 = 1.0;
    a03 = a13 = a23 = 0.0;
    set_rot(u, v);
    return *this;
  }

  matrix4<T>& set_scale(const vector3<T>& s);
  vector3<T>& get_scale(vector3<T>& s) const;
  matrix4<T>& as_scale(const vector3<T>& s);
  matrix4<T>& as_scale(const T& s);
  matrix4<T>& set_translation(const vector3<T>& t);
  inline matrix4<T>& set_translate(const vector3<T>& t) {
    return set_translation(t);
  }
  vector3<T>& get_translation(vector3<T>& t) const;
  matrix4<T>& as_translation(const vector3<T>& t);

  matrix4<T> operator*(const matrix4<T>&) const;
  // TL
  matrix4<T>& operator*=(const matrix4<T>& M) {
    *this = mult(*this, M);
    return *this;
  }

  // TL: some additional methods that look like OpenGL...
  //  they behave the same as the OpenGL matrix system
  //  But: using vector3<T> class; rotation is in Radians
  //  TODO: optimize
  matrix4<T>& identity() {
    mat_array[0] = T(1);
    mat_array[1] = T(0);
    mat_array[2] = T(0);
    mat_array[3] = T(0);
    mat_array[4] = T(0);
    mat_array[5] = T(1);
    mat_array[6] = T(0);
    mat_array[7] = T(0);
    mat_array[8] = T(0);
    mat_array[9] = T(0);
    mat_array[10] = T(1);
    mat_array[11] = T(0);
    mat_array[12] = T(0);
    mat_array[13] = T(0);
    mat_array[14] = T(0);
    mat_array[15] = T(1);

    return *this;
  }
  matrix4<T>& translate(vector3<T> t) {
    *this *= matrix4<T>().as_translation(t);
    return *this;
  }
  matrix4<T>& translate(T* t) {
    *this *= matrix4<T>().as_translation(*(vector3<T>*)t);
    return *this;
  }
  matrix4<T>& scale(vector3<T> s) {
    *this *= matrix4<T>().as_scale(s);
    return *this;
  }
  matrix4<T>& scale(T s) {
    *this *= matrix4<T>().as_scale(s);
    return *this;
  }
  matrix4<T>& rotate(const T& theta, const vector3<T>& v) {
    *this *= matrix4<T>().as_rot(theta, v);
    return *this;
  }
  matrix4<T>& rotate(quaternion<T>& q) {
    *this *= matrix4<T>().identity().set_rot(q);
    return *this;
  }

  union {
    struct {
      T a00, a10, a20, a30;  // standard names for components
      T a01, a11, a21, a31;  // standard names for components
      T a02, a12, a22, a32;  // standard names for components
      T a03, a13, a23, a33;  // standard names for components
    };
    T mat_array[16];  // array access
  };

  T* get_value() { return mat_array; }
  const T* get_value() const { return mat_array; }

#ifdef NVMATH_SUPPORTS_GLM
  matrix4(const glm::mat4& f) {
    mat_array[0] = f[0].x;
    mat_array[1] = f[0].y;
    mat_array[2] = f[0].z;
    mat_array[3] = f[0].w;
    mat_array[4] = f[1].x;
    mat_array[5] = f[1].y;
    mat_array[6] = f[1].z;
    mat_array[7] = f[1].w;
    mat_array[8] = f[2].x;
    mat_array[9] = f[2].y;
    mat_array[10] = f[2].z;
    mat_array[11] = f[2].w;
    mat_array[12] = f[3].x;
    mat_array[13] = f[3].y;
    mat_array[14] = f[3].z;
    mat_array[15] = f[3].w;
  }
  operator glm::mat4() const {
    return glm::mat4(mat_array[0], mat_array[1], mat_array[2], mat_array[3],
                     mat_array[4], mat_array[5], mat_array[6], mat_array[7],
                     mat_array[8], mat_array[9], mat_array[10], mat_array[11],
                     mat_array[12], mat_array[13], mat_array[14],
                     mat_array[15]);
  }
#endif
};  // struct matrix4

// quaternion<T>ernion
template <class T>
struct quaternion {
 public:
  quaternion() : quaternion(0, 0, 0, 0) {}
  explicit quaternion(T* q) {
    x = q[0];
    y = q[1];
    z = q[2];
    w = q[3];
  }
  template <typename T0, typename T1, typename T2, typename T3>
  quaternion(T0 x, T1 y, T2 z, T3 w) : x(T(x)), y(T(y)), z(T(z)), w(T(w)) {}
  quaternion(const quaternion<T>& quaternion) = default;
  quaternion(const vector3<T>& axis, T angle);
  explicit quaternion(const vector3<T>& eulerXYZ);  // From Euler
  explicit quaternion(const matrix3<T>& rot);
  explicit quaternion(const matrix4<T>& rot);
  quaternion<T>& operator=(const quaternion<T>& quaternion);
  quaternion<T> operator-() { return quaternion<T>(-x, -y, -z, -w); }
  quaternion<T> inverse();
  quaternion<T> conjugate();
  void normalize();
  void from_matrix(const matrix3<T>& mat);
  void from_matrix(const matrix4<T>& mat);
  void to_matrix(matrix3<T>& mat) const;
  void to_matrix(matrix4<T>& mat) const;
  void to_euler_xyz(vector3<T>& r);
  void to_euler_xyz(T* r);
  void from_euler_xyz(vector3<T> r);
  quaternion<T>& operator*=(const quaternion<T>& q) {
    *this = *this * q;
    return *this;
  }

  T& operator[](int i) { return comp[i]; }
  const T& operator[](int i) const { return comp[i]; }
  union {
    struct {
      T x, y, z, w;
    };
    T comp[4];
  };
};  // struct quaternion

enum Axis {
  AXIS_X,
  AXIS_Y,
  AXIS_Z,
};

template <typename T>
inline Axis get_major_axis(const vector2<T>& vec) {
  return (vec.x > vec.y) ? AXIS_X : AXIS_Y;
}
template <typename T>
inline Axis get_major_axis(const vector3<T>& vec) {
  return (vec.x > vec.y) ? ((vec.x > vec.z) ? AXIS_X : AXIS_Z)
                         : ((vec.y > vec.z) ? AXIS_Y : AXIS_Z);
}

enum VecDirection {
  VECDIR_NNN,  // X = Bit 2 | Y = Bit 1| Z = Bit 0
  VECDIR_NNP,
  VECDIR_NPN,
  VECDIR_NPP,
  VECDIR_PNN,
  VECDIR_PNP,
  VECDIR_PPN,
  VECDIR_PPP
};

enum PlaneSide {
  PLANESIDE_ON_PLANE = 0,  // a primitive is considered as being in the plane
  PLANESIDE_FRONT = 1,     // a primitive is considered as in the front of the
                           // plane; i.e. the positive halfspace
  PLANESIDE_BACK = 2,  // a primitive is considered as in the back of the plane;
                       // i.e. the negative halfspace
  PLANESIDE_SPANNING = 3  // a primitive is considered as straddling cross the
                          // plane, i.e. parts of it are in the positive and
                          // other parts are in the negative halfspace
};

template <typename T>
inline VecDirection get_vector_direction(const vector3<T>& vec) {
  int dir = !std::signbit(vec.x) ? 4 : 0;
  dir |= !std::signbit(vec.y) << 1;
  dir |= !std::signbit(vec.z);
  return static_cast<VecDirection>(dir);
}

// A plane is defined as x * p.x + y * p.y + z * p.z + p.w = 0
// A plane defines the positive halfspace as x * p.x + y * p.y + z * p.z + p.w >
// 0 A plane defines the negative halfspace as x * p.x + y * p.y + z * p.z + p.w
// < 0
template <typename T>
struct plane : public vector4<T> {
  enum NormalizeInConstruction { no, yes };

  inline plane(){};
  inline explicit plane(const vector4<T>& v,
                        NormalizeInConstruction normalizePlane = yes);
  // define plane from normal and a point in the plane
  inline explicit plane(const vector3<T>& normal,
                        const vector3<T>& point = vector3<T>(0, 0, 0),
                        NormalizeInConstruction normalizePlane = yes);
  // define plane from normal and distance along the normal (in units of the
  // normal's length)
  inline explicit plane(const vector3<T>& normal,
                        const float dist,
                        NormalizeInConstruction normalizePlane = yes);
  // define plane from three points that lie in the plane
  inline explicit plane(const vector3<T>& v1,
                        const vector3<T>& v2,
                        const vector3<T>& v3,
                        NormalizeInConstruction normalizePlane = yes);
  // define plane from 3 coefficients: x*a + y*b + z*c + d = 0
  inline explicit plane(T a,
                        T b,
                        T c,
                        T d,
                        NormalizeInConstruction normalizePlane = yes);

  vector3<T> normal() const { return vector3<T>(this->x, this->y, this->z); }
  T distanceFromOrigin() const { return -this->w; }

  plane operator-();
};

typedef vector2<nv_scalar> vec2f;
typedef vector3<nv_scalar> vec3f;
typedef vector4<nv_scalar> vec4f;
typedef matrix3<nv_scalar> mat3f;
typedef matrix4<nv_scalar> mat4f;
typedef quaternion<nv_scalar> quatf;
typedef plane<nv_scalar> planef;

typedef vector2<int> vec2i;
typedef vector3<int> vec3i;
typedef vector4<int> vec4i;

typedef vector2<unsigned int> vec2ui;
typedef vector3<unsigned int> vec3ui;
typedef vector4<unsigned int> vec4ui;

typedef unsigned int uint;

template <typename T>
vector2<T> make_vec2(T const* const ptr);
template <typename T>
vector3<T> make_vec3(T const* const ptr);
template <typename T>
vector4<T> make_vec4(T const* const ptr);

}  // namespace nvmath

#endif
