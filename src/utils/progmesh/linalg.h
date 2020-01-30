// linalg.h - v2.0 - Single-header public domain linear algebra library
//
// The intent of this library is to provide the bulk of the functionality
// you need to write programs that frequently use small, fixed-size vectors
// and matrices, in domains such as computational geometry or computer
// graphics. It strives for terse, readable source code.
//
// The original author of this software is Sterling Orsten, and its permanent
// home is <http://github.com/sgorsten/linalg/>. If you find this software
// useful, an acknowledgement in your source text and/or product documentation
// is appreciated, but not required.
//
// The author acknowledges significant insights and contributions by:
//     Stan Melax <http://github.com/melax/>
//     Dimitri Diakopoulos <http://github.com/ddiakopoulos/>

// This is free and unencumbered software released into the public domain.
//
// Anyone is free to copy, modify, publish, use, compile, sell, or
// distribute this software, either in source code form or as a compiled
// binary, for any purpose, commercial or non-commercial, and by any
// means.
//
// In jurisdictions that recognize copyright laws, the author or authors
// of this software dedicate any and all copyright interest in the
// software to the public domain. We make this dedication for the benefit
// of the public at large and to the detriment of our heirs and
// successors. We intend this dedication to be an overt act of
// relinquishment in perpetuity of all present and future rights to this
// software under copyright law.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
// IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
// OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
// ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
// OTHER DEALINGS IN THE SOFTWARE.
//
// For more information, please refer to <http://unlicense.org/>

#pragma once
#ifndef LINALG_H
#define LINALG_H

#include <array>    // For std::array, used in the relational operator overloads
#include <cmath>    // For various unary math functions, such as std::sqrt
#include <cstdint>  // For implementing namespace linalg::aliases

namespace linalg {
// Small, fixed-length vector type, consisting of exactly M elements of type T,
// and presumed to be a column-vector unless otherwise noted
template <class T, int M>
struct vec;
template <class T>
struct vec<T, 2> {
  T x, y;
  vec() : x(), y() {}
  vec(T x, T y) : x(x), y(y) {}
  explicit vec(T s) : x(s), y(s) {}
  explicit vec(const T* p) : vec(p[0], p[1]) {}
  template <class U>
  explicit vec(const vec<U, 2>& v)
      : vec(static_cast<T>(v.x), static_cast<T>(v.y)) {}
  const T& operator[](int i) const { return (&x)[i]; }
  T& operator[](int i) { return (&x)[i]; }
};
template <class T>
struct vec<T, 3> {
  T x, y, z;
  vec() : x(), y(), z() {}
  vec(T x, T y, T z) : x(x), y(y), z(z) {}
  explicit vec(T s) : x(s), y(s), z(s) {}
  explicit vec(const T* p) : vec(p[0], p[1], p[2]) {}
  template <class U>
  explicit vec(const vec<U, 3>& v)
      : vec(static_cast<T>(v.x), static_cast<T>(v.y), static_cast<T>(v.z)) {}
  vec(const vec<T, 2>& xy, T z) : vec(xy.x, xy.y, z) {}
  const T& operator[](int i) const { return (&x)[i]; }
  T& operator[](int i) { return (&x)[i]; }
  const vec<T, 2>& xy() const {
    return *reinterpret_cast<const vec<T, 2>*>(this);
  }
  vec<T, 2>& xy() { return *reinterpret_cast<vec<T, 2>*>(this); }
};
template <class T>
struct vec<T, 4> {
  T x, y, z, w;
  vec() : x(), y(), z(), w() {}
  vec(T x, T y, T z, T w) : x(x), y(y), z(z), w(w) {}
  explicit vec(T s) : x(s), y(s), z(s), w(s) {}
  explicit vec(const T* p) : vec(p[0], p[1], p[2], p[3]) {}
  template <class U>
  explicit vec(const vec<U, 4>& v)
      : vec(static_cast<T>(v.x),
            static_cast<T>(v.y),
            static_cast<T>(v.z),
            static_cast<T>(v.w)) {}
  vec(const vec<T, 3>& xyz, T w) : vec(xyz.x, xyz.y, xyz.z, w) {}
  const T& operator[](int i) const { return (&x)[i]; }
  T& operator[](int i) { return (&x)[i]; }
  const vec<T, 2>& xy() const {
    return *reinterpret_cast<const vec<T, 2>*>(this);
  }
  vec<T, 2>& xy() { return *reinterpret_cast<vec<T, 2>*>(this); }
  const vec<T, 3>& xyz() const {
    return *reinterpret_cast<const vec<T, 3>*>(this);
  }
  vec<T, 3>& xyz() { return *reinterpret_cast<vec<T, 3>*>(this); }
};

// Small, fixed-size matrix type, consisting of exactly M rows and N columns of
// type T, stored in column-major order.
template <class T, int M, int N>
struct mat;
template <class T, int M>
struct mat<T, M, 2> {
  typedef vec<T, M> V;
  V x, y;
  mat() : x(), y() {}
  mat(V x, V y) : x(x), y(y) {}
  explicit mat(T s) : x(s), y(s) {}
  explicit mat(const T* p) : x(p + M * 0), y(p + M * 1) {}
  template <class U>
  explicit mat(const mat<U, M, 2>& m) : mat(V(m.x), V(m.y)) {}
  vec<T, 2> row(int i) const { return {x[i], y[i]}; }
  const V& operator[](int j) const { return (&x)[j]; }
  V& operator[](int j) { return (&x)[j]; }
};
template <class T, int M>
struct mat<T, M, 3> {
  typedef vec<T, M> V;
  V x, y, z;
  mat() : x(), y(), z() {}
  mat(V x, V y, V z) : x(x), y(y), z(z) {}
  explicit mat(T s) : x(s), y(s), z(s) {}
  explicit mat(const T* p) : x(p + M * 0), y(p + M * 1), z(p + M * 2) {}
  template <class U>
  explicit mat(const mat<U, M, 3>& m) : mat(V(m.x), V(m.y), V(m.z)) {}
  vec<T, 3> row(int i) const { return {x[i], y[i], z[i]}; }
  const V& operator[](int j) const { return (&x)[j]; }
  V& operator[](int j) { return (&x)[j]; }
};
template <class T, int M>
struct mat<T, M, 4> {
  typedef vec<T, M> V;
  V x, y, z, w;
  mat() : x(), y(), z(), w() {}
  mat(V x, V y, V z, V w) : x(x), y(y), z(z), w(w) {}
  explicit mat(T s) : x(s), y(s), z(s), w(s) {}
  explicit mat(const T* p)
      : x(p + M * 0), y(p + M * 1), z(p + M * 2), w(p + M * 3) {}
  template <class U>
  explicit mat(const mat<U, M, 4>& m) : mat(V(m.x), V(m.y), V(m.z), V(m.w)) {}
  vec<T, 4> row(int i) const { return {x[i], y[i], z[i], w[i]}; }
  const V& operator[](int j) const { return (&x)[j]; }
  V& operator[](int j) { return (&x)[j]; }
};

// Type traits for a binary operation involving linear algebra types, used for
// SFINAE on templated functions and operator overloads
template <class A, class B>
struct traits {};
template <class T, int M>
struct traits<vec<T, M>, vec<T, M>> {
  typedef T scalar;
  typedef vec<T, M> result;
  typedef vec<bool, M> bool_result;
  typedef vec<decltype(+T()), M> arith_result;
  typedef std::array<T, M> compare_as;
};
template <class T, int M>
struct traits<vec<T, M>, T> {
  typedef T scalar;
  typedef vec<T, M> result;
  typedef vec<bool, M> bool_result;
  typedef vec<decltype(+T()), M> arith_result;
};
template <class T, int M>
struct traits<T, vec<T, M>> {
  typedef T scalar;
  typedef vec<T, M> result;
  typedef vec<bool, M> bool_result;
  typedef vec<decltype(+T()), M> arith_result;
};
template <class T, int M, int N>
struct traits<mat<T, M, N>, mat<T, M, N>> {
  typedef T scalar;
  typedef mat<T, M, N> result;
  typedef mat<bool, M, N> bool_result;
  typedef mat<decltype(+T()), M, N> arith_result;
  typedef std::array<T, M * N> compare_as;
};
template <class T, int M, int N>
struct traits<mat<T, M, N>, T> {
  typedef T scalar;
  typedef mat<T, M, N> result;
  typedef mat<bool, M, N> bool_result;
  typedef mat<decltype(+T()), M, N> arith_result;
};
template <class T, int M, int N>
struct traits<T, mat<T, M, N>> {
  typedef T scalar;
  typedef mat<T, M, N> result;
  typedef mat<bool, M, N> bool_result;
  typedef mat<decltype(+T()), M, N> arith_result;
};
template <class A, class B = A>
using scalar_t =
    typename traits<A, B>::scalar;  // Underlying scalar type when performing
                                    // elementwise operations
template <class A, class B = A>
using result_t = typename traits<A, B>::result;  // Result of calling a function
                                                 // on linear algebra types
template <class A, class B = A>
using bool_result_t =
    typename traits<A, B>::bool_result;  // Result of a comparison or unary not
                                         // operation on linear algebra types
template <class A, class B = A>
using arith_result_t =
    typename traits<A, B>::arith_result;  // Result of an arithmetic operation
                                          // on linear algebra types (accounts
                                          // for integer promotion)

// Produce a scalar by applying f(T,T) -> T to adjacent pairs of elements from
// vector a in left-to-right order (matching the associativity of arithmetic and
// logical operators)
template <class T, class F>
T fold(const vec<T, 2>& a, F f) {
  return f(a.x, a.y);
}
template <class T, class F>
T fold(const vec<T, 3>& a, F f) {
  return f(f(a.x, a.y), a.z);
}
template <class T, class F>
T fold(const vec<T, 4>& a, F f) {
  return f(f(f(a.x, a.y), a.z), a.w);
}

// Produce a vector/matrix by applying f(T,T) to corresponding pairs of elements
// from vectors/matrix a and b
template <class T, class F>
auto zip(const vec<T, 2>& a, const vec<T, 2>& b, F f)
    -> vec<decltype(f(T(), T())), 2> {
  return {f(a.x, b.x), f(a.y, b.y)};
}
template <class T, class F>
auto zip(const vec<T, 3>& a, const vec<T, 3>& b, F f)
    -> vec<decltype(f(T(), T())), 3> {
  return {f(a.x, b.x), f(a.y, b.y), f(a.z, b.z)};
}
template <class T, class F>
auto zip(const vec<T, 4>& a, const vec<T, 4>& b, F f)
    -> vec<decltype(f(T(), T())), 4> {
  return {f(a.x, b.x), f(a.y, b.y), f(a.z, b.z), f(a.w, b.w)};
}
template <class T, int M, class F>
auto zip(const vec<T, M>& a, T b, F f) -> vec<decltype(f(T(), T())), M> {
  return zip(a, vec<T, M>(b), f);
}
template <class T, int M, class F>
auto zip(T a, const vec<T, M>& b, F f) -> vec<decltype(f(T(), T())), M> {
  return zip(vec<T, M>(a), b, f);
}
template <class T, int M, class F>
auto zip(const mat<T, M, 2>& a, const mat<T, M, 2>& b, F f)
    -> mat<decltype(f(T(), T())), M, 2> {
  return {zip(a.x, b.x, f), zip(a.y, b.y, f)};
}
template <class T, int M, class F>
auto zip(const mat<T, M, 3>& a, const mat<T, M, 3>& b, F f)
    -> mat<decltype(f(T(), T())), M, 3> {
  return {zip(a.x, b.x, f), zip(a.y, b.y, f), zip(a.z, b.z, f)};
}
template <class T, int M, class F>
auto zip(const mat<T, M, 4>& a, const mat<T, M, 4>& b, F f)
    -> mat<decltype(f(T(), T())), M, 4> {
  return {zip(a.x, b.x, f), zip(a.y, b.y, f), zip(a.z, b.z, f),
          zip(a.w, b.w, f)};
}
template <class T, int M, int N, class F>
auto zip(const mat<T, M, N>& a, T b, F f) -> mat<decltype(f(T(), T())), M, N> {
  return zip(a, mat<T, M, N>(b), f);
}
template <class T, int M, int N, class F>
auto zip(T a, const mat<T, M, N>& b, F f) -> mat<decltype(f(T(), T())), M, N> {
  return zip(mat<T, M, N>(a), b, f);
}

// Produce a vector/matrix by applying f(T) to elements from vector/matrix a
template <class T, int M, class F>
auto map(const vec<T, M>& a, F f) -> vec<decltype(f(T())), M> {
  return zip(a, a, [f](T l, T) { return f(l); });
}
template <class T, int M, int N, class F>
auto map(const mat<T, M, N>& a, F f) -> mat<decltype(f(T())), M, N> {
  return zip(a, a, [f](T l, T) { return f(l); });
}

// Relational operators are defined to compare the elements of two vectors or
// matrices lexicographically, in column-major order
template <class A, class C = typename traits<A, A>::compare_as>
bool operator==(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) == reinterpret_cast<const C&>(b);
}
template <class A, class C = typename traits<A, A>::compare_as>
bool operator!=(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) != reinterpret_cast<const C&>(b);
}
template <class A, class C = typename traits<A, A>::compare_as>
bool operator<(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) < reinterpret_cast<const C&>(b);
}
template <class A, class C = typename traits<A, A>::compare_as>
bool operator>(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) > reinterpret_cast<const C&>(b);
}
template <class A, class C = typename traits<A, A>::compare_as>
bool operator<=(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) <= reinterpret_cast<const C&>(b);
}
template <class A, class C = typename traits<A, A>::compare_as>
bool operator>=(const A& a, const A& b) {
  return reinterpret_cast<const C&>(a) >= reinterpret_cast<const C&>(b);
}

// Functions for coalescing scalar values
template <int M>
bool any(const vec<bool, M>& a) {
  return fold(a, [](bool l, bool r) { return l || r; });
}
template <int M>
bool all(const vec<bool, M>& a) {
  return fold(a, [](bool l, bool r) { return l && r; });
}
template <class T, int M>
T sum(const vec<T, M>& a) {
  return fold(a, [](T l, T r) { return l + r; });
}
template <class T, int M>
T product(const vec<T, M>& a) {
  return fold(a, [](T l, T r) { return l * r; });
}
template <class T, int M>
int argmin(const vec<T, M>& a) {
  int j = 0;
  for (int i = 1; i < M; ++i)
    if (a[i] < a[j])
      j = i;
  return j;
}
template <class T, int M>
int argmax(const vec<T, M>& a) {
  int j = 0;
  for (int i = 1; i < M; ++i)
    if (a[i] > a[j])
      j = i;
  return j;
}
template <class T, int M>
T minelem(const vec<T, M>& a) {
  return a[argmin(a)];
}
template <class T, int M>
T maxelem(const vec<T, M>& a) {
  return a[argmax(a)];
}

// Overloads for unary operators on vectors are implemented in terms of
// elementwise application of the operator
template <class A>
arith_result_t<A> operator+(const A& a) {
  return map(a, [](scalar_t<A> l) { return +l; });
}
template <class A>
arith_result_t<A> operator-(const A& a) {
  return map(a, [](scalar_t<A> l) { return -l; });
}
template <class A>
arith_result_t<A> operator~(const A& a) {
  return map(a, [](scalar_t<A> l) { return ~l; });
}
template <class A>
bool_result_t<A> operator!(const A& a) {
  return map(a, [](scalar_t<A> l) { return !l; });
}

// Mirror the set of unary scalar math functions to apply elementwise to vectors
template <class A>
result_t<A> abs(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::abs(l); });
}
template <class A>
result_t<A> floor(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::floor(l); });
}
template <class A>
result_t<A> ceil(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::ceil(l); });
}
template <class A>
result_t<A> exp(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::exp(l); });
}
template <class A>
result_t<A> log(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::log(l); });
}
template <class A>
result_t<A> log10(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::log10(l); });
}
template <class A>
result_t<A> sqrt(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::sqrt(l); });
}
template <class A>
result_t<A> sin(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::sin(l); });
}
template <class A>
result_t<A> cos(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::cos(l); });
}
template <class A>
result_t<A> tan(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::tan(l); });
}
template <class A>
result_t<A> asin(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::asin(l); });
}
template <class A>
result_t<A> acos(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::acos(l); });
}
template <class A>
result_t<A> atan(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::atan(l); });
}
template <class A>
result_t<A> sinh(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::sinh(l); });
}
template <class A>
result_t<A> cosh(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::cosh(l); });
}
template <class A>
result_t<A> tanh(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::tanh(l); });
}
template <class A>
result_t<A> round(const A& a) {
  return map(a, [](scalar_t<A> l) { return std::round(l); });
}

// Overloads for vector op vector are implemented in terms of elementwise
// application of the operator, followed by casting back to the original type
// (integer promotion is suppressed)
template <class A, class B>
arith_result_t<A, B> operator+(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l + r; });
}
template <class A, class B>
arith_result_t<A, B> operator-(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l - r; });
}
template <class A, class B>
arith_result_t<A, B> operator*(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l * r; });
}
template <class A, class B>
arith_result_t<A, B> operator/(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l / r; });
}
template <class A, class B>
arith_result_t<A, B> operator%(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l % r; });
}
template <class A, class B>
arith_result_t<A, B> operator|(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l | r; });
}
template <class A, class B>
arith_result_t<A, B> operator^(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l ^ r; });
}
template <class A, class B>
arith_result_t<A, B> operator&(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l & r; });
}
template <class A, class B>
arith_result_t<A, B> operator<<(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l << r; });
}
template <class A, class B>
arith_result_t<A, B> operator>>(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l >> r; });
}

// Overloads for assignment operators are implemented trivially
template <class A, class B>
result_t<A, A>& operator+=(A& a, const B& b) {
  return a = a + b;
}
template <class A, class B>
result_t<A, A>& operator-=(A& a, const B& b) {
  return a = a - b;
}
template <class A, class B>
result_t<A, A>& operator*=(A& a, const B& b) {
  return a = a * b;
}
template <class A, class B>
result_t<A, A>& operator/=(A& a, const B& b) {
  return a = a / b;
}
template <class A, class B>
result_t<A, A>& operator%=(A& a, const B& b) {
  return a = a % b;
}
template <class A, class B>
result_t<A, A>& operator|=(A& a, const B& b) {
  return a = a | b;
}
template <class A, class B>
result_t<A, A>& operator^=(A& a, const B& b) {
  return a = a ^ b;
}
template <class A, class B>
result_t<A, A>& operator&=(A& a, const B& b) {
  return a = a & b;
}
template <class A, class B>
result_t<A, A>& operator<<=(A& a, const B& b) {
  return a = a << b;
}
template <class A, class B>
result_t<A, A>& operator>>=(A& a, const B& b) {
  return a = a >> b;
}

// Mirror the set of binary scalar math functions to apply elementwise to
// vectors
template <class A, class B>
result_t<A, B> min(const A& a, const B& b) {
  return zip(a, b,
             [](scalar_t<A, B> l, scalar_t<A, B> r) { return l < r ? l : r; });
}
template <class A, class B>
result_t<A, B> max(const A& a, const B& b) {
  return zip(a, b,
             [](scalar_t<A, B> l, scalar_t<A, B> r) { return l > r ? l : r; });
}
template <class A, class B>
result_t<A, B> fmod(const A& a, const B& b) {
  return zip(
      a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return std::fmod(l, r); });
}
template <class A, class B>
result_t<A, B> pow(const A& a, const B& b) {
  return zip(a, b,
             [](scalar_t<A, B> l, scalar_t<A, B> r) { return std::pow(l, r); });
}
template <class A, class B>
result_t<A, B> atan2(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) {
    return std::atan2(l, r);
  });
}
template <class A, class B>
result_t<A, B> clamp(const A& a, const B& b, const B& c) {
  return min(max(a, b), c);
}  // TODO: Revisit

// Functions for componentwise application of equivalence and relational
// operators
template <class A, class B>
bool_result_t<A, B> equal(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l == r; });
}
template <class A, class B>
bool_result_t<A, B> nequal(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l != r; });
}
template <class A, class B>
bool_result_t<A, B> less(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l < r; });
}
template <class A, class B>
bool_result_t<A, B> greater(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l > r; });
}
template <class A, class B>
bool_result_t<A, B> lequal(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l <= r; });
}
template <class A, class B>
bool_result_t<A, B> gequal(const A& a, const B& b) {
  return zip(a, b, [](scalar_t<A, B> l, scalar_t<A, B> r) { return l >= r; });
}

// Support for vector algebra
template <class T>
T cross(const vec<T, 2>& a, const vec<T, 2>& b) {
  return a.x * b.y - a.y * b.x;
}
template <class T>
vec<T, 3> cross(const vec<T, 3>& a, const vec<T, 3>& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
template <class T, int M>
T dot(const vec<T, M>& a, const vec<T, M>& b) {
  return sum(a * b);
}
template <class T, int M>
T length2(const vec<T, M>& a) {
  return dot(a, a);
}
template <class T, int M>
T length(const vec<T, M>& a) {
  return std::sqrt(length2(a));
}
template <class T, int M>
vec<T, M> normalize(const vec<T, M>& a) {
  return a / length(a);
}
template <class T, int M>
T distance2(const vec<T, M>& a, const vec<T, M>& b) {
  return length2(b - a);
}
template <class T, int M>
T distance(const vec<T, M>& a, const vec<T, M>& b) {
  return length(b - a);
}
template <class T, int M>
T uangle(const vec<T, M>& a, const vec<T, M>& b) {
  T d = dot(a, b);
  return d > 1 ? 0 : std::acos(d < -1 ? -1 : d);
}
template <class T, int M>
vec<T, M> lerp(const vec<T, M>& a, const vec<T, M>& b, T t) {
  return a * (1 - t) + b * t;
}
template <class T, int M>
vec<T, M> nlerp(const vec<T, M>& a, const vec<T, M>& b, T t) {
  return normalize(lerp(a, b, t));
}
template <class T, int M>
vec<T, M> slerp(const vec<T, M>& a, const vec<T, M>& b, T t) {
  T th = uangle(a, b);
  return th == 0 ? a
                 : a * (std::sin(th * (1 - t)) / std::sin(th)) +
                       b * (std::sin(th * t) / std::sin(th));
}
template <class T, int M>
mat<T, M, 2> outerprod(const vec<T, M>& a, const vec<T, 2>& b) {
  return {a * b.x, a * b.y};
}
template <class T, int M>
mat<T, M, 3> outerprod(const vec<T, M>& a, const vec<T, 3>& b) {
  return {a * b.x, a * b.y, a * b.z};
}
template <class T, int M>
mat<T, M, 4> outerprod(const vec<T, M>& a, const vec<T, 4>& b) {
  return {a * b.x, a * b.y, a * b.z, a * b.w};
}

// Support for quaternion algebra using 4D vectors, representing xi + yj + zk +
// w
template <class T>
vec<T, 4> qconj(const vec<T, 4>& q) {
  return {-q.x, -q.y, -q.z, q.w};
}
template <class T>
vec<T, 4> qinv(const vec<T, 4>& q) {
  return qconj(q) / length2(q);
}
template <class T>
vec<T, 4> qmul(const vec<T, 4>& a, const vec<T, 4>& b) {
  return {a.x * b.w + a.w * b.x + a.y * b.z - a.z * b.y,
          a.y * b.w + a.w * b.y + a.z * b.x - a.x * b.z,
          a.z * b.w + a.w * b.z + a.x * b.y - a.y * b.x,
          a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
template <class T, class... R>
vec<T, 4> qmul(const vec<T, 4>& a, R... r) {
  return qmul(a, qmul(r...));
}
// TODO: qexp, qlog

// Support for 3D spatial rotations using quaternions, via qmul(qmul(q, v),
// qconj(q))
template <class T>
vec<T, 3> qxdir(const vec<T, 4>& q) {
  return {q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z,
          (q.x * q.y + q.z * q.w) * 2, (q.z * q.x - q.y * q.w) * 2};
}
template <class T>
vec<T, 3> qydir(const vec<T, 4>& q) {
  return {(q.x * q.y - q.z * q.w) * 2,
          q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z,
          (q.y * q.z + q.x * q.w) * 2};
}
template <class T>
vec<T, 3> qzdir(const vec<T, 4>& q) {
  return {(q.z * q.x + q.y * q.w) * 2, (q.y * q.z - q.x * q.w) * 2,
          q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z};
}
template <class T>
mat<T, 3, 3> qmat(const vec<T, 4>& q) {
  return {qxdir(q), qydir(q), qzdir(q)};
}
template <class T>
vec<T, 3> qrot(const vec<T, 4>& q, const vec<T, 3>& v) {
  return qxdir(q) * v.x + qydir(q) * v.y + qzdir(q) * v.z;
}
template <class T>
T qangle(const vec<T, 4>& q) {
  return std::acos(q.w) * 2;
}
template <class T>
vec<T, 3> qaxis(const vec<T, 4>& q) {
  return normalize(q.xyz());
}
template <class T>
vec<T, 4> qnlerp(const vec<T, 4>& a, const vec<T, 4>& b, T t) {
  return nlerp(a, dot(a, b) < 0 ? -b : b, t);
}
template <class T>
vec<T, 4> qslerp(const vec<T, 4>& a, const vec<T, 4>& b, T t) {
  return slerp(a, dot(a, b) < 0 ? -b : b, t);
}

// Support for matrix algebra
template <class T, int M>
vec<T, M> mul(const mat<T, M, 2>& a, const vec<T, 2>& b) {
  return a.x * b.x + a.y * b.y;
}
template <class T, int M>
vec<T, M> mul(const mat<T, M, 3>& a, const vec<T, 3>& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
template <class T, int M>
vec<T, M> mul(const mat<T, M, 4>& a, const vec<T, 4>& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}
template <class T, int M, int N>
mat<T, M, 2> mul(const mat<T, M, N>& a, const mat<T, N, 2>& b) {
  return {mul(a, b.x), mul(a, b.y)};
}
template <class T, int M, int N>
mat<T, M, 3> mul(const mat<T, M, N>& a, const mat<T, N, 3>& b) {
  return {mul(a, b.x), mul(a, b.y), mul(a, b.z)};
}
template <class T, int M, int N>
mat<T, M, 4> mul(const mat<T, M, N>& a, const mat<T, N, 4>& b) {
  return {mul(a, b.x), mul(a, b.y), mul(a, b.z), mul(a, b.w)};
}
#if _MSC_VER >= 1910
template <class T, int M, int N, class... R>
constexpr auto mul(const mat<T, M, N>& a, R... r) {
  return mul(a, mul(r...));
}
#else
template <class T, int M, int N, class... R>
constexpr auto mul(const mat<T, M, N>& a, R... r)
    -> decltype(mul(a, mul(r...))) {
  return mul(a, mul(r...));
}
#endif
template <class T, int M>
mat<T, M, 2> transpose(const mat<T, 2, M>& m) {
  return {m.row(0), m.row(1)};
}
template <class T, int M>
mat<T, M, 3> transpose(const mat<T, 3, M>& m) {
  return {m.row(0), m.row(1), m.row(2)};
}
template <class T, int M>
mat<T, M, 4> transpose(const mat<T, 4, M>& m) {
  return {m.row(0), m.row(1), m.row(2), m.row(3)};
}
template <class T>
mat<T, 2, 2> adjugate(const mat<T, 2, 2>& a) {
  return {{a.y.y, -a.x.y}, {-a.y.x, a.x.x}};
}
template <class T>
mat<T, 3, 3> adjugate(const mat<T, 3, 3>& a) {
  return {{a.y.y * a.z.z - a.z.y * a.y.z, a.z.y * a.x.z - a.x.y * a.z.z,
           a.x.y * a.y.z - a.y.y * a.x.z},
          {a.y.z * a.z.x - a.z.z * a.y.x, a.z.z * a.x.x - a.x.z * a.z.x,
           a.x.z * a.y.x - a.y.z * a.x.x},
          {a.y.x * a.z.y - a.z.x * a.y.y, a.z.x * a.x.y - a.x.x * a.z.y,
           a.x.x * a.y.y - a.y.x * a.x.y}};
}
template <class T>
mat<T, 4, 4> adjugate(const mat<T, 4, 4>& a) {
  return {{a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
               a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
               a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w,
           a.x.y * a.w.z * a.z.w + a.z.y * a.x.z * a.w.w +
               a.w.y * a.z.z * a.x.w - a.w.y * a.x.z * a.z.w -
               a.z.y * a.w.z * a.x.w - a.x.y * a.z.z * a.w.w,
           a.x.y * a.y.z * a.w.w + a.w.y * a.x.z * a.y.w +
               a.y.y * a.w.z * a.x.w - a.x.y * a.w.z * a.y.w -
               a.y.y * a.x.z * a.w.w - a.w.y * a.y.z * a.x.w,
           a.x.y * a.z.z * a.y.w + a.y.y * a.x.z * a.z.w +
               a.z.y * a.y.z * a.x.w - a.x.y * a.y.z * a.z.w -
               a.z.y * a.x.z * a.y.w - a.y.y * a.z.z * a.x.w},
          {a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x +
               a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x -
               a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x,
           a.x.z * a.z.w * a.w.x + a.w.z * a.x.w * a.z.x +
               a.z.z * a.w.w * a.x.x - a.x.z * a.w.w * a.z.x -
               a.z.z * a.x.w * a.w.x - a.w.z * a.z.w * a.x.x,
           a.x.z * a.w.w * a.y.x + a.y.z * a.x.w * a.w.x +
               a.w.z * a.y.w * a.x.x - a.x.z * a.y.w * a.w.x -
               a.w.z * a.x.w * a.y.x - a.y.z * a.w.w * a.x.x,
           a.x.z * a.y.w * a.z.x + a.z.z * a.x.w * a.y.x +
               a.y.z * a.z.w * a.x.x - a.x.z * a.z.w * a.y.x -
               a.y.z * a.x.w * a.z.x - a.z.z * a.y.w * a.x.x},
          {a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y +
               a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y -
               a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y,
           a.x.w * a.w.x * a.z.y + a.z.w * a.x.x * a.w.y +
               a.w.w * a.z.x * a.x.y - a.x.w * a.z.x * a.w.y -
               a.w.w * a.x.x * a.z.y - a.z.w * a.w.x * a.x.y,
           a.x.w * a.y.x * a.w.y + a.w.w * a.x.x * a.y.y +
               a.y.w * a.w.x * a.x.y - a.x.w * a.w.x * a.y.y -
               a.y.w * a.x.x * a.w.y - a.w.w * a.y.x * a.x.y,
           a.x.w * a.z.x * a.y.y + a.y.w * a.x.x * a.z.y +
               a.z.w * a.y.x * a.x.y - a.x.w * a.y.x * a.z.y -
               a.z.w * a.x.x * a.y.y - a.y.w * a.z.x * a.x.y},
          {a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z +
               a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z -
               a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z,
           a.x.x * a.z.y * a.w.z + a.w.x * a.x.y * a.z.z +
               a.z.x * a.w.y * a.x.z - a.x.x * a.w.y * a.z.z -
               a.z.x * a.x.y * a.w.z - a.w.x * a.z.y * a.x.z,
           a.x.x * a.w.y * a.y.z + a.y.x * a.x.y * a.w.z +
               a.w.x * a.y.y * a.x.z - a.x.x * a.y.y * a.w.z -
               a.w.x * a.x.y * a.y.z - a.y.x * a.w.y * a.x.z,
           a.x.x * a.y.y * a.z.z + a.z.x * a.x.y * a.y.z +
               a.y.x * a.z.y * a.x.z - a.x.x * a.z.y * a.y.z -
               a.y.x * a.x.y * a.z.z - a.z.x * a.y.y * a.x.z}};
}
template <class T>
T determinant(const mat<T, 2, 2>& a) {
  return a.x.x * a.y.y - a.x.y * a.y.x;
}
template <class T>
T determinant(const mat<T, 3, 3>& a) {
  return a.x.x * (a.y.y * a.z.z - a.z.y * a.y.z) +
         a.x.y * (a.y.z * a.z.x - a.z.z * a.y.x) +
         a.x.z * (a.y.x * a.z.y - a.z.x * a.y.y);
}
template <class T>
T determinant(const mat<T, 4, 4>& a) {
  return a.x.x * (a.y.y * a.z.z * a.w.w + a.w.y * a.y.z * a.z.w +
                  a.z.y * a.w.z * a.y.w - a.y.y * a.w.z * a.z.w -
                  a.z.y * a.y.z * a.w.w - a.w.y * a.z.z * a.y.w) +
         a.x.y * (a.y.z * a.w.w * a.z.x + a.z.z * a.y.w * a.w.x +
                  a.w.z * a.z.w * a.y.x - a.y.z * a.z.w * a.w.x -
                  a.w.z * a.y.w * a.z.x - a.z.z * a.w.w * a.y.x) +
         a.x.z * (a.y.w * a.z.x * a.w.y + a.w.w * a.y.x * a.z.y +
                  a.z.w * a.w.x * a.y.y - a.y.w * a.w.x * a.z.y -
                  a.z.w * a.y.x * a.w.y - a.w.w * a.z.x * a.y.y) +
         a.x.w * (a.y.x * a.w.y * a.z.z + a.z.x * a.y.y * a.w.z +
                  a.w.x * a.z.y * a.y.z - a.y.x * a.z.y * a.w.z -
                  a.w.x * a.y.y * a.z.z - a.z.x * a.w.y * a.y.z);
}
template <class T, int N>
mat<T, N, N> inverse(const mat<T, N, N>& a) {
  return adjugate(a) / determinant(a);
}

// Vectors and matrices can be used as ranges
template <class T, int M>
T* begin(vec<T, M>& a) {
  return &a[0];
}
template <class T, int M>
const T* begin(const vec<T, M>& a) {
  return &a[0];
}
template <class T, int M>
T* end(vec<T, M>& a) {
  return begin(a) + M;
}
template <class T, int M>
const T* end(const vec<T, M>& a) {
  return begin(a) + M;
}
template <class T, int M, int N>
vec<T, M>* begin(mat<T, M, N>& a) {
  return &a[0];
}
template <class T, int M, int N>
const vec<T, M>* begin(const mat<T, M, N>& a) {
  return &a[0];
}
template <class T, int M, int N>
vec<T, M>* end(mat<T, M, N>& a) {
  return begin(a) + N;
}
template <class T, int M, int N>
const vec<T, M>* end(const mat<T, M, N>& a) {
  return begin(a) + N;
}

// Factory functions for 3D spatial transformations (will possibly be removed or
// changed in a future version)
template <class T>
vec<T, 4> rotation_quat(const vec<T, 3>& axis, T angle) {
  return {axis * std::sin(angle / 2), std::cos(angle / 2)};
}
template <class T>
mat<T, 4, 4> translation_matrix(const vec<T, 3>& translation) {
  return {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {translation, 1}};
}
template <class T>
mat<T, 4, 4> rotation_matrix(const vec<T, 4>& rotation) {
  return {{qxdir(rotation), 0},
          {qydir(rotation), 0},
          {qzdir(rotation), 0},
          {0, 0, 0, 1}};
}
template <class T>
mat<T, 4, 4> pose_matrix(const vec<T, 4>& q, const vec<T, 3>& p) {
  return {{qxdir(q), 0}, {qydir(q), 0}, {qzdir(q), 0}, {p, 1}};
}
template <class T>
mat<T, 4, 4> frustum_matrix(T l, T r, T b, T t, T n, T f) {
  return {{2 * n / (r - l), 0, 0, 0},
          {0, 2 * n / (t - b), 0, 0},
          {(r + l) / (r - l), (t + b) / (t - b), -(f + n) / (f - n), -1},
          {0, 0, -2 * f * n / (f - n), 0}};
}
template <class T>
mat<T, 4, 4> perspective_matrix(T fovy, T aspect, T n, T f) {
  T y = n * std::tan(fovy / 2), x = y * aspect;
  return frustum_matrix(-x, x, -y, y, n, f);
}

// Provide typedefs for common element types and vector/matrix sizes
namespace aliases {
typedef vec<bool, 2> bool2;
typedef vec<uint8_t, 2> byte2;
typedef vec<int16_t, 2> short2;
typedef vec<uint16_t, 2> ushort2;
typedef vec<bool, 3> bool3;
typedef vec<uint8_t, 3> byte3;
typedef vec<int16_t, 3> short3;
typedef vec<uint16_t, 3> ushort3;
typedef vec<bool, 4> bool4;
typedef vec<uint8_t, 4> byte4;
typedef vec<int16_t, 4> short4;
typedef vec<uint16_t, 4> ushort4;
typedef vec<int, 2> int2;
typedef vec<unsigned, 2> uint2;
typedef vec<float, 2> float2;
typedef vec<double, 2> double2;
typedef vec<int, 3> int3;
typedef vec<unsigned, 3> uint3;
typedef vec<float, 3> float3;
typedef vec<double, 3> double3;
typedef vec<int, 4> int4;
typedef vec<unsigned, 4> uint4;
typedef vec<float, 4> float4;
typedef vec<double, 4> double4;
typedef mat<bool, 2, 2> bool2x2;
typedef mat<int, 2, 2> int2x2;
typedef mat<float, 2, 2> float2x2;
typedef mat<double, 2, 2> double2x2;
typedef mat<bool, 2, 3> bool2x3;
typedef mat<int, 2, 3> int2x3;
typedef mat<float, 2, 3> float2x3;
typedef mat<double, 2, 3> double2x3;
typedef mat<bool, 2, 4> bool2x4;
typedef mat<int, 2, 4> int2x4;
typedef mat<float, 2, 4> float2x4;
typedef mat<double, 2, 4> double2x4;
typedef mat<bool, 3, 2> bool3x2;
typedef mat<int, 3, 2> int3x2;
typedef mat<float, 3, 2> float3x2;
typedef mat<double, 3, 2> double3x2;
typedef mat<bool, 3, 3> bool3x3;
typedef mat<int, 3, 3> int3x3;
typedef mat<float, 3, 3> float3x3;
typedef mat<double, 3, 3> double3x3;
typedef mat<bool, 3, 4> bool3x4;
typedef mat<int, 3, 4> int3x4;
typedef mat<float, 3, 4> float3x4;
typedef mat<double, 3, 4> double3x4;
typedef mat<bool, 4, 2> bool4x2;
typedef mat<int, 4, 2> int4x2;
typedef mat<float, 4, 2> float4x2;
typedef mat<double, 4, 2> double4x2;
typedef mat<bool, 4, 3> bool4x3;
typedef mat<int, 4, 3> int4x3;
typedef mat<float, 4, 3> float4x3;
typedef mat<double, 4, 3> double4x3;
typedef mat<bool, 4, 4> bool4x4;
typedef mat<int, 4, 4> int4x4;
typedef mat<float, 4, 4> float4x4;
typedef mat<double, 4, 4> double4x4;
}  // namespace aliases
}  // namespace linalg

#endif
