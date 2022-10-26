// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_ESPEIGEN_H_
#define ESP_CORE_ESPEIGEN_H_

/** @file */

// Eigen has an enum that clashes with X11 Success define
#ifdef Success
#undef Success
#endif
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <Eigen/StdVector>

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/RectangularMatrix.h>

#include "esp/core/configure.h"

namespace Eigen {
typedef Matrix<uint8_t, 3, 1> Vector3uc;
typedef Matrix<uint32_t, 3, 1> Vector3ui;
typedef Matrix<uint8_t, 4, 1> Vector4uc;
typedef Matrix<uint32_t, 4, 1> Vector4ui;
typedef Matrix<uint64_t, 4, 1> Vector4ul;

typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMatrixXf;

//! Eigen JSON string format specification
static const IOFormat kJsonFormat(StreamPrecision,
                                  DontAlignCols,
                                  ",",   // coef separator
                                  ",",   // row separator
                                  "",    // row prefix
                                  "",    // col prefix
                                  "[",   // mat prefix
                                  "]");  // mat suffix

template <typename T, int numRows, int numCols>
std::ostream& operator<<(std::ostream& os,
                         const Matrix<T, numRows, numCols>& matrix) {
  return os << matrix.format(kJsonFormat);
}

//! Write Eigen matrix types into ostream in JSON string format
template <typename T, int numRows, int numCols>
typename std::enable_if<numRows == Dynamic || numCols == Dynamic,
                        Corrade::Utility::Debug&>::type
operator<<(Corrade::Utility::Debug& os,
           const Matrix<T, numRows, numCols>& matrix) {
  return os << matrix.format(kJsonFormat);
}

template <typename T, int numRows, int numCols>
typename std::enable_if<(numRows != Dynamic && numCols != Dynamic) &&
                            (numRows != 1 && numCols != 1),
                        Corrade::Utility::Debug&>::type
operator<<(Corrade::Utility::Debug& os,
           const Matrix<T, numRows, numCols>& matrix) {
  return os << Magnum::Math::RectangularMatrix<numRows, numCols, T>{matrix};
}

template <typename T, int numRows, int numCols>
typename std::enable_if<(numRows != Dynamic && numCols != Dynamic) &&
                            (numRows == 1 || numCols == 1),
                        Corrade::Utility::Debug&>::type
operator<<(Corrade::Utility::Debug& os,
           const Matrix<T, numRows, numCols>& matrix) {
  return os << Magnum::Math::Vector<(numRows == 1 ? numCols : numRows), T>{
             matrix};
}

//! Write Eigen map into ostream in JSON string format
template <typename T>
std::ostream& operator<<(std::ostream& os, const Map<T>& m) {
  return os << m.format(kJsonFormat);
}

}  // namespace Eigen

// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4i)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4f)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d)
// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector4uc)

//! core simulator namespace
namespace esp {

// basic types
typedef Eigen::Vector2f vec2f;
typedef Eigen::Vector3f vec3f;
typedef Eigen::Vector4f vec4f;
typedef Eigen::Vector2d vec2d;
typedef Eigen::Vector3d vec3d;
typedef Eigen::Vector4d vec4d;
typedef Eigen::Vector2i vec2i;
typedef Eigen::Vector3i vec3i;
typedef Eigen::Vector4i vec4i;
typedef Eigen::Matrix3f mat3f;
typedef Eigen::Matrix4f mat4f;
typedef Eigen::Matrix3d mat3d;
typedef Eigen::Matrix4d mat4d;
typedef Eigen::Quaternionf quatf;
typedef Eigen::Vector3uc vec3uc;
typedef Eigen::Vector3ui vec3ui;
typedef Eigen::Vector4uc vec4uc;
typedef Eigen::Vector4ui vec4ui;
typedef Eigen::Vector4i vec4i;
typedef Eigen::Vector4ul vec4ul;
typedef Eigen::VectorXi vecXi;
typedef Eigen::AlignedBox3f box3f;

typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform;

//! Write box3f into ostream in JSON string format
inline std::ostream& operator<<(std::ostream& os, const box3f& bbox) {
  return os << "{min:" << bbox.min() << ",max:" << bbox.max() << "}";
}

//! Write box3f as a magnum range
inline Corrade::Utility::Debug& operator<<(Corrade::Utility::Debug& os,
                                           const box3f& bbox) {
  return os << Magnum::Range3D{bbox};
}

}  // namespace esp

#endif  // ESP_CORE_ESPEIGEN_H_
