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

#include <Magnum/EigenIntegration/GeometryIntegration.h>
#include <Magnum/EigenIntegration/Integration.h>
#include <Magnum/Math/Range.h>
#include <Magnum/Math/RectangularMatrix.h>

#include "esp/core/configure.h"

namespace Eigen {

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

// EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f))

#endif  // ESP_CORE_ESPEIGEN_H_
