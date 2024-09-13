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

namespace Eigen {

typedef Matrix<float, Dynamic, Dynamic, RowMajor> RowMatrixXf;

}  // namespace Eigen

#endif  // ESP_CORE_ESPEIGEN_H_
