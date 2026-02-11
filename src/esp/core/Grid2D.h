// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_GRID2D_H_
#define ESP_CORE_GRID2D_H_

#include <Corrade/Containers/Array.h>

namespace esp {
namespace core {

/**
 * @brief Lightweight 2D grid backed by a contiguous row-major
 * Corrade::Containers::Array.
 *
 * Replaces Eigen dynamic matrices for simple 2D grid data such as
 * top-down navigability maps.
 */
template <typename T>
class Grid2D {
 public:
  Grid2D() : rows_{0}, cols_{0} {}

  Grid2D(int rows, int cols)
      : rows_{rows},
        cols_{cols},
        data_{Corrade::ValueInit, static_cast<std::size_t>(rows * cols)} {}

  T& operator()(int row, int col) { return data_[row * cols_ + col]; }
  const T& operator()(int row, int col) const {
    return data_[row * cols_ + col];
  }

  int rows() const { return rows_; }
  int cols() const { return cols_; }

  T* data() { return data_.data(); }
  const T* data() const { return data_.data(); }

 private:
  int rows_;
  int cols_;
  Corrade::Containers::Array<T> data_;
};

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_GRID2D_H_
