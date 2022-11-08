// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONUTILS_H_
#define ESP_IO_JSONUTILS_H_

namespace esp {
namespace io {

// when writing to json, we often want to clamp tiny floats to 0.0 so they can
// be serialized concisely as "0.0"
template <typename T>
T squashTinyDecimals(T x) {
  const T eps = 1e-6f;
  return (x < eps && x > -eps) ? T(0.0) : x;
}

}  // namespace io
}  // namespace esp

#endif
