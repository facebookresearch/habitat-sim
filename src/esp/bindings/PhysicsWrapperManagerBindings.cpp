// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace physics {

void initPhysicsWrapperManagerBindings(pybind11::module& m) {}

}  // namespace physics
}  // namespace esp