// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/OpaqueTypes.h"

namespace py = pybind11;
using namespace py::literals;

#include "esp/geo/geo.h"

void initGeoBindings(py::module& m) {
  auto geo = m.def_submodule("geo");

  geo.attr("UP") = esp::geo::ESP_UP;
  geo.attr("GRAVITY") = esp::geo::ESP_GRAVITY;
  geo.attr("FRONT") = esp::geo::ESP_FRONT;
  geo.attr("BACK") = esp::geo::ESP_BACK;
  geo.attr("LEFT") = esp::geo::ESP_FRONT.cross(esp::geo::ESP_GRAVITY);
  geo.attr("RIGHT") = esp::geo::ESP_FRONT.cross(esp::geo::ESP_UP);
}
