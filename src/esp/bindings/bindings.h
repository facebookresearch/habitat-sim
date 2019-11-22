// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <pybind11/pybind11.h>
#include "esp/bindings/OpaqueTypes.h"

namespace esp {

namespace geo {
void initGeoBindings(pybind11::module& m);
}

namespace gfx {
void initGfxBindings(pybind11::module& m);
}

namespace nav {
void initShortestPathBindings(pybind11::module& m);
}

namespace scene {
void initSceneBindings(pybind11::module& m);
}

namespace sensor {
void initSensorBindings(pybind11::module& m);
}

}  // namespace esp
