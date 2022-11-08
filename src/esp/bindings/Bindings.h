// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_BINDINGS_BINDINGS_H_
#define ESP_BINDINGS_BINDINGS_H_

#include <pybind11/pybind11.h>
#include "esp/bindings/OpaqueTypes.h"

namespace esp {

namespace core {

void initCoreBindings(pybind11::module& m);
namespace config {

void initConfigBindings(pybind11::module& m);
}  // namespace config
}  // namespace core

namespace geo {
void initGeoBindings(pybind11::module& m);
}

namespace gfx {
void initGfxBindings(pybind11::module& m);
namespace replay {
void initGfxReplayBindings(pybind11::module& m);
}  // namespace replay
}  // namespace gfx

namespace metadata {
void initAttributesBindings(pybind11::module& m);
void initMetadataMediatorBindings(pybind11::module& m);
namespace managers {
void initAttributesManagersBindings(pybind11::module& m);
}  // namespace managers
}  // namespace metadata

namespace nav {
void initShortestPathBindings(pybind11::module& m);
}

namespace physics {
void initPhysicsBindings(pybind11::module& m);

void initPhysicsObjectBindings(pybind11::module& m);

void initPhysicsWrapperManagerBindings(pybind11::module& m);
}  // namespace physics

namespace scene {
void initSceneBindings(pybind11::module& m);
}

namespace sensor {
void initSensorBindings(pybind11::module& m);
}

namespace sim {
void initSimBindings(pybind11::module& m);
}

}  // namespace esp

#endif  // ESP_BINDINGS_BINDINGS_H_
