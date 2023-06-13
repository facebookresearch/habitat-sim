// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"

#include "esp/assets/ResourceManager.h"
#include "esp/core/Check.h"
#include "esp/core/Configuration.h"
#include "esp/core/RigidState.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {

void initEspBindings(py::module& m) {
  // ==== box3f ====
  py::class_<box3f>(m, "BBox")
      .def_property_readonly("sizes", &box3f::sizes)
      .def_property_readonly("center", &box3f::center);
}

}  // namespace esp

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  m.attr("built_with_bullet") =
#ifdef ESP_BUILD_WITH_BULLET
      true;
#else
      false;
#endif

  m.attr("audio_enabled") =
#ifdef ESP_BUILD_WITH_AUDIO
      true;
#else
      false;
#endif

  /* This function pointer is used by ESP_CHECK(). If it's null, it
     std::abort()s, if not, it calls it to cause a Python AssertionError */
  esp::core::throwInPython = [](const char* const message) {
    PyErr_SetString(PyExc_AssertionError, message);
    throw pybind11::error_already_set{};
  };

  py::module_::import("magnum.scenegraph");

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  // NOTE(msb) These need to be run in dependency order.
  // TODO(msb) gfx, scene, and sensor should not cross-depend
  // TODO(msb) sim and sensor should not cross-depend
  esp::initEspBindings(m);
  esp::core::config::initConfigBindings(m);
  esp::core::initCoreBindings(m);
  esp::geo::initGeoBindings(m);
  esp::scene::initSceneBindings(m);
  esp::gfx::initGfxBindings(m);
  esp::gfx::replay::initGfxReplayBindings(m);
  esp::sensor::initSensorBindings(m);
  esp::nav::initShortestPathBindings(m);
  esp::metadata::initAttributesBindings(m);
  esp::metadata::initMetadataMediatorBindings(m);
  esp::metadata::managers::initAttributesManagersBindings(m);
  // These depend on SceneNode bindings
  esp::physics::initPhysicsBindings(m);
  esp::physics::initPhysicsObjectBindings(m);
  esp::physics::initPhysicsWrapperManagerBindings(m);
  esp::sim::initSimBindings(m);
}
