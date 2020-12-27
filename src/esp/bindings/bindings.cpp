// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/core//random.h"
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

namespace core {

void initCoreBindings(py::module& m) {
  py::class_<Configuration, Configuration::ptr>(m, "ConfigurationGroup")
      .def(py::init(&Configuration::create<>))
      .def("get_bool", &Configuration::getBool)
      .def("get_string", &Configuration::getString)
      .def("get_int", &Configuration::getInt)
      .def("get_double", &Configuration::getDouble)
      .def("get_vec3", &Configuration::getVec3)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<double>)
      .def("set", &Configuration::set<bool>)
      .def("set", &Configuration::set<Magnum::Vector3>)
      .def("add_string_to_group", &Configuration::addStringToGroup)
      .def("get_string_group", &Configuration::getStringGroup)
      .def("has_value", &Configuration::hasValue)
      .def("remove_value", &Configuration::removeValue);

  // ==== struct RigidState ===
  py::class_<RigidState, RigidState::ptr>(m, "RigidState")
      .def(py::init(&RigidState::create<>))
      .def(py::init(&RigidState::create<const Magnum::Quaternion&,
                                        const Magnum::Vector3&>))
      .def_readwrite("rotation", &RigidState::rotation)
      .def_readwrite("translation", &RigidState::translation);

  py::class_<Random, Random::ptr>(m, "Random")
      .def(py::init(&Random::create<>))
      .def("seed", &Random::seed)
      .def("uniform_float_01", &Random::uniform_float_01)
      .def("uniform_float", &Random::uniform_float)
      .def("uniform_int", py::overload_cast<>(&Random::uniform_int))
      .def("uniform_int", py::overload_cast<int, int>(&Random::uniform_int))
      .def("uniform_uint", &Random::uniform_uint)
      .def("normal_float_01", &Random::normal_float_01);
}

}  // namespace core
}  // namespace esp

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  m.import("magnum.scenegraph");

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");

  // NOTE(msb) These need to be run in dependency order.
  // TODO(msb) gfx, scene, and sensor should not cross-depend
  // TODO(msb) sim and sensor should not cross-depend
  esp::initEspBindings(m);
  esp::core::initCoreBindings(m);
  esp::metadata::initAttributesBindings(m);
  esp::metadata::initMetadataMediatorBindings(m);
  esp::metadata::managers::initAttributesManagersBindings(m);
  esp::geo::initGeoBindings(m);
  esp::physics::initPhysicsBindings(m);
  esp::scene::initSceneBindings(m);
  esp::gfx::initGfxBindings(m);
  esp::gfx::replay::initGfxReplayBindings(m);
  esp::sensor::initSensorBindings(m);
  esp::nav::initShortestPathBindings(m);
  esp::sim::initSimBindings(m);
}
