// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"

#include "esp/core/Configuration.h"
#include "esp/physics/PhysicsManager.h"

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
  py::class_<Configuration, Configuration::ptr>(m, "Configuration")
      .def(py::init(&Configuration::create<>))
      .def("getBool", &Configuration::getBool)
      .def("getString", &Configuration::getString)
      .def("getInt", &Configuration::getInt)
      .def("getFloat", &Configuration::getFloat)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<float>)
      .def("set", &Configuration::set<bool>);
}

}  // namespace core

namespace physics {

void initPhysicsBindings(py::module& m) {
  // ==== enum object MotionType ====
  py::enum_<MotionType>(m, "MotionType")
      .value("ERROR_MOTIONTYPE", MotionType::ERROR_MOTIONTYPE)
      .value("STATIC", MotionType::STATIC)
      .value("KINEMATIC", MotionType::KINEMATIC)
      .value("DYNAMIC", MotionType::DYNAMIC);
}

}  // namespace physics
}  // namespace esp

PYBIND11_MODULE(habitat_sim_bindings, m) {
  m.attr("cuda_enabled") =
#ifdef ESP_BUILD_WITH_CUDA
      true;
#else
      false;
#endif

  m.import("magnum.scenegraph");

  esp::initEspBindings(m);
  esp::core::initCoreBindings(m);
  esp::geo::initGeoBindings(m);
  esp::gfx::initGfxBindings(m);
  esp::nav::initShortestPathBindings(m);
  esp::physics::initPhysicsBindings(m);
  esp::scene::initSceneBindings(m);
  esp::sensor::initSensorBindings(m);

  py::bind_map<std::map<std::string, std::string>>(m, "MapStringString");
}
