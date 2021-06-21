// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/core/Configuration.h"
#include "esp/core/random.h"

namespace py = pybind11;
using py::literals::operator""_a;

namespace esp {
namespace core {

void initCoreBindings(py::module& m) {
  py::class_<Configuration, Configuration::ptr>(m, "ConfigurationGroup")
      .def(py::init(&Configuration::create<>))
      .def("get_bool", &Configuration::getBool)
      .def("get_string", &Configuration::getString)
      .def("get_int", &Configuration::getInt)
      .def("get_double", &Configuration::getDouble)
      .def("get_vec3", &Configuration::getVec3)
      .def("get_quat", &Configuration::getQuat)
      .def("get", &Configuration::getString)
      .def("set", &Configuration::set<std::string>)
      .def("set", &Configuration::set<int>)
      .def("set", &Configuration::set<double>)
      .def("set", &Configuration::set<bool>)
      .def("set", &Configuration::set<Magnum::Vector3>)
      .def("set", &Configuration::set<Magnum::Quaternion>)
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
