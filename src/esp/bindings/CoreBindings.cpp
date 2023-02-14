// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"
#include "esp/core/Random.h"
#include "esp/core/Utility.h"

namespace py = pybind11;
using py::literals::operator""_a;
using esp::logging::LoggingContext;

namespace esp {
namespace core {

void initCoreBindings(py::module& m) {
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

  auto core = m.def_submodule("core");
  py::class_<LoggingContext>(core, "LoggingContext")
      .def_static(
          "reinitialize_from_env",
          []() {
            auto core = py::module_::import(
                "habitat_sim._ext.habitat_sim_bindings.core");
            delete core.attr("_logging_context").cast<LoggingContext*>();
            core.attr("_logging_context") = new LoggingContext{};
          })
      .def_static("current", &LoggingContext::current,
                  py::return_value_policy::reference)
      .def_property_readonly("sim_is_quiet", [](LoggingContext& self) -> bool {
        return self.levelFor(logging::Subsystem::sim) >=
               logging::LoggingLevel::Debug;
      });
  core.attr("_logging_context") = new LoggingContext{};

  core.def("orthonormalize_rotation_shear",
           &orthonormalizeRotationShear<float>);
  core.def("orthonormalize_rotation_shear",
           &orthonormalizeRotationShear<double>);
}
}  // namespace core

}  // namespace esp
