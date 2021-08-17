// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/core/Configuration.h"
#include "esp/core/random.h"

namespace py = pybind11;
using py::literals::operator""_a;
using esp::logging::LoggingContext;

namespace esp {
namespace core {

void initCoreBindings(py::module& m) {
  py::enum_<ConfigStoredType>(m, "ConfigStoredType")
      .value("Boolean", ConfigStoredType::Boolean)
      .value("Integer", ConfigStoredType::Integer)
      .value("Double", ConfigStoredType::Double)
      .value("String", ConfigStoredType::String)
      .value("MagnumVec3", ConfigStoredType::MagnumVec3)
      .value("MagnumQuat", ConfigStoredType::MagnumQuat)
      .value("MagnumRad", ConfigStoredType::MagnumRad);

  py::class_<Configuration, Configuration::ptr>(m, "ConfigurationGroup")
      .def(py::init(&Configuration::create<>))
      .def("get_bool", &Configuration::getBool)
      .def("get_string", &Configuration::getString)
      .def("get_int", &Configuration::getInt)
      // python floats are doubles
      .def("get_float", &Configuration::getDouble)
      .def("get_vec3", &Configuration::getVec3)
      .def("get_quat", &Configuration::getQuat)
      .def("get_rad", &Configuration::getRad)

      .def("get", &Configuration::getAsString)

      .def("get_bool_keys", &Configuration::getBoolKeys)
      .def("get_string_keys", &Configuration::getStringKeys)
      .def("get_int_keys", &Configuration::getIntKeys)
      .def("get_float_keys", &Configuration::getDoubleKeys)
      .def("get_vec3_keys", &Configuration::getVec3Keys)
      .def("get_quat_keys", &Configuration::getQuatKeys)
      .def("get_rad_keys", &Configuration::getRadKeys)
      .def("get_value_types", &Configuration::getValueTypes,
           R"(Returns a dictionary where the keys are the names of the values
           this configuration holds and the values are the types of these values.)")
      .def(
          "get_config_keys", &Configuration::getSubConfigKeys,
          R"(Retrieves a list of the keys of this configuration's subconfigurations)")

      .def("get_subconfig", &Configuration::editSubConfig,
           R"(Get the subconfiguration with the given name.)")
      .def("get_subconfig_copy", &Configuration::getSubConfigCopy,
           R"(Get a copy of the subconfiguration with the given name.)")
      .def("save_subconfig", &Configuration::setSubConfigPtr,
           R"(Save a subconfiguration with the given name.)", "name"_a,
           "subconfig"_a)

      .def("set", [](Configuration& self, const std::string& key,
                     const std::string& val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const char* val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const int val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const double val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const bool val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const Magnum::Quaternion& val) { self.set(key, val); })
      .def("set", [](Configuration& self, const std::string& key,
                     const Magnum::Vector3& val) { self.set(key, val); })

      .def("set_string", [](Configuration& self, const std::string& key,
                            const std::string& val) { self.set(key, val); })
      .def("set_int", [](Configuration& self, const std::string& key,
                         const int val) { self.set(key, val); })
      .def("set_float", [](Configuration& self, const std::string& key,
                           const double val) { self.set(key, val); })
      .def("set_bool", [](Configuration& self, const std::string& key,
                          const bool val) { self.set(key, val); })
      .def("set_quat",
           [](Configuration& self, const std::string& key,
              const Magnum::Quaternion& val) { self.set(key, val); })
      .def("set_vec3", [](Configuration& self, const std::string& key,
                          const Magnum::Vector3& val) { self.set(key, val); })
      .def("set_rad", [](Configuration& self, const std::string& key,
                         const Magnum::Rad& val) { self.set(key, val); })

      //.def("add_string_to_group", &Configuration::addStringToGroup)
      .def("has_value", &Configuration::hasValue,
           R"(Returns whether this Configuration has the passed key.)", "key"_a)
      .def("has_bool", &Configuration::hasBool)
      .def("has_int", &Configuration::hasInt)
      .def("has_string", &Configuration::hasString)
      .def("has_float", &Configuration::hasDouble)
      .def("has_quat", &Configuration::hasQuat)
      .def("has_vec3", &Configuration::hasVec3)
      .def("has_rad", &Configuration::hasRad)

      .def("remove_bool", &Configuration::removeBool)
      .def("remove_string", &Configuration::removeString)
      .def("remove_int", &Configuration::removeInt)
      .def("remove_float", &Configuration::removeDouble)
      .def("remove_vec3", &Configuration::removeVec3)
      .def("remove_quat", &Configuration::removeQuat)
      .def("remove_rad", &Configuration::removeRad);

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
}

}  // namespace core

}  // namespace esp
