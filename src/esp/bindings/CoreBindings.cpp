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
namespace config {

void initCoreBindings(py::module& m) {
  py::enum_<ConfigStoredType>(m, "ConfigStoredType")
      .value("Unknown", ConfigStoredType::Unknown)
      .value("Boolean", ConfigStoredType::Boolean)
      .value("Integer", ConfigStoredType::Integer)
      .value("Double", ConfigStoredType::Double)
      .value("String", ConfigStoredType::String)
      .value("MagnumVec3", ConfigStoredType::MagnumVec3)
      .value("MagnumQuat", ConfigStoredType::MagnumQuat)
      .value("MagnumRad", ConfigStoredType::MagnumRad);

  py::class_<Configuration, Configuration::ptr>(m, "Configuration")
      .def(py::init(&Configuration::create<>))

      .def(
          "get",
          [](Configuration& self, const std::string& key) {
            // switch on type
            switch (self.getType(key)) {
              case ConfigStoredType::Boolean:
                return py::cast(self.getBool(key));
              case ConfigStoredType::Integer:
                return py::cast(self.getInt(key));
              case ConfigStoredType::Double:
                return py::cast(self.getDouble(key));
              case ConfigStoredType::String:
                return py::cast(self.getString(key));
              case ConfigStoredType::MagnumVec3:
                return py::cast(self.getVec3(key));
              case ConfigStoredType::MagnumQuat:
                return py::cast(self.getQuat(key));
              case ConfigStoredType::MagnumRad:
                return py::cast(self.getRad(key));
              default:
                // unknown type or value not found
                throw py::value_error("No valid value found for key " + key);
                return py::cast(nullptr);
            }
            CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          },
          R"(Retrieve the requested value, if it exists)")

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

      .def(
          "get_type", &Configuration::getType,
          R"(Retrieves the ConfigStoredType of the value referred to by the passed key.)")

      .def(
          "get_as_string", &Configuration::getAsString,
          R"(Retrieves a string representation of the value referred to by the passed key.)")

      .def("get_bool_keys",
           &Configuration::getStoredKeys<ConfigStoredType::Boolean>)
      .def("get_string_keys",
           &Configuration::getStoredKeys<ConfigStoredType::String>)
      .def("get_int_keys",
           &Configuration::getStoredKeys<ConfigStoredType::Integer>)
      .def("get_float_keys",
           &Configuration::getStoredKeys<ConfigStoredType::Double>)
      .def("get_vec3_keys",
           &Configuration::getStoredKeys<ConfigStoredType::MagnumVec3>)
      .def("get_quat_keys",
           &Configuration::getStoredKeys<ConfigStoredType::MagnumQuat>)
      .def("get_rad_keys",
           &Configuration::getStoredKeys<ConfigStoredType::MagnumRad>)

      .def("get_keys_and_types", &Configuration::getValueTypes,
           R"(Returns a dictionary where the keys are the names of the values
           this configuration holds and the values are the types of these values.)")
      .def(
          "get_subconfig_keys", &Configuration::getSubconfigKeys,
          R"(Retrieves a list of the keys of this configuration's subconfigurations)")

      .def("get_subconfig", &Configuration::editSubconfig,
           pybind11::return_value_policy::reference,
           R"(Get the subconfiguration with the given name.)", "name"_a)
      .def("get_subconfig_copy", &Configuration::getSubconfigCopy,
           pybind11::return_value_policy::reference,
           R"(Get a copy of the subconfiguration with the given name.)",
           "name"_a)
      .def("save_subconfig", &Configuration::setSubconfigPtr,
           R"(Save a subconfiguration with the given name.)", "name"_a,
           "subconfig"_a)

      .def(
          "find_value_location", &Configuration::findValue,
          R"(Returns a list of keys, in order, for the traversal of the nested subconfigurations in
          this Configuration to get the requested key's value or subconfig.  Key is not found if list is empty.)",
          "key"_a)

      .def(
          "has_value", &Configuration::hasValue,
          R"(Returns whether or not this Configuration has the passed key. Does not check subconfigurations.)",
          "key"_a)

      .def(
          "has_bool",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key, ConfigStoredType::Boolean);
          },
          R"(Returns true if specified key references a boolean value in this configuration.)")
      .def(
          "has_int",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key, ConfigStoredType::Integer);
          },
          R"(Returns true if specified key references a integer value in this configuration.)")
      .def(
          "has_string",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key, ConfigStoredType::String);
          },
          R"(Returns true if specified key references a string value in this configuration.)")
      .def(
          "has_float",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key, ConfigStoredType::Double);
          },
          R"(Returns true if specified key references a float value in this configuration.)")
      .def(
          "has_quat",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key,
                                              ConfigStoredType::MagnumQuat);
          },
          R"(Returns true if specified key references a Magnum::Quaternion value in this configuration.)")
      .def(
          "has_vec3",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key,
                                              ConfigStoredType::MagnumVec3);
          },
          R"(Returns true if specified key references a Magnum::Vector3 value in this configuration.)")
      .def(
          "has_rad",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType(key, ConfigStoredType::MagnumRad);
          },
          R"(Returns true if specified key references a Magnum::Rad value in this configuration.)")

      .def(
          "has_subconfig", &Configuration::hasSubconfig,
          R"(Returns true if specified key references an existing subconfiguration within this configuration.)")

      .def(
          "remove",
          [](Configuration& self, const std::string& key) {
            // switch on type
            switch (self.getType(key)) {
              case ConfigStoredType::Boolean:
                return py::cast(self.removeBool(key));
              case ConfigStoredType::Integer:
                return py::cast(self.removeInt(key));
              case ConfigStoredType::Double:
                return py::cast(self.removeDouble(key));
              case ConfigStoredType::String:
                return py::cast(self.removeString(key));
              case ConfigStoredType::MagnumVec3:
                return py::cast(self.removeVec3(key));
              case ConfigStoredType::MagnumQuat:
                return py::cast(self.removeQuat(key));
              case ConfigStoredType::MagnumRad:
                return py::cast(self.removeRad(key));
              default:
                // unknown type or value not found
                throw py::value_error("No valid value found for key " + key);
                return py::cast(nullptr);
            }
            CORRADE_INTERNAL_ASSERT_UNREACHABLE();
          },
          R"(Retrieve and remove the requested value, if it exists)")

      .def(
          "remove_subconfig", &Configuration::removeSubconfig,
          R"(Removes and returns subconfiguration corresponding to passed key, if found. Gives warning otherwise.)");

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

}  // namespace config
}  // namespace core

}  // namespace esp
