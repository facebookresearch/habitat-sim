// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/bindings.h"
#include "esp/core/Configuration.h"
namespace py = pybind11;
using py::literals::operator""_a;
using esp::logging::LoggingContext;

namespace esp {
namespace core {
namespace config {

void initConfigBindings(py::module& m) {
  py::enum_<ConfigStoredType>(m, "ConfigStoredType")
      .value("Unknown", ConfigStoredType::Unknown)
      .value("Boolean", ConfigStoredType::Boolean)
      .value("Integer", ConfigStoredType::Integer)
      .value("Float", ConfigStoredType::Double)
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
                return py::cast(self.get<bool>(key));
              case ConfigStoredType::Integer:
                return py::cast(self.get<int>(key));
              case ConfigStoredType::Double:
                return py::cast(self.get<double>(key));
              case ConfigStoredType::String:
                return py::cast(self.get<std::string>(key));
              case ConfigStoredType::MagnumVec3:
                return py::cast(self.get<Mn::Vector3>(key));
              case ConfigStoredType::MagnumQuat:
                return py::cast(self.get<Mn::Quaternion>(key));
              case ConfigStoredType::MagnumRad:
                return py::cast(self.get<Mn::Rad>(key));
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
           py::return_value_policy::reference_internal,
           R"(Get the subconfiguration with the given name.)", "name"_a)
      .def("get_subconfig_copy", &Configuration::getSubconfigCopy,
           py::return_value_policy::reference,
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
            return self.checkMapForKeyAndType<bool>(key);
          },
          R"(Returns true if specified key references a boolean value in this configuration.)")
      .def(
          "has_int",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<int>(key);
          },
          R"(Returns true if specified key references a integer value in this configuration.)")
      .def(
          "has_string",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<std::string>(key);
          },
          R"(Returns true if specified key references a string value in this configuration.)")
      .def(
          "has_float",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<double>(key);
          },
          R"(Returns true if specified key references a float value in this configuration.)")
      .def(
          "has_quat",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<Mn::Quaternion>(key);
          },
          R"(Returns true if specified key references a Magnum::Quaternion value in this configuration.)")
      .def(
          "has_vec3",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<Mn::Vector3>(key);
          },
          R"(Returns true if specified key references a Magnum::Vector3 value in this configuration.)")
      .def(
          "has_rad",
          [](Configuration& self, const std::string& key) {
            return self.checkMapForKeyAndType<Mn::Rad>(key);
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
                return py::cast(self.removeAndRetrieve<bool>(key));
              case ConfigStoredType::Integer:
                return py::cast(self.removeAndRetrieve<int>(key));
              case ConfigStoredType::Double:
                return py::cast(self.removeAndRetrieve<double>(key));
              case ConfigStoredType::String:
                return py::cast(self.removeAndRetrieve<std::string>(key));
              case ConfigStoredType::MagnumVec3:
                return py::cast(self.removeAndRetrieve<Mn::Vector3>(key));
              case ConfigStoredType::MagnumQuat:
                return py::cast(self.removeAndRetrieve<Mn::Quaternion>(key));
              case ConfigStoredType::MagnumRad:
                return py::cast(self.removeAndRetrieve<Mn::Rad>(key));
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

}  // initConfigBindings

}  // namespace config
}  // namespace core

}  // namespace esp
