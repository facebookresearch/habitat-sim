// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "esp/bindings/Bindings.h"
#include "esp/core/Configuration.h"
namespace py = pybind11;
using py::literals::operator""_a;
using esp::logging::LoggingContext;

namespace esp {
namespace core {
namespace config {

py::object getObjectForConfigValue(const ConfigValue& value) {
  switch (value.getType()) {
    case ConfigValType::Unknown:
      return py::cast(nullptr);
    case ConfigValType::Boolean:
      return py::cast(value.get<bool>());
    case ConfigValType::Integer:
      return py::cast(value.get<int>());
    case ConfigValType::Double:
      return py::cast(value.get<double>());
    case ConfigValType::String:
      return py::cast(value.get<std::string>());
    case ConfigValType::MagnumVec2:
      return py::cast(value.get<Mn::Vector2>());
    case ConfigValType::MagnumVec3:
      return py::cast(value.get<Mn::Vector3>());
    case ConfigValType::MagnumVec4:
      return py::cast(value.get<Mn::Vector4>());
    case ConfigValType::MagnumMat3:
      return py::cast(value.get<Mn::Matrix3>());
    case ConfigValType::MagnumMat4:
      return py::cast(value.get<Mn::Matrix4>());
    case ConfigValType::MagnumQuat:
      return py::cast(value.get<Mn::Quaternion>());
    case ConfigValType::MagnumRad:
      return py::cast(value.get<Mn::Rad>());
  }
  return py::cast(nullptr);
}

void initConfigBindings(py::module& m) {
  py::enum_<ConfigValType>(m, "ConfigValType")
      .value("Unknown", ConfigValType::Unknown)
      .value("Boolean", ConfigValType::Boolean)
      .value("Integer", ConfigValType::Integer)
      .value("Float", ConfigValType::Double)
      .value("String", ConfigValType::String)
      .value("MagnumVec2", ConfigValType::MagnumVec2)
      .value("MagnumVec3", ConfigValType::MagnumVec3)
      .value("MagnumVec4", ConfigValType::MagnumVec4)
      .value("MagnumMat3", ConfigValType::MagnumMat3)
      .value("MagnumMat4", ConfigValType::MagnumMat4)
      .value("MagnumQuat", ConfigValType::MagnumQuat)
      .value("MagnumRad", ConfigValType::MagnumRad);

  py::class_<Configuration, Configuration::ptr>(m, "Configuration")
      .def(py::init(&Configuration::create<>))
      .def_property_readonly(
          "top_level_num_entries", &Configuration::getNumEntries,
          R"(Holds the total number of values and subconfigs this Configuration holds
          at the base level (does not recurse subconfigs).)")
      .def_property_readonly(
          "top_level_num_configs", &Configuration::getNumSubconfigs,
          R"(Holds the total number of subconfigs this Configuration holds at the base
          level (does not recurse subconfigs).)")
      .def_property_readonly(
          "top_level_num_values", &Configuration::getNumValues,
          R"(Holds the total number of values this Configuration holds at the base
          level (does not recurse subconfigs).)")
      .def_property_readonly(
          "total_num_entries", &Configuration::getConfigTreeNumEntries,
          R"(Holds the total number of values and subconfigs this Configuration holds
          across all levels (recurses subconfigs).)")
      .def_property_readonly(
          "total_num_configs", &Configuration::getConfigTreeNumSubconfigs,
          R"(Holds the total number of subconfigs this Configuration holds across all
          levels (recurses subconfigs).)")
      .def_property_readonly(
          "total_num_values", &Configuration::getConfigTreeNumValues,
          R"(Holds the total number of values this Configuration holds across all
          levels (recurses subconfigs).)")
      .def(
          "get",
          [](Configuration& self, const std::string& key) {
            return getObjectForConfigValue(self.get(key));
          },
          R"(Retrieve the requested value referenced by key argument, if it exists)")

      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const std::string& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified string value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key, const char* val) {
            self.set(key, val);
          },
          R"(Set the value specified by given string key to be specified string value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key, const int val) {
            self.set(key, val);
          },
          R"(Set the value specified by given string key to be specified integer value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key, const double val) {
            self.set(key, val);
          },
          R"(Set the value specified by given string key to be specified double value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key, const bool val) {
            self.set(key, val);
          },
          R"(Set the value specified by given string key to be specified boolean value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Quaternion& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Quaternion value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Vector2& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Vector2 value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Vector3& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Vector3 value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Vector4& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Vector4 value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Matrix3& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Matrix3 value)",
          "key"_a, "value"_a)
      .def(
          "set",
          [](Configuration& self, const std::string& key,
             const Magnum::Matrix4& val) { self.set(key, val); },
          R"(Set the value specified by given string key to be specified Magnum::Matrix4 value)",
          "key"_a, "value"_a)
      .def(
          "get_type", &Configuration::getType,
          R"(Retrieves the ConfigValType of the value referred to by the passed key.)")

      .def(
          "get_as_string", &Configuration::getAsString,
          R"(Retrieves a string representation of the value referred to by the passed key.)")

      .def(
          "get_keys_by_type", &Configuration::getKeysByType,
          R"(Retrieves a list of all the keys of values of the specified types. Takes ConfigValType
          enum value as argument, and whether the keys should be sorted or not.)",
          "value_type"_a, "sorted"_a = false)

      .def("get_keys_and_types", &Configuration::getValueTypes,
           R"(Returns a dictionary where the keys are the names of the values
           this configuration holds and the values are the types of these values.)")

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
          "has_key_to_type", &Configuration::hasKeyOfType,
          R"(Returns whether passed key points to a value of specified ConfigValType)",
          "key"_a, "value_type"_a)
      .def(
          "remove",
          [](Configuration& self, const std::string& key) {
            return getObjectForConfigValue(self.remove(key));
          },
          R"(Retrieve and remove the requested value, if it exists)")

      // subconfigs
      .def(
          "get_subconfig_keys", &Configuration::getSubconfigKeys,
          R"(Retrieves a list of the keys of this configuration's subconfigurations,
          specifying whether the keys should be sorted or not)",
          "sorted"_a = false)

      .def("get_subconfig", &Configuration::editSubconfig<Configuration>,
           py::return_value_policy::reference_internal,
           R"(Get the subconfiguration with the given name.)", "name"_a)
      .def("get_subconfig_copy",
           &Configuration::getSubconfigCopy<Configuration>,
           py::return_value_policy::reference,
           R"(Get a copy of the subconfiguration with the given name.)",
           "name"_a)
      .def("save_subconfig", &Configuration::setSubconfigPtr<Configuration>,
           R"(Save a subconfiguration with the given name.)", "name"_a,
           "subconfig"_a)
      .def(
          "has_subconfig", &Configuration::hasSubconfig,
          R"(Returns true if specified key references an existing subconfiguration within this configuration.)")
      .def(
          "remove_subconfig", &Configuration::removeSubconfig,
          R"(Removes and returns subconfiguration corresponding to passed key, if found. Gives warning otherwise.)")
      .def("__repr__", &Configuration::getAllValsAsString, "new_line"_a = "\n");

}  // initConfigBindings

}  // namespace config
}  // namespace core

}  // namespace esp
