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
    case ConfigValType::MagnumRad:
      return py::cast(static_cast<Mn::Radd>(value.get<Mn::Rad>()));
    case ConfigValType::MagnumDeg:
      return py::cast(static_cast<Mn::Degd>(value.get<Mn::Deg>()));
    case ConfigValType::Double:
      return py::cast(value.get<double>());
    case ConfigValType::MagnumVec2:
      return py::cast(value.get<Mn::Vector2>());
    case ConfigValType::MagnumVec2i:
      return py::cast(value.get<Mn::Vector2i>());
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
    case ConfigValType::String:
      return py::cast(value.get<std::string>());
  }
  return py::cast(nullptr);
}

template <class T>
void declareSetter(
    pybind11::class_<esp::core::config::Configuration,
                     esp::core::config::Configuration::ptr>& pyAbsAttr,
    const std::string& typeName) {
  pyAbsAttr.def(
      "set",
      [](Configuration& self, const std::string& key, const T val) {
        self.set(key, val);
      },
      ("Set the value specified by given string key to be specified " +
       typeName + " value.")
          .c_str(),
      "key"_a, "value"_a);
}  // declareSetter

template <class T>
void declareInitializer(
    pybind11::class_<esp::core::config::Configuration,
                     esp::core::config::Configuration::ptr>& pyAbsAttr,
    const std::string& typeName) {
  pyAbsAttr.def(
      "init",
      [](Configuration& self, const std::string& key, const T val) {
        self.init(key, val);
      },
      ("Initialize the value specified by given string key to be specified " +
       typeName + " value.")
          .c_str(),
      "key"_a, "value"_a);
}  // declareInitializer

void initConfigBindings(py::module& m) {
  py::enum_<ConfigValType>(m, "ConfigValType")
      .value("Unknown", ConfigValType::Unknown)
      .value("Boolean", ConfigValType::Boolean)
      .value("Integer", ConfigValType::Integer)
      .value("MagnumRad", ConfigValType::MagnumRad)
      .value("MagnumDeg", ConfigValType::MagnumDeg)
      .value("Float", ConfigValType::Double)
      .value("MagnumVec2", ConfigValType::MagnumVec2)
      .value("MagnumVec2i", ConfigValType::MagnumVec2i)
      .value("MagnumVec3", ConfigValType::MagnumVec3)
      .value("MagnumVec4", ConfigValType::MagnumVec4)
      .value("MagnumQuat", ConfigValType::MagnumQuat)
      .value("MagnumMat3", ConfigValType::MagnumMat3)
      .value("MagnumMat4", ConfigValType::MagnumMat4)
      .value("String", ConfigValType::String);

  auto pyConfiguration =
      py::class_<Configuration, Configuration::ptr>(m, "Configuration");
  pyConfiguration.def(py::init(&Configuration::create<>))
      .def("__repr__", &Configuration::getAllValsAsString, "new_line"_a = "\n")
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
          "has_key_to_type", &Configuration::hasKeyToValOfType,
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
          "has_subconfig",
          static_cast<bool (Configuration::*)(const std::string&) const>(
              &Configuration::hasSubconfig),
          R"(Returns true if specified key references an existing subconfiguration within this configuration.)")
      .def(
          "remove_subconfig", &Configuration::removeSubconfig,
          R"(Removes and returns subconfiguration corresponding to passed key, if found. Gives warning otherwise.)");
  // Setter bindings
  declareSetter<std::string&>(pyConfiguration, "string");
  declareSetter<bool>(pyConfiguration, "boolean");
  declareSetter<int>(pyConfiguration, "integer");
  declareSetter<double>(pyConfiguration, "floating-point");
  declareSetter<Magnum::Vector2&>(pyConfiguration, "Magnum::Vector2");
  declareSetter<Magnum::Vector2i&>(pyConfiguration, "Magnum::Vector2i");
  declareSetter<Magnum::Vector3&>(pyConfiguration, "Magnum::Vector3");
  declareSetter<Magnum::Vector4&>(pyConfiguration, "Magnum::Vector4");
  declareSetter<Magnum::Color4&>(pyConfiguration, "Magnum::Color4");
  declareSetter<Magnum::Quaternion&>(pyConfiguration, "Magnum::Quaternion");
  declareSetter<Magnum::Matrix3&>(pyConfiguration, "Magnum::Matrix3");
  declareSetter<Magnum::Matrix4&>(pyConfiguration, "Magnum::Matrix4");
  // Use Radd version for bindings
  pyConfiguration.def(
      "set",
      [](Configuration& self, const std::string& key, const Mn::Radd val) {
        self.set(key, static_cast<Mn::Rad>(val));
      },
      "Set the value specified by given string key to be specified Magnum::Rad "
      "value.",
      "key"_a, "value"_a);
  // Use Degd version for bindings
  pyConfiguration.def(
      "set",
      [](Configuration& self, const std::string& key, const Mn::Degd val) {
        self.set(key, static_cast<Mn::Deg>(val));
      },
      "Set the value specified by given string key to be specified Magnum::Deg "
      "value.",
      "key"_a, "value"_a);
  // Initializer bindings
  // Initializers are like setters but the value specified will not be
  // automatically saved to file unless it is changed.
  declareInitializer<std::string&>(pyConfiguration, "string");
  declareInitializer<bool>(pyConfiguration, "boolean");
  declareInitializer<int>(pyConfiguration, "integer");
  declareInitializer<double>(pyConfiguration, "floating-point");
  declareInitializer<Magnum::Vector2&>(pyConfiguration, "Magnum::Vector2");
  declareInitializer<Magnum::Vector2i&>(pyConfiguration, "Magnum::Vector2i");
  declareInitializer<Magnum::Vector3&>(pyConfiguration, "Magnum::Vector3");
  declareInitializer<Magnum::Vector4&>(pyConfiguration, "Magnum::Vector4");
  declareInitializer<Magnum::Color4&>(pyConfiguration, "Magnum::Color4");
  declareInitializer<Magnum::Quaternion&>(pyConfiguration,
                                          "Magnum::Quaternion");
  declareInitializer<Magnum::Matrix3&>(pyConfiguration, "Magnum::Matrix3");
  declareInitializer<Magnum::Matrix4&>(pyConfiguration, "Magnum::Matrix4");
  // Use Radd version for bindings
  pyConfiguration.def(
      "init",
      [](Configuration& self, const std::string& key, const Mn::Radd val) {
        self.init(key, static_cast<Mn::Rad>(val));
      },
      "Initialize the value specified by given string key to be specified "
      "Magnum::Rad value.",
      "key"_a, "value"_a);
  // Use Degd version for bindings
  pyConfiguration.def(
      "init",
      [](Configuration& self, const std::string& key, const Mn::Degd val) {
        self.init(key, static_cast<Mn::Deg>(val));
      },
      "Initialize the value specified by given string key to be specified "
      "Magnum::Deg value.",
      "key"_a, "value"_a);

}  // initConfigBindings

}  // namespace config
}  // namespace core

}  // namespace esp
