// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CONFIGURATION_H_
#define ESP_CORE_CONFIGURATION_H_

#include <Corrade/Utility/Configuration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <string>

#include "esp/core/esp.h"

namespace esp {
namespace core {

class Configuration {
 public:
  // virtual destructor set to that pybind11 recognizes attributes inheritance
  // from configuration to be polymorphic
  virtual ~Configuration() = default;

  template <typename T>
  bool set(const std::string& key, const T& value) {
    return cfg.setValue(key, value);
  }

  // subgroup set
  template <typename T>
  bool setSubgroupValue(const std::string& subgroupName,
                        const std::string& key,
                        const T& value) {
    addNewSubgroup(subgroupName);
    return cfg.group(subgroupName)->setValue(key, value);
  }

  bool setBool(const std::string& key, bool value) { return set(key, value); }
  bool setFloat(const std::string& key, float value) { return set(key, value); }
  bool setDouble(const std::string& key, double value) {
    return set(key, value);
  }
  bool setInt(const std::string& key, int value) { return set(key, value); }
  bool setString(const std::string& key, const std::string& value) {
    return set(key, value);
  }
  bool setVec3(const std::string& key, const Magnum::Vector3& value) {
    return set(key, value);
  }
  bool setQuat(const std::string& key, const Magnum::Quaternion& value) {
    return set(key, value);
  }
  bool setRad(const std::string& key, Magnum::Rad value) {
    return set(key, value);
  }

  template <typename T>
  T get(const std::string& key) const {
    return cfg.value<T>(key);
  }

  // subgroup get
  template <typename T>
  T getSubgroupValue(const std::string& subgroupName, const std::string& key) {
    addNewSubgroup(subgroupName);
    return cfg.group(subgroupName)->value<T>(key);
  }

  bool getBool(const std::string& key) const { return get<bool>(key); }
  float getFloat(const std::string& key) const { return get<float>(key); }
  double getDouble(const std::string& key) const { return get<double>(key); }
  int getInt(const std::string& key) const { return get<int>(key); }
  std::string getString(const std::string& key) const {
    return get<std::string>(key);
  }
  Magnum::Vector3 getVec3(const std::string& key) const {
    return get<Magnum::Vector3>(key);
  }
  Magnum::Quaternion getQuat(const std::string& key) const {
    return get<Magnum::Quaternion>(key);
  }
  Magnum::Rad getRad(const std::string& key) const {
    return get<Magnum::Rad>(key);
  }

  std::shared_ptr<Configuration> getConfigSubgroupAsPtr(
      const std::string& name) const {
    std::shared_ptr<Configuration> configPtr =
        std::make_shared<Configuration>();
    if (cfg.hasGroup(name)) {
      configPtr->cfg = *cfg.group(name);
    }
    return configPtr;
  }

  int getNumConfigSubgroups(const std::string& name) const {
    if (cfg.hasGroup(name)) {
      return cfg.group(name)->valueCount();
    }
    return 0;
  }

  /**@brief Add a string to a group and return the resulting group size. */
  int addStringToGroup(const std::string& key, const std::string& value) {
    cfg.addValue(key, value);
    return cfg.valueCount(key);
  }

  /**@brief Collect and return strings in a key group. */
  std::vector<std::string> getStringGroup(const std::string& key) const {
    std::vector<std::string> strings;
    for (size_t v = 0; v < cfg.valueCount(key); ++v) {
      strings.push_back(cfg.value<std::string>(key, v));
    }
    return strings;
  }

  bool hasValue(const std::string& key) const { return cfg.hasValue(key); }

  bool removeValue(const std::string& key) { return cfg.removeValue(key); }

 protected:
  /**
   * @brief if no subgroup named this will make one, otherwise does nothing.
   * @return whether a group was made or not
   */
  bool addNewSubgroup(const std::string& name) {
    if (cfg.hasGroup(name)) {
      return false;
    }
    cfg.addGroup(name);
    return true;
  }

  Corrade::Utility::ConfigurationGroup cfg;

  ESP_SMART_POINTERS(Configuration)
};  // namespace core

// Below uses std::variant; not yet available in clang c++17
/*
#include <std/variant>

class Configuration {
 public:
  Configuration() : values_() {}

  template <typename T>
  bool set(const std::string& key, const T& value) {
    bool didOverride = hasValue(key);
    values_[key] = value;
    return didOverride;
  }

  template <typename T>
  T get(const std::string& key) const {
    return boost::get<T>(values_.at(key));
  }

  bool hasValue(const std::string& key) {
    return values_.count(key) > 0;
  }

 protected:
  typedef boost::variant<bool, int, float, double, std::string> ConfigValue;
  std::map<std::string, ConfigValue> values_;

  ESP_SMART_POINTERS(Configuration)
};
*/

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
