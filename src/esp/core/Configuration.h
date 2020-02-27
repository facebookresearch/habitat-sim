// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <Corrade/Utility/Configuration.h>
#include <string>

#include "esp/core/esp.h"

namespace esp {
namespace core {

class Configuration {
 public:
  template <typename T>
  bool set(const std::string& key, const T& value) {
    return cfg.setValue(key, value);
  }
  bool setBool(const std::string& key, bool value) { return set(key, value); }
  bool setFloat(const std::string& key, float value) { return set(key, value); }
  bool setInt(const std::string& key, int value) { return set(key, value); }
  bool setString(const std::string& key, std::string value) {
    return set(key, value);
  }

  template <typename T>
  T get(const std::string& key) const {
    return cfg.value<T>(key);
  }
  bool getBool(const std::string& key) const { return get<bool>(key); }
  float getFloat(const std::string& key) const { return get<float>(key); }
  int getInt(const std::string& key) const { return get<int>(key); }
  std::string getString(const std::string& key) const {
    return get<std::string>(key);
  }

  bool hasValue(const std::string& key) const { return cfg.hasValue(key); }

 protected:
  Corrade::Utility::Configuration cfg;

  ESP_SMART_POINTERS(Configuration)
};

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
