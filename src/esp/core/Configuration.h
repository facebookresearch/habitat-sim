// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CONFIGURATION_H_
#define ESP_CORE_CONFIGURATION_H_

#include <Corrade/Utility/Configuration.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/ConfigurationValue.h>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "esp/core/esp.h"

namespace esp {
namespace core {

class Configuration {
 public:
  Configuration() = default;

  Configuration(const Configuration& otr)
      : configMap_(),
        intMap_(otr.intMap_),
        boolMap_(otr.boolMap_),
        floatMap_(otr.floatMap_),
        doubleMap_(otr.doubleMap_),
        stringMap_(otr.stringMap_),
        vecMap_(),
        quatMap_(),
        radMap_() {
    configMap_.reserve(otr.configMap_.size());
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
    vecMap_.reserve(otr.vecMap_.size());
    for (const auto& entry : otr.vecMap_) {
      vecMap_[entry.first] = Magnum::Vector3{entry.second};
    }
    quatMap_.reserve(otr.quatMap_.size());
    for (const auto& entry : otr.quatMap_) {
      quatMap_[entry.first] = Magnum::Quaternion{entry.second};
    }
    radMap_.reserve(otr.radMap_.size());
    for (const auto& entry : otr.radMap_) {
      radMap_[entry.first] = Magnum::Rad{entry.second};
    }
    numEntries = calcNumEntries();
  }

  // virtual destructor set to that pybind11 recognizes attributes inheritance
  // from configuration to be polymorphic

  virtual ~Configuration() = default;

  template <typename T>
  void set(CORRADE_UNUSED const std::string& key,
           CORRADE_UNUSED const T& value) {
    ESP_ERROR() << "Unknown/unsupported type :" << typeid(T).name()
                << "for key :" << key;
  }
  void set(const std::string& key, const bool& value) {
    addValToMap(key, value, boolMap_);
  }
  void set(const std::string& key, const int& value) {
    addValToMap(key, value, intMap_);
  }
  void set(const std::string& key, const float& value) {
    addValToMap(key, value, floatMap_);
  }
  void set(const std::string& key, const double& value) {
    addValToMap(key, value, doubleMap_);
  }
  void set(const std::string& key, const char* value) {
    addValToMap(key, std::string(value), stringMap_);
  }

  void set(const std::string& key, const std::string& value) {
    addValToMap(key, value, stringMap_);
  }

  void set(const std::string& key, const Magnum::Vector3& value) {
    addValToMap(key, value, vecMap_);
  }
  void set(const std::string& key, const Magnum::Quaternion& value) {
    addValToMap(key, value, quatMap_);
  }
  void set(const std::string& key, const Magnum::Rad& value) {
    addValToMap(key, value, radMap_);
  }

  bool hasBool(const std::string& key) const {
    return checkMapForKey(key, boolMap_);
  }
  bool hasFloat(const std::string& key) const {
    return checkMapForKey(key, floatMap_);
  }
  bool hasDouble(const std::string& key) const {
    return checkMapForKey(key, doubleMap_);
  }
  bool hasInt(const std::string& key) const {
    return checkMapForKey(key, intMap_);
  }
  bool hasString(const std::string& key) const {
    return checkMapForKey(key, stringMap_);
  }
  bool hasVec3(const std::string& key) const {
    return checkMapForKey(key, vecMap_);
  }
  bool hasQuat(const std::string& key) const {
    return checkMapForKey(key, quatMap_);
  }
  bool hasRad(const std::string& key) const {
    return checkMapForKey(key, radMap_);
  }

  bool getBool(const std::string& key) const {
    return getValFromMap(key, boolMap_);
  }
  float getFloat(const std::string& key) const {
    return getValFromMap(key, floatMap_);
  }
  double getDouble(const std::string& key) const {
    return getValFromMap(key, doubleMap_);
  }
  int getInt(const std::string& key) const {
    return getValFromMap(key, intMap_);
  }
  std::string getString(const std::string& key) const {
    return getValFromMap(key, stringMap_);
  }
  Magnum::Vector3 getVec3(const std::string& key) const {
    return getValFromMap(key, vecMap_);
  }
  Magnum::Quaternion getQuat(const std::string& key) const {
    return getValFromMap(key, quatMap_);
  }
  Magnum::Rad getRad(const std::string& key) const {
    return getValFromMap(key, radMap_);
  }

  std::string getBoolAsString(const std::string& key) const {
    const bool val = getValFromMap(key, boolMap_);
    return (val ? "True" : "False");
  }
  std::string getFloatAsString(const std::string& key) const {
    const float val = getValFromMap(key, floatMap_);
    return std::to_string(val);
  }
  std::string getDoubleAsString(const std::string& key) const {
    const double val = getValFromMap(key, doubleMap_);
    return std::to_string(val);
  }
  std::string getIntAsString(const std::string& key) const {
    const int val = getValFromMap(key, intMap_);
    return std::to_string(val);
  }

  std::string getVec3AsString(const std::string& key) const {
    const Magnum::Vector3 val = getValFromMap(key, vecMap_);
    std::string begin = "[";
    return begin.append(std::to_string(val.x()))
        .append(",")
        .append(std::to_string(val.y()))
        .append(",")
        .append(std::to_string(val.z()))
        .append("]");
  }
  std::string getQuatAsString(const std::string& key) const {
    const Magnum::Quaternion val = getValFromMap(key, quatMap_);
    std::string begin = "[";
    return begin.append(std::to_string(val.vector().x()))
        .append(",")
        .append(std::to_string(val.vector().y()))
        .append(",")
        .append(std::to_string(val.vector().z()))
        .append(",")
        .append(std::to_string(val.scalar()))
        .append("]");
  }
  std::string getRadAsString(const std::string& key) const {
    const Magnum::Rad val = getValFromMap(key, radMap_);
    return std::to_string(val.operator float());
  }

  std::vector<std::string> getBoolKeys() const {
    return getKeysFromMap(boolMap_);
  }
  std::vector<std::string> getFloatKeys() const {
    return getKeysFromMap(floatMap_);
  }
  std::vector<std::string> getDoubleKeys() const {
    return getKeysFromMap(doubleMap_);
  }
  std::vector<std::string> getIntKeys() const {
    return getKeysFromMap(intMap_);
  }
  std::vector<std::string> getStringKeys() const {
    return getKeysFromMap(stringMap_);
  }
  std::vector<std::string> getVec3Keys() const {
    return getKeysFromMap(vecMap_);
  }
  std::vector<std::string> getQuatKeys() const {
    return getKeysFromMap(quatMap_);
  }
  std::vector<std::string> getRadKeys() const {
    return getKeysFromMap(radMap_);
  }

  bool removeBool(const std::string& key) {
    return removeValFromMap(key, boolMap_);
  }
  float removeFloat(const std::string& key) {
    return removeValFromMap(key, floatMap_);
  }
  double removeDouble(const std::string& key) {
    return removeValFromMap(key, doubleMap_);
  }
  int removeInt(const std::string& key) {
    return removeValFromMap(key, intMap_);
  }
  std::string removeString(const std::string& key) {
    return removeValFromMap(key, stringMap_);
  }
  Magnum::Vector3 removeVec3(const std::string& key) {
    return removeValFromMap(key, vecMap_);
  }
  Magnum::Quaternion removeQuat(const std::string& key) {
    return removeValFromMap(key, quatMap_);
  }
  Magnum::Rad removeRad(const std::string& key) {
    return removeValFromMap(key, radMap_);
  }

  std::shared_ptr<Configuration> getConfigSubgroupCopy(
      const std::string& name) const {
    if (configMap_.count(name) > 0) {
      // if exists return copy, so that consumers can modify it freely
      return std::make_shared<Configuration>(*configMap_.at(name));
    }
    return std::make_shared<Configuration>();
  }

  /**
   * @brief Use this function when you wish to modify this configuration's
   * subgroup.
   */
  std::shared_ptr<Configuration> editConfigSubgroup(const std::string& name) {
    makeNewSubgroup(name);
    return configMap_.at(name);
  }

  /**
   * @brief move specified subgroup config into configMap at desired name
   */
  void setConfigSubgroupPtr(const std::string& name,
                            std::shared_ptr<Configuration>& configPtr) {
    // overwrite if exists already, move to minimize copies
    if (configMap_.count(name) == 0) {
      ++numEntries;
    }
    configMap_[name] = std::move(configPtr);
  }  // setConfigSubgroupPtr

  int getNumConfigSubgroups(const std::string& name) const {
    if (checkMapForKey(name, configMap_)) {
      return configMap_.at(name)->getNumEntries();
    }
    ESP_ERROR() << "No subgroup named :" << name;
    return 0;
  }

  /**
   *@brief Add a string to a group and return the resulting group size.
   */
  int addStringToGroup(const std::string& key, const std::string& value) {
    stringMap_[key] = value;
    return stringMap_.size();
  }

  int getNumEntries() const { return numEntries; }

  bool hasValues() { return numEntries > 0; }

  bool hasValue(const std::string& key) const {
    return (configMap_.count(key) > 0) || (intMap_.count(key) > 0) ||
           (boolMap_.count(key) > 0) || (floatMap_.count(key) > 0) ||
           (doubleMap_.count(key) > 0) || (stringMap_.count(key) > 0) ||
           (vecMap_.count(key) > 0) || (quatMap_.count(key) > 0) ||
           (radMap_.count(key) > 0);
  }
  /**
   * @brief Builds and returns @ref Corrade::Utility::ConfigurationGroup
   * holding the values in this esp::core::Configuration.
   *
   * @return a reference to a configuration group for this configuration
   * object.
   */
  Corrade::Utility::ConfigurationGroup getConfigGroup() const {
    Corrade::Utility::ConfigurationGroup cfg{};
    putValsInConfigGroup(cfg, boolMap_);
    putValsInConfigGroup(cfg, intMap_);
    putValsInConfigGroup(cfg, floatMap_);
    putValsInConfigGroup(cfg, doubleMap_);
    putValsInConfigGroup(cfg, stringMap_);
    putValsInConfigGroup(cfg, vecMap_);
    putValsInConfigGroup(cfg, quatMap_);
    putValsInConfigGroup(cfg, radMap_);
    // TODO add subgroup Configuration group for each subgroup Configuration
    return cfg;
  }

 protected:
  /**
   * @brief if no subgroup with given name this will make one, otherwise does
   * nothing.
   * @param name Desired name of new subgroup.
   * @return whether a group was made or not
   */
  bool makeNewSubgroup(const std::string& name) {
    if (configMap_.count(name) > 0) {
      return false;
    }
    ++numEntries;
    configMap_[name] = std::make_shared<Configuration>();
    return true;
  }

  template <typename T>
  bool checkMapForKey(const std::string& key,
                      const std::unordered_map<std::string, T>& map) const {
    return map.count(key) > 0;
  }

  template <typename T>
  void addValToMap(const std::string& key,
                   const T& val,
                   std::unordered_map<std::string, T>& map) {
    if (map.count(key) == 0) {
      ++numEntries;
    }
    map[key] = val;
  }

  template <typename T>
  T getValFromMap(const std::string& key,
                  const std::unordered_map<std::string, T>& map) const {
    if (map.count(key) != 0) {
      // return a copy
      return T{map.at(key)};
    }
    ESP_ERROR() << "Key :" << key << "not present in map of type <std::string,"
                << typeid(T).name() << ">";
    return {};
  }

  template <typename T>
  T removeValFromMap(const std::string& key,
                     std::unordered_map<std::string, T>& map) {
    if (map.count(key) != 0) {
      T val = T{map.at(key)};
      map.erase(key);
      --numEntries;
      if (numEntries < 0) {
        numEntries = calcNumEntries();
      }
      // return a copy
      return val;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in map of type <std::string,"
                  << typeid(T).name() << ">";
    return {};
  }

  template <typename T>
  std::vector<std::string> getKeysFromMap(
      const std::unordered_map<std::string, T>& map) const {
    std::vector<std::string> keys;
    keys.reserve(map.size());
    for (const auto& entry : map) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  template <typename T>
  void putValsInConfigGroup(
      Corrade::Utility::ConfigurationGroup& cfg,
      const std::unordered_map<std::string, T>& map) const {
    for (const auto& entry : map) {
      cfg.setValue(entry.first, entry.second);
    }
  }

  int calcNumEntries() {
    return configMap_.size() + intMap_.size() + boolMap_.size() +
           floatMap_.size() + doubleMap_.size() + stringMap_.size() +
           vecMap_.size() + quatMap_.size() + radMap_.size();
  }

  // number of entries added to this configuration
  int numEntries = 0;

  // Map to hold configurations as subgroups
  std::unordered_map<std::string, std::shared_ptr<Configuration>> configMap_{};

  // Maps to hold various simple types
  std::unordered_map<std::string, int> intMap_{};
  std::unordered_map<std::string, bool> boolMap_{};
  std::unordered_map<std::string, float> floatMap_{};
  std::unordered_map<std::string, double> doubleMap_{};
  std::unordered_map<std::string, std::string> stringMap_{};
  std::unordered_map<std::string, Magnum::Vector3> vecMap_{};
  std::unordered_map<std::string, Magnum::Quaternion> quatMap_{};
  std::unordered_map<std::string, Magnum::Rad> radMap_{};
  // Corrade::Utility::ConfigurationGroup cfg;
  ESP_SMART_POINTERS(Configuration)
};  // namespace core

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
