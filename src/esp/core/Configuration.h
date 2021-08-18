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

/**
 * @brief This enum lists every type of value that can be currently stored
 * directly in an @ref esp::core::Configuration.
 */
enum class ConfigStoredType {
  Unknown = ID_UNDEFINED,
  Boolean,
  Integer,
  Double,
  String,
  MagnumVec3,
  MagnumQuat,
  MagnumRad
};

/**
 * @brief This class uses an anonymous tagged union to store values of different
 * types, as well as providing access to the values in a type safe manner.
 */
class ConfigValue {
 private:
  /**
   * @brief This is the type of the data represented in this ConfigValue.
   */
  ConfigStoredType type{ConfigStoredType::Unknown};

  /**
   * @brief This anonymous union holds the various typed values that may
   * represent this ConfigValue.  Unions are struct-like constructs where the
   * storage for all the member variables starts at the same point in memory. By
   * accessing the appropriate value, the type of the data will be preserved.
   */
  union {
    bool b;
    int i;
    double d;
    std::string s;
    Magnum::Vector3 v;
    Magnum::Quaternion q;
    Magnum::Rad r;
  };

  /**
   * @brief Copy the passed @p val into this ConfigValue.
   * @param val source val to copy into this config
   * @param src string name of calling method, for debugging.
   */
  void copyValueInto(const ConfigValue& val, const std::string& src);

  /**
   * @brief Delete the current value.  Resets type to int, i to 0.
   * @param src string name of calling method, for debugging.
   */
  void deleteCurrentValue(const std::string& src);

 public:
  ConfigValue() : i{0} {}
  ConfigValue(const ConfigValue& otr);
  ~ConfigValue();
  ConfigValue& operator=(const ConfigValue& otr);

  bool isValid() const { return type != ConfigStoredType::Unknown; }

  // setters
  void set(bool _b);
  void set(int _i);
  void set(double _d);
  void set(const std::string& _s);
  void set(const char* _s);
  void set(const Magnum::Vector3& _v);
  void set(const Magnum::Quaternion& _q);
  void set(const Magnum::Rad& _r);

  // getters
  auto getBool() const {
    getterTypeCheck(ConfigStoredType::Boolean);
    return b;
  }
  auto getInt() const {
    getterTypeCheck(ConfigStoredType::Integer);
    return i;
  }
  auto getDouble() const {
    getterTypeCheck(ConfigStoredType::Double);
    return d;
  }
  auto getString() const {
    getterTypeCheck(ConfigStoredType::String);
    return s;
  }
  auto getVec3() const {
    getterTypeCheck(ConfigStoredType::MagnumVec3);
    return v;
  }
  auto getQuat() const {
    getterTypeCheck(ConfigStoredType::MagnumQuat);
    return q;
  }
  auto getRad() const {
    getterTypeCheck(ConfigStoredType::MagnumRad);
    return r;
  }

  /**
   * @brief Returns the current type of this @ref ConfigValue
   */
  ConfigStoredType getType() const { return type; }

  /**
   * @brief Retrieve a string representation of the data held in this @ref
   * ConfigValue
   */
  std::string getAsString() const;

  /**
   * @brief Compare the type of this @ref ConfigValue with the passed type.
   */
  bool compareType(const ConfigStoredType& checkType) const {
    return (type == checkType);
  }

  /**
   * @brief Copy this @ref ConfigValue into the passed @ref
   * Corrade::Utility::ConfigurationGroup
   */
  bool putValueInConfigGroup(const std::string& key,
                             Corrade::Utility::ConfigurationGroup& cfg) const;

 protected:
  /**
   * @brief check current Type against passed (Desired) type.  Return if they
   * are equal; destroy if current type is not trivially destructible.
   */
  bool checkTypeAndDest(const ConfigStoredType& checkType);

  /**
   * @brief verify type being queried is actual type of object
   */
  void getterTypeCheck(const ConfigStoredType& checkType) const;

 public:
  ESP_SMART_POINTERS(ConfigValue)
};  // ConfigValue

/**
 * @brief This class holds configuration data in a map of ConfigValues, and also
 * supports nested configurations via a map of smart pointers to this type.
 */
class Configuration {
 public:
  Configuration() = default;

  Configuration(const Configuration& otr)
      : configMap_(), valueMap_(otr.valueMap_) {
    configMap_.reserve(otr.configMap_.size());
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
  }

  // virtual destructor set to that pybind11 recognizes attributes inheritance
  // from configuration to be polymorphic

  virtual ~Configuration() = default;

  bool hasBool(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::Boolean);
  }
  bool hasDouble(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::Double);
  }
  bool hasInt(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::Integer);
  }
  bool hasString(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::String);
  }
  bool hasVec3(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::MagnumVec3);
  }
  bool hasQuat(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::MagnumQuat);
  }
  bool hasRad(const std::string& key) const {
    return checkMapForKeyAndType(key, ConfigStoredType::MagnumRad);
  }

  // ****************** Getters ******************
  bool getBool(const std::string& key) const {
    if (hasBool(key)) {
      return valueMap_.at(key).getBool();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as boolean";
    return {};
  }

  double getDouble(const std::string& key) const {
    if (hasDouble(key)) {
      return valueMap_.at(key).getDouble();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as double";
    return {};
  }
  int getInt(const std::string& key) const {
    if (hasInt(key)) {
      return valueMap_.at(key).getInt();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as integer";
    return {};
  }
  std::string getString(const std::string& key) const {
    if (hasString(key)) {
      return valueMap_.at(key).getString();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as std::string";
    return {};
  }

  Magnum::Vector3 getVec3(const std::string& key) const {
    if (hasVec3(key)) {
      return valueMap_.at(key).getVec3();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as Magnum::Vector3";
    return {};
  }
  Magnum::Quaternion getQuat(const std::string& key) const {
    if (hasQuat(key)) {
      return valueMap_.at(key).getQuat();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as Magnum::Quaternion";
    return {};
  }
  Magnum::Rad getRad(const std::string& key) const {
    if (hasRad(key)) {
      return valueMap_.at(key).getRad();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as Magnum::Rad";
    return {};
  }

  // ****************** String Conversion ******************

  /**
   * @brief This method will look for the provided key, and return a string
   * holding the object, if it is found in one of this configuration's maps
   */
  std::string getAsString(const std::string& key) const {
    if ((valueMap_.count(key) > 0) && (valueMap_.at(key).isValid())) {
      return valueMap_.at(key).getAsString();
    }
    std::string retVal = "Key ";
    retVal.append(key).append(
        " does not represent a valid value in this configuration.");
    ESP_WARNING() << retVal;
    return retVal;
  }

  // ****************** Key List Retrieval ******************
  std::vector<std::string> getKeys() const {
    std::vector<std::string> keys;
    keys.reserve(valueMap_.size());
    for (const auto& entry : valueMap_) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  /**
   * @brief This function returns this configuration's subconfig keys.
   */
  std::vector<std::string> getSubConfigKeys() const {
    std::vector<std::string> keys;
    keys.reserve(configMap_.size());
    for (const auto& entry : configMap_) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  std::vector<std::string> getBoolKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::Boolean);
  }
  std::vector<std::string> getDoubleKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::Double);
  }
  std::vector<std::string> getIntKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::Integer);
  }
  std::vector<std::string> getStringKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::String);
  }
  std::vector<std::string> getVec3Keys() const {
    return getKeysFromMapOfType(ConfigStoredType::MagnumVec3);
  }
  std::vector<std::string> getQuatKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::MagnumQuat);
  }
  std::vector<std::string> getRadKeys() const {
    return getKeysFromMapOfType(ConfigStoredType::MagnumRad);
  }
  // ****************** Setters ******************
  template <typename T>
  void set(CORRADE_UNUSED const std::string& key,
           CORRADE_UNUSED const T& value) {
    ESP_ERROR() << "Unknown/unsupported type :" << typeid(T).name()
                << "for key :" << key;
  }
  void set(const std::string& key, const bool& value) {
    valueMap_[key].set(value);
  }
  void set(const std::string& key, const int& value) {
    valueMap_[key].set(value);
  }

  void set(const std::string& key, const double& value) {
    valueMap_[key].set(value);
  }
  void set(const std::string& key, const char* value) {
    valueMap_[key].set(value);
  }

  void set(const std::string& key, const std::string& value) {
    valueMap_[key].set(value);
  }

  void set(const std::string& key, const Magnum::Vector3& value) {
    valueMap_[key].set(value);
  }
  void set(const std::string& key, const Magnum::Quaternion& value) {
    valueMap_[key].set(value);
  }
  void set(const std::string& key, const Magnum::Rad& value) {
    valueMap_[key].set(value);
  }
  // ****************** Value removal ******************
  void removeValueFromMap(const std::string& key) {
    if (valueMap_.count(key) > 0) {
      valueMap_.erase(key);
    }
  }

  bool removeBool(const std::string& key) {
    if (hasBool(key)) {
      bool v = valueMap_.at(key).getBool();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as boolean";
    return {};
  }

  double removeDouble(const std::string& key) {
    if (hasDouble(key)) {
      double v = valueMap_.at(key).getDouble();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key << "not present in configuration as double";
    return {};
  }
  int removeInt(const std::string& key) {
    if (hasInt(key)) {
      int v = valueMap_.at(key).getInt();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as integer";
    return {};
  }
  std::string removeString(const std::string& key) {
    if (hasString(key)) {
      std::string v = valueMap_.at(key).getString();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as std::string";
    return {};
  }

  Magnum::Vector3 removeVec3(const std::string& key) {
    if (hasVec3(key)) {
      Magnum::Vector3 v = valueMap_.at(key).getVec3();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Magnum::Vector3";
    return {};
  }
  Magnum::Quaternion removeQuat(const std::string& key) {
    if (hasQuat(key)) {
      Magnum::Quaternion v = valueMap_.at(key).getQuat();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Magnum::Quaternion";
    return {};
  }
  Magnum::Rad removeRad(const std::string& key) {
    if (hasRad(key)) {
      return valueMap_.at(key).getRad();
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Magnum::Rad";
    return {};
  }

  std::shared_ptr<Configuration> removeSubconfig(const std::string& key) {
    if (configMap_.count(key) > 0) {
      auto val = configMap_.at(key);
      configMap_.erase(key);
      return val;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in map of subconfigurations.";
    return {};
  }

  // ****************** Subconfiguration accessors ******************

  /**
   * @brief Retrieves a shared pointer to a copy of the subConfig @ref
   * esp::core::Configuration that has the passed @p name . This will create a
   * pointer to a new sub-configuration if none exists already with that name,
   * but will not add this configuration to this Configuration's internal;
   * storage.
   *
   * @param name The name of the configuration to edit.
   * @return A pointer to a copy of the configuration having the requested
   * name, or a pointer to an empty configuration.
   */
  std::shared_ptr<Configuration> getSubConfigCopy(
      const std::string& name) const {
    if (configMap_.count(name) > 0) {
      // if exists return copy, so that consumers can modify it freely
      return std::make_shared<Configuration>(*configMap_.at(name));
    }
    return std::make_shared<Configuration>();
  }

  /**
   * @brief Retrieves the stored shared pointer to the subConfig @ref
   * esp::core::Configuration that has the passed @p name . This will create a
   * new sub-configuration if none exists.
   *
   * Use this function when you wish to modify this configuration's
   * subgroup.
   * @param name The name of the configuration to edit.
   * @return The actual pointer to the configuration having the requested
   * name.
   */
  std::shared_ptr<Configuration>& editSubConfig(const std::string& name) {
    makeNewSubgroup(name);
    return configMap_.at(name);
  }

  /**
   * @brief move specified subgroup config into configMap at desired name
   */
  void setSubConfigPtr(const std::string& name,
                       std::shared_ptr<Configuration>& configPtr) {
    configMap_[name] = std::move(configPtr);
  }  // setSubConfigPtr

  int getNumSubConfigs(const std::string& name) const {
    if (configMap_.count(name) > 0) {
      return configMap_.at(name)->getNumEntries();
    }
    ESP_WARNING() << "No SubConfig found named :" << name;
    return 0;
  }

  /**
   * @brief Return number of entries in this configuration. This only counts
   * subconfiguration entries as a single entry.
   */
  int getNumEntries() const { return configMap_.size() + valueMap_.size(); }

  bool hasValues() const { return getNumEntries() > 0; }

  /**
   * @brief Checks if passed @p key is contained in this configuration.
   * Returns the highest level where @p key was found, or 0 if not found
   * @param key The key to look for
   * @return The level @p key was found. 0 if not found (so can be treated as
   * bool)
   */
  int hasValue(const std::string& key) const {
    return hasValueInternal(key, 0);
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
    putAllValuesInConfigGroup(cfg);
    return cfg;
  }

  /**
   * @brief This method will build a vector of all the config values this
   * configuration holds and the types of these values.
   */
  std::unordered_map<std::string, ConfigStoredType> getValueTypes() const {
    std::unordered_map<std::string, ConfigStoredType> res{};
    res.reserve(valueMap_.size());
    for (const auto& elem : valueMap_) {
      res[elem.first] = elem.second.getType();
    }
    return res;
  }

  /**
   * @brief Merges configuration pointed to by @p config into this
   * configuration, including all subconfigs.  Passed config overwrites
   * existing data in this config.
   * @param config The source of configuration data we wish to merge into this
   * configuration.
   */
  void overwriteWithConfig(const std::shared_ptr<Configuration>& src) {
    if (!src->hasValues()) {
      return;
    }
    // copy every element over from src
    for (const auto& elem : src->valueMap_) {
      valueMap_[elem.first] = elem.second;
    }
    // merge subconfigs
    for (const auto& subConfig : configMap_) {
      const auto name = subConfig.first;
      // make if DNE
      makeNewSubgroup(name);
      // merge src subconfig
      configMap_[name]->overwriteWithConfig(subConfig.second);
    }
  }

 protected:
  /**
   * @brief Checks if passed @p key is contained in this configuration.
   * Returns the highest level where @p key was found
   * @param key The key to look for
   * @param parentLevel The parent level to the current iteration.  If
   * iteration finds
   * @p key, it will return parentLevel+1
   * @return The level @p key was found. 0 if not found (so can be treated
   * as bool)
   */
  int hasValueInternal(const std::string& key, int parentLevel = 0) const {
    int curLevel = parentLevel + 1;
    if (valueMap_.count(key) > 0) {
      return curLevel;
    }
    // not found by here in data maps, check subconfigs, to see what level
    for (const auto& subConfig : configMap_) {
      if (subConfig.first == key) {
        // name of subconfiguration
        return curLevel;
      }
      // find level of
      int resLevel = subConfig.second->hasValueInternal(key, curLevel);
      // if found, will be greater than curLevel
      if (resLevel > curLevel) {
        return resLevel;
      }
    }
    // if not found, return lowest level having been checked
    return parentLevel;
  }

  /**
   * @brief Populate the passed cfg with all the values this map holds, along
   * with the values any subgroups/sub-Configs it may hold
   */
  void putAllValuesInConfigGroup(
      Corrade::Utility::ConfigurationGroup& cfg) const {
    // put tagged union values in map
    for (const auto& entry : valueMap_) {
      entry.second.putValueInConfigGroup(entry.first, cfg);
    }

    for (const auto& subConfig : configMap_) {
      const auto name = subConfig.first;
      auto* cfgSubGroup = cfg.addGroup(name);
      subConfig.second->putAllValuesInConfigGroup(*cfgSubGroup);
    }
  }

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
    configMap_[name] = std::make_shared<Configuration>();
    return true;
  }

  /**
   * @brief check if storage map of this configuration has the passed key and if
   * it is the requested type.
   * @param key the key to check
   * @param type the @ref ConfigStoredType type to check for
   */
  bool checkMapForKeyAndType(const std::string& key,
                             const ConfigStoredType& type) const {
    return (valueMap_.count(key) > 0 && valueMap_.at(key).compareType(type));
  }

  /**
   * @brief Retrieve a vector of the keys of the map for the values of the
   * specified type.
   */
  std::vector<std::string> getKeysFromMapOfType(
      const ConfigStoredType& type) const {
    std::vector<std::string> keys;
    keys.reserve(valueMap_.size());
    int count = 0;
    for (const auto& entry : valueMap_) {
      if (entry.second.compareType(type)) {
        ++count;
        keys.push_back(entry.first);
      }
    }
    keys.resize(count);
    return keys;
  }

  // Map to hold configurations as subgroups
  std::unordered_map<std::string, std::shared_ptr<Configuration>> configMap_{};

  std::unordered_map<std::string, ConfigValue> valueMap_{};

  ESP_SMART_POINTERS(Configuration)
};  // namespace core

}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
