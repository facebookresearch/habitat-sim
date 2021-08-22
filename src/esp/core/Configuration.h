// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CONFIGURATION_H_
#define ESP_CORE_CONFIGURATION_H_

#include <Corrade/Utility/Configuration.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Magnum.h>
#include <string>
#include <typeinfo>
#include <unordered_map>

#include "esp/core/Check.h"
#include "esp/core/esp.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace core {

namespace config {

/**
 * @brief This enum lists every type of value that can be currently stored
 * directly in an @ref esp::core::Configuration.
 */
enum class ConfigStoredType {
  Unknown = ID_UNDEFINED,
  Boolean,
  Integer,
  Double,
  MagnumVec3,
  MagnumQuat,
  MagnumRad,

  _nonTrivialTypes,
  /**
   * @brief All enum values of nontrivial types must be added after @p String .
   */
  String = _nonTrivialTypes,
};

/**
 * @brief Quick check to see if type is trivial or not
 */
constexpr bool isConfigStoredTypeNonTrivial(ConfigStoredType type) {
  return static_cast<int>(type) >=
         static_cast<int>(ConfigStoredType::_nonTrivialTypes);
}

/**
 * @brief Function template to return type enum for specified type. Add a
 * specialization for each new type we wish to support.
 */
template <class T>
constexpr ConfigStoredType configStoredTypeFor() {
  static_assert(sizeof(T) == 0, "unsupported type ");
  return {};
}

template <>
constexpr ConfigStoredType configStoredTypeFor<bool>() {
  return ConfigStoredType::Boolean;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<int>() {
  return ConfigStoredType::Integer;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<double>() {
  return ConfigStoredType::Double;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<std::string>() {
  return ConfigStoredType::String;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Vector3>() {
  return ConfigStoredType::MagnumVec3;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Quaternion>() {
  return ConfigStoredType::MagnumQuat;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Rad>() {
  return ConfigStoredType::MagnumRad;
}

MAGNUM_EXPORT Mn::Debug& operator<<(Mn::Debug& debug,
                                    const ConfigStoredType& value);
/**
 * @brief This class uses an anonymous tagged union to store values of different
 * types, as well as providing access to the values in a type safe manner.
 */
class ConfigValue {
 private:
  /**
   * @brief This is the type of the data represented in this ConfigValue.
   */
  ConfigStoredType _type{ConfigStoredType::Unknown};

  /**
   * @brief The data this ConfigValue holds - four floats at most, doubles and
   * 64bit pointers need 8-byte alignment
   */
  alignas(sizeof(void*) * 2) char _data[4 * 8];

  /**
   * @brief Copy the passed @p val into this ConfigValue.
   * @param val source val to copy into this config
   */
  void copyValueInto(const ConfigValue& val);

  /**
   * @brief Move the passed @p val into this ConfigVal.
   * @param val source val to copy into this config
   */
  void moveValueInto(ConfigValue&& val);

  /**
   * @brief Delete the current value.  Resets type to int
   */
  void deleteCurrentValue();

 public:
  ConfigValue() = default;
  ConfigValue(const ConfigValue& otr);
  ConfigValue(ConfigValue&& otr) noexcept;
  ~ConfigValue();
  ConfigValue& operator=(const ConfigValue& otr);

  ConfigValue& operator=(ConfigValue&& otr) noexcept;

  bool isValid() const { return _type != ConfigStoredType::Unknown; }

  template <class T>
  void set(const T& value) {
    // this never fails, not a bool anymore
    deleteCurrentValue();
    // this will blow up at compile time if such type is not supported
    _type = configStoredTypeFor<T>();
    // see later
    static_assert(isConfigStoredTypeNonTrivial(configStoredTypeFor<T>()) !=
                      std::is_trivially_copyable<T>::value,
                  "something's off!");
    // this will blow up if we added new larger types but forgot to update the
    // storage
    static_assert(sizeof(T) <= sizeof(_data), "internal storage too small");
    static_assert(alignof(T) <= alignof(ConfigValue),
                  "internal storage too unaligned");

    //_data should be destructed at this point, construct a new value
    new (_data) T{value};
  }

  template <class T>
  const T& get() const {
    ESP_CHECK(_type == configStoredTypeFor<T>(),
              "Attempting to access ConfigValue of"
                  << _type << "with type" << configStoredTypeFor<T>());
    return *reinterpret_cast<const T*>(_data);
  }

  /**
   * @brief Returns the current type of this @ref ConfigValue
   */
  ConfigStoredType getType() const { return _type; }

  /**
   * @brief Retrieve a string representation of the data held in this @ref
   * ConfigValue
   */
  std::string getAsString() const;

  /**
   * @brief Compare the type of this @ref ConfigValue with the passed type.
   */
  bool compareType(const ConfigStoredType& checkType) const {
    return (_type == checkType);
  }

  /**
   * @brief Copy this @ref ConfigValue into the passed @ref
   * Cr::Utility::ConfigurationGroup
   */
  bool putValueInConfigGroup(const std::string& key,
                             Cr::Utility::ConfigurationGroup& cfg) const;

 public:
  ESP_SMART_POINTERS(ConfigValue)
};  // ConfigValue

MAGNUM_EXPORT Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue& value);

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

  // ****************** Getters ******************
  bool getBool(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::Boolean)) {
      return valueMap_.at(key).get<bool>();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as boolean";
    return {};
  }

  double getDouble(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::Double)) {
      return valueMap_.at(key).get<double>();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as double";
    return {};
  }
  int getInt(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::Integer)) {
      return valueMap_.at(key).get<int>();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as integer";
    return {};
  }
  std::string getString(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::String)) {
      return valueMap_.at(key).get<std::string>();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as std::string";
    return {};
  }

  Mn::Vector3 getVec3(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumVec3)) {
      return valueMap_.at(key).get<Mn::Vector3>();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as Mn::Vector3";
    return {};
  }
  Mn::Quaternion getQuat(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumQuat)) {
      return valueMap_.at(key).get<Mn::Quaternion>();
    }
    ESP_ERROR() << "Key :" << key
                << "not present in configuration as Mn::Quaternion";
    return {};
  }
  Mn::Rad getRad(const std::string& key) const {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumRad)) {
      return valueMap_.at(key).get<Mn::Rad>();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as Mn::Rad";
    return {};
  }

  ConfigStoredType getType(const std::string& key) const {
    if (valueMap_.count(key) > 0) {
      return valueMap_.at(key).getType();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration.";
    return ConfigStoredType::Unknown;
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
  std::vector<std::string> getSubconfigKeys() const {
    std::vector<std::string> keys;
    keys.reserve(configMap_.size());
    for (const auto& entry : configMap_) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  template <ConfigStoredType storedType>
  std::vector<std::string> getStoredKeys() const {
    std::vector<std::string> keys;
    // reserve space for all keys
    keys.reserve(valueMap_.size());
    unsigned int count = 0;
    for (const auto& entry : valueMap_) {
      if (entry.second.compareType(storedType)) {
        ++count;
        keys.push_back(entry.first);
      }
    }
    if (count < keys.size()) {
      keys.resize(count);
    }
    return keys;
  }

  // ****************** Setters ******************
  template <typename T>
  void set(const std::string& key, const T& value) {
    valueMap_[key].set<T>(value);
  }
  void set(const std::string& key, const char* value) {
    valueMap_[key].set<std::string>(std::string(value));
  }

  // ****************** Value removal ******************
  void removeValueFromMap(const std::string& key) {
    if (valueMap_.count(key) > 0) {
      valueMap_.erase(key);
    }
  }

  bool removeBool(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::Boolean)) {
      bool v = valueMap_.at(key).get<bool>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as boolean";
    return {};
  }

  double removeDouble(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::Double)) {
      double v = valueMap_.at(key).get<double>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key << "not present in configuration as double";
    return {};
  }
  int removeInt(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::Integer)) {
      int v = valueMap_.at(key).get<int>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as integer";
    return {};
  }
  std::string removeString(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::String)) {
      std::string v = valueMap_.at(key).get<std::string>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as std::string";
    return {};
  }

  Mn::Vector3 removeVec3(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumVec3)) {
      Mn::Vector3 v = valueMap_.at(key).get<Mn::Vector3>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Mn::Vector3";
    return {};
  }
  Mn::Quaternion removeQuat(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumQuat)) {
      Mn::Quaternion v = valueMap_.at(key).get<Mn::Quaternion>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Mn::Quaternion";
    return {};
  }
  Mn::Rad removeRad(const std::string& key) {
    if (checkMapForKeyAndType(key, ConfigStoredType::MagnumRad)) {
      Mn::Rad v = valueMap_.at(key).get<Mn::Rad>();
      valueMap_.erase(key);
      return v;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in configuration as Mn::Rad";
    return {};
  }

  std::shared_ptr<Configuration> removeSubconfig(const std::string& key) {
    if (hasSubconfig(key)) {
      auto val = configMap_.at(key);
      configMap_.erase(key);
      return val;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in map of subconfigurations.";
    return {};
  }
  // ***************** Check if value is present ******************
  /**
   * @brief check if storage map of this configuration has the passed key and
   * if it is the requested type.
   * @param key the key to check
   * @param type the @ref ConfigStoredType type to check for
   */
  bool checkMapForKeyAndType(const std::string& key,
                             const ConfigStoredType& type) const {
    return (valueMap_.count(key) > 0 && valueMap_.at(key).compareType(type));
  }

  /**
   * @brief return if passed key corresponds to a subconfig in this
   * configuration
   */
  bool hasSubconfig(const std::string& key) const {
    return (configMap_.count(key) > 0);
  }

  // ****************** Subconfiguration accessors ******************

  /**
   * @brief Retrieves a shared pointer to a copy of the subConfig @ref
   * esp::core::Configuration that has the passed @p name . This will create a
   * pointer to a new sub-configuration if none exists already with that name,
   * but will not add this configuration to this Configuration's internal
   * storage.
   *
   * @param name The name of the configuration to edit.
   * @return A pointer to a copy of the configuration having the requested
   * name, or a pointer to an empty configuration.
   */
  std::shared_ptr<Configuration> getSubconfigCopy(
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
   * subgroup, possibly creating it in the process.
   * @param name The name of the configuration to edit.
   * @return The actual pointer to the configuration having the requested
   * name.
   */
  std::shared_ptr<Configuration>& editSubconfig(const std::string& name) {
    makeNewSubgroup(name);
    return configMap_.at(name);
  }

  /**
   * @brief move specified subgroup config into configMap at desired name
   */
  void setSubconfigPtr(const std::string& name,
                       std::shared_ptr<Configuration>& configPtr) {
    configMap_[name] = std::move(configPtr);
  }  // setSubconfigPtr

  int getNumSubconfigs(const std::string& name) const {
    if (configMap_.count(name) > 0) {
      return configMap_.at(name)->getNumEntries();
    }
    ESP_WARNING() << "No Subconfig found named :" << name;
    return 0;
  }

  /**
   * @brief Return number of entries in this configuration. This only counts
   * subconfiguration entries as a single entry.
   */
  int getNumEntries() const { return configMap_.size() + valueMap_.size(); }

  bool hasValues() const { return getNumEntries() > 0; }

  /**
   * @brief Returns whether this @ref Configuration has the passed @p key as a
   * non-configuration value. Does not check subconfigurations.
   */
  bool hasValue(const std::string& key) const {
    return valueMap_.count(key) > 0;
  }

  /**
   * @brief Checks if passed @p key is contained in this configuration.
   * Returns a list of nested subconfiguration keys, in order, to the
   * configuration where the key was found, ending in the requested @p key. If
   * list is empty, @p key was not found.
   * @param key The key to look for
   * @return A breadcrumb list to where the value referenced by @p key
   * resides. An empty list means the value was not found.
   */
  std::vector<std::string> findValue(const std::string& key) const {
    std::vector<std::string> breadcrumbs{};
    findValueInternal(key, 0, breadcrumbs);
    return breadcrumbs;
  }

  /**
   * @brief Builds and returns @ref Cr::Utility::ConfigurationGroup
   * holding the values in this esp::core::Configuration.
   *
   * @return a reference to a configuration group for this configuration
   * object.
   */
  Cr::Utility::ConfigurationGroup getConfigGroup() const {
    Cr::Utility::ConfigurationGroup cfg{};
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
   * iteration finds @p key, it will return parentLevel+1
   * @param breadcrumb [out] List of keys to subconfigs to get to value.
   * Always should end with @p key.
   * @return The level @p key was found. 0 if not found (so can be treated
   * as bool)
   */
  int findValueInternal(const std::string& key,
                        int parentLevel,
                        std::vector<std::string>& breadcrumb) const {
    int curLevel = parentLevel + 1;
    if (valueMap_.count(key) > 0) {
      // Found at this level, access directly via key to get value
      breadcrumb.push_back(key);
      return curLevel;
    }

    // not found by here in data maps, check subconfigs, to see what level
    for (const auto& subConfig : configMap_) {
      if (subConfig.first == key) {
        // key matches name of subconfiguration
        breadcrumb.push_back(key);
        return curLevel;
      }
      // add subconfig key to breadcrumb
      breadcrumb.push_back(subConfig.first);
      // search this subconfiguration
      int resLevel =
          subConfig.second->findValueInternal(key, curLevel, breadcrumb);
      // if found, will be greater than curLevel
      if (resLevel > curLevel) {
        return resLevel;
      } else {
        // remove subconfig key from breadcrumb
        breadcrumb.pop_back();
      }
    }
    // if not found, return lowest level having been checked
    return parentLevel;
  }

  /**
   * @brief Populate the passed cfg with all the values this map holds, along
   * with the values any subgroups/sub-Configs it may hold
   */
  void putAllValuesInConfigGroup(Cr::Utility::ConfigurationGroup& cfg) const {
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
    // Attempt to insert an empty pointer
    auto result = configMap_.insert({name, std::shared_ptr<Configuration>{}});
    // If name already present, nothing inserted
    if (!result.second) {
      return false;
    }
    // Not present yet, fill it with an actual instance
    result.first->second = std::make_shared<Configuration>();
    return true;
  }

  // Map to hold configurations as subgroups
  std::unordered_map<std::string, std::shared_ptr<Configuration>> configMap_{};

  // Map that haolds all config values
  std::unordered_map<std::string, ConfigValue> valueMap_{};

  ESP_SMART_POINTERS(Configuration)
};  // class Configuration

}  // namespace config
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
