// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_CONFIGURATION_H_
#define ESP_CORE_CONFIGURATION_H_

#include <Corrade/Utility/Configuration.h>
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Magnum.h>
#include <string>
#include <unordered_map>

#include "esp/core/Check.h"
#include "esp/core/Esp.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace core {

namespace config {

/**
 * @brief This enum lists every type of value that can be currently stored
 * directly in an @ref esp::core::Configuration.  All supported types should
 * have entries in this enum class.  All non-trivial types should have their
 * enums placed below @p _nonTrivialTypes tag.
 */
enum class ConfigStoredType {
  Unknown = ID_UNDEFINED,
  Boolean,
  Integer,
  Double,
  MagnumVec2,
  MagnumVec3,
  MagnumVec4,
  MagnumMat3,
  MagnumQuat,
  MagnumRad,

  _nonTrivialTypes,
  /**
   * @brief All enum values of nontrivial types must be added after @p String .
   */
  String = _nonTrivialTypes,
};

/**
 * @brief Retrieve a string description of the passed @ref ConfigStoredType enum
 * value.
 */
std::string getNameForStoredType(const ConfigStoredType& value);

/**
 * @brief Quick check to see if type is trivial or not.
 */
constexpr bool isConfigStoredTypeNonTrivial(ConfigStoredType type) {
  return static_cast<int>(type) >=
         static_cast<int>(ConfigStoredType::_nonTrivialTypes);
}

/**
 * @brief Function template to return type enum for specified type. All
 * supported types should have a specialization of this function handling their
 * type to @ref ConfigStoredType enum tags mapping.
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
constexpr ConfigStoredType configStoredTypeFor<Mn::Vector2>() {
  return ConfigStoredType::MagnumVec2;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Vector3>() {
  return ConfigStoredType::MagnumVec3;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Color3>() {
  return ConfigStoredType::MagnumVec3;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Vector4>() {
  return ConfigStoredType::MagnumVec4;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Color4>() {
  return ConfigStoredType::MagnumVec4;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Matrix3>() {
  return ConfigStoredType::MagnumMat3;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Quaternion>() {
  return ConfigStoredType::MagnumQuat;
}
template <>
constexpr ConfigStoredType configStoredTypeFor<Mn::Rad>() {
  return ConfigStoredType::MagnumRad;
}

/**
 * @brief Stream operator to support display of @ref ConfigStoredType enum tags
 */
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
   * @brief The data this ConfigValue holds.
   * Aligns to individual 8-byte bounds. The pair the Configuration map holds
   * consists of a std::string key (sizeof:24 bytes) and a ConfigValue. The
   * _type is 4 bytes, 4 bytes of padding (on 64 bit machines) and 48 bytes for
   * data.
   */
  alignas(8) char _data[6 * 8] = {0};

  /**
   * @brief Copy the passed @p val into this ConfigValue.  If this @ref
   * ConfigValue's type is not trivial, this will call the appropriate copy
   * handler for the type.
   * @param val source val to copy into this config
   */
  void copyValueFrom(const ConfigValue& val);

  /**
   * @brief Move the passed @p val into this ConfigVal. If this @ref
   * ConfigValue's type is not trivial, this will call the appropriate move
   * handler for the type.
   * @param val source val to copy into this config
   */
  void moveValueFrom(ConfigValue&& val);

  /**
   * @brief Delete the current value. If this @ref
   * ConfigValue's type is not trivial, this will call the appropriate
   * destructor handler for the type.
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

  /**
   * @brief Write this ConfigValue to an appropriately configured json object.
   */
  io::JsonGenericValue writeToJsonObject(io::JsonAllocator& allocator) const;

  template <class T>
  void set(const T& value) {
    deleteCurrentValue();
    // This will blow up at compile time if given type is not supported
    _type = configStoredTypeFor<T>();
    // These asserts are checking the integrity of the support for T's type, and
    // will fire if conditions are not met.

    // This fails if we added a new type into @ref ConfigStoredType enum
    // improperly (trivial type added after entry
    // ConfigStoredType::_nonTrivialTypes, or vice-versa)
    static_assert(isConfigStoredTypeNonTrivial(configStoredTypeFor<T>()) !=
                      std::is_trivially_copyable<T>::value,
                  "Something's incorrect about enum placement for added type!");
    // This fails if a new type was added that is too large for internal storage
    static_assert(
        sizeof(T) <= sizeof(_data),
        "ConfigValue's internal storage is too small for added type!");
    // This fails if a new type was added whose alignment does not match
    // internal storage alignment
    static_assert(
        alignof(T) <= alignof(ConfigValue),
        "ConfigValue's internal storage improperly aligned for added type!");

    //_data should be destructed at this point, construct a new value
    new (_data) T{value};
  }

  template <class T>
  const T& get() const {
    ESP_CHECK(_type == configStoredTypeFor<T>(),
              "Attempting to access ConfigValue of"
                  << _type << "with type" << configStoredTypeFor<T>());
    auto val = [&]() { return reinterpret_cast<const T*>(this->_data); };
    return *val();
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
  // convenience typedefs
  typedef std::unordered_map<std::string, ConfigValue> ValueMapType;
  typedef std::map<std::string, std::shared_ptr<Configuration>> ConfigMapType;

  Configuration() = default;

  Configuration(const Configuration& otr)
      : configMap_(), valueMap_(otr.valueMap_) {
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
  }  // copy ctor

  Configuration(Configuration&& otr) noexcept
      : configMap_(std::move(otr.configMap_)),
        valueMap_(std::move(otr.valueMap_)) {}  // move ctor

  // virtual destructor set to that pybind11 recognizes attributes inheritance
  // from configuration to be polymorphic
  virtual ~Configuration() = default;

  /**
   * @brief Copy Assignment.
   */
  Configuration& operator=(const Configuration& otr);

  /**
   * @brief Move Assignment.
   */
  Configuration& operator=(Configuration&& otr) noexcept = default;

  // ****************** Getters ******************
  /**
   * @brief Get @ref ConfigValue specified by key, or empty ConfigValue if DNE.
   *
   * @param key The key of the value desired to be retrieved.
   * @return ConfigValue specified by key. If none exists, will be empty
   * ConfigValue, with type @ref ConfigStoredType::Unknown
   */
  ConfigValue get(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second;
    }
    ESP_WARNING() << "Key :" << key << "not present in configuration";
    return {};
  }

  /**
   * @brief Get value specified by @p key and expected to be type @p T and
   * return it if it exists and is appropriate type.  Otherwise throw a
   * warning and return a default value.
   * @tparam The type of the value desired
   * @param key The key of the value desired to be retrieved.
   * @return The value held at @p key, expected to be type @p T .  If not
   * found, or not of expected type, gives an error message and returns a
   * default value.
   */
  template <class T>
  T get(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    const ConfigStoredType desiredType = configStoredTypeFor<T>();
    if (mapIter != valueMap_.end() &&
        (mapIter->second.getType() == desiredType)) {
      return mapIter->second.get<T>();
    }
    ESP_ERROR() << "Key :" << key << "not present in configuration as"
                << getNameForStoredType(desiredType);
    return {};
  }

  /**
   * @brief Return the @ref ConfigStoredType enum representing the type of the
   * value referenced by the passed @p key or @ref ConfigStoredType::Unknown
   * if unknown/unspecified.
   */
  ConfigStoredType getType(const std::string& key) const {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.getType();
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
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      return mapIter->second.getAsString();
    }
    std::string retVal = Cr::Utility::formatString(
        "Key {} does not represent a valid value in this configuration.", key);
    ESP_WARNING() << retVal;
    return retVal;
  }

  // ****************** Key List Retrieval ******************

  /**
   * @brief Retrieve list of keys present in this @ref Configuration's
   * valueMap_.  Subconfigs are not included.
   */
  std::vector<std::string> getKeys() const {
    std::vector<std::string> keys;
    keys.reserve(valueMap_.size());
    for (const auto& entry : valueMap_) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  /**
   * @brief This function returns this @ref Configuration's subconfig keys.
   */
  std::vector<std::string> getSubconfigKeys() const {
    std::vector<std::string> keys;
    keys.reserve(configMap_.size());
    for (const auto& entry : configMap_) {
      keys.push_back(entry.first);
    }
    return keys;
  }

  /**
   * @brief Retrieve a list of all the keys in this @ref Configuration pointing
  to values of passed type @p storedType.
   * @param storedType The desired type of value whose key should be returned.
   * @return vector of string keys pointing to values of desired @p storedType
   */
  std::vector<std::string> getStoredKeys(ConfigStoredType storedType) const {
    std::vector<std::string> keys;
    // reserve space for all keys
    keys.reserve(valueMap_.size());
    unsigned int count = 0;
    for (const auto& entry : valueMap_) {
      if (entry.second.getType() == storedType) {
        ++count;
        keys.push_back(entry.first);
      }
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

  void set(const std::string& key, float value) {
    valueMap_[key].set<double>(static_cast<double>(value));
  }

  // ****************** Value removal ******************

  /**
   * @brief Remove value specified by @p key and return it if it exists.
   * Otherwise throw a warning and return a default value.
   * @param key The key of the value desired to be retrieved/removed.
   * @return The erased value, held at @p key if found.  If not found, or not of
   * expected type, gives a warning and returns a default value.
   */

  ConfigValue remove(const std::string& key) {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    if (mapIter != valueMap_.end()) {
      valueMap_.erase(mapIter);
      return mapIter->second;
    }
    ESP_WARNING() << "Key :" << key << "not present in configuration";
    return {};
  }

  /**
   * @brief Remove value specified by @p key and expected to be type @p T and
   * return it if it exists and is appropriate type.  Otherwise throw a
   * warning and return a default value.
   * @tparam The type of the value desired
   * @param key The key of the value desired to be retrieved/removed.
   * @return The erased value, held at @p key and expected to be type @p T ,
   * if found.  If not found, or not of expected type, gives a warning and
   * returns a default value.
   */
  template <class T>
  T remove(const std::string& key) {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    const ConfigStoredType desiredType = configStoredTypeFor<T>();
    if (mapIter != valueMap_.end() &&
        (mapIter->second.getType() == desiredType)) {
      valueMap_.erase(mapIter);
      return mapIter->second.get<T>();
    }
    ESP_WARNING() << "Key :" << key << "not present in configuration as"
                  << getNameForStoredType(desiredType);
    return {};
  }

  /**
   * @brief Return number of value and subconfig entries in this configuration.
   * This only counts each subconfiguration entry as a single entry.
   */
  int getNumEntries() const { return configMap_.size() + valueMap_.size(); }

  /**
   * @brief Return number of subconfig entries in this configuration. This only
   * counts each subconfiguration entry as a single entry.
   */
  int getNumSubconfigEntries() const { return configMap_.size(); }

  /**
   * @brief returns number of values in this configuration.
   */
  int getNumValues() const { return valueMap_.size(); }

  /**
   * @brief Returns whether this @ref Configuration has the passed @p key as a
   * non-configuration value. Does not check subconfigurations.
   */
  bool hasValue(const std::string& key) const {
    return valueMap_.count(key) > 0;
  }

  bool hasKeyOfType(const std::string& key, ConfigStoredType desiredType) {
    ValueMapType::const_iterator mapIter = valueMap_.find(key);
    return (mapIter != valueMap_.end() &&
            (mapIter->second.getType() == desiredType));
  }

  /**
   * @brief Checks if passed @p key is contained in this configuration.
   * Returns a list of nested subconfiguration keys, in order, to the
   * configuration where the key was found, ending in the requested @p key.
   * If list is empty, @p key was not found.
   * @param key The key to look for
   * @return A breadcrumb list to where the value referenced by @p key
   * resides. An empty list means the value was not found.
   */
  std::vector<std::string> findValue(const std::string& key) const;

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

  // ****************** Subconfiguration accessors ******************

  /**
   * @brief return if passed key corresponds to a subconfig in this
   * configuration
   */
  bool hasSubconfig(const std::string& key) const {
    ConfigMapType::const_iterator mapIter = configMap_.find(key);
    return (mapIter != configMap_.end());
  }

  /**
   * @brief Templated subconfig copy getter. Retrieves a shared pointer to a
   * copy of the subConfig @ref esp::core::Configuration that has the passed @p
   * name .
   *
   * @tparam Type to return. Must inherit from @ref esp::core::Configuration
   * @param name The name of the configuration to retrieve.
   * @return A pointer to a copy of the configuration having the requested name,
   * cast to the appropriate type, or nullptr if not found.
   */

  template <class T>
  std::shared_ptr<T> getSubconfigCopy(const std::string& name) const {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    auto configIter = configMap_.find(name);
    if (configIter != configMap_.end()) {
      // if exists return copy, so that consumers can modify it freely
      return std::make_shared<T>(
          *std::static_pointer_cast<T>(configIter->second));
    }
    return nullptr;
  }

  /**
   * @brief return pointer to read-only sub-configuration of given @p name. Will
   * fail if configuration with given name dne.
   * @param name The name of the desired configuration.
   */
  std::shared_ptr<const Configuration> getSubconfigView(
      const std::string& name) const {
    auto configIter = configMap_.find(name);
    CORRADE_ASSERT(configIter != configMap_.end(), "", nullptr);
    // if exists return actual object
    return configIter->second;
  }

  /**
   * @brief Templated Version. Retrieves the stored shared pointer to the
   * subConfig @ref esp::core::Configuration that has the passed @p name , cast
   * to the specified type. This will create a shared pointer to a new
   * sub-configuration if none exists and return it, cast to specified type.
   *
   * Use this function when you wish to modify this configuration's
   * subgroup, possibly creating it in the process.
   * @tparam The type to cast the @ref esp::core::Configuration to.  Type is
   * checked to verify that it inherits from Configuration.
   * @param name The name of the configuration to edit.
   * @return The actual pointer to the configuration having the requested
   * name, cast to the specified type.
   */
  template <class T>
  std::shared_ptr<T> editSubconfig(const std::string& name) {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");
    // retrieve existing (or create new) subgroup, with passed name
    return std::static_pointer_cast<T>(addSubgroup(name));
  }

  /**
   * @brief move specified subgroup config into configMap at desired name
   */
  template <class T>
  void setSubconfigPtr(const std::string& name, std::shared_ptr<T>& configPtr) {
    static_assert(std::is_base_of<Configuration, T>::value,
                  "Configuration : Desired subconfig must be derived from "
                  "core::config::Configuration");

    configMap_[name] = std::move(configPtr);
  }  // setSubconfigPtr

  std::shared_ptr<Configuration> removeSubconfig(const std::string& key) {
    ConfigMapType::const_iterator mapIter = configMap_.find(key);
    if (mapIter != configMap_.end()) {
      configMap_.erase(mapIter);
      return mapIter->second;
    }
    ESP_WARNING() << "Key :" << key
                  << "not present in map of subconfigurations.";
    return {};
  }

  int getSubconfigNumEntries(const std::string& name) const {
    auto configIter = configMap_.find(name);
    if (configIter != configMap_.end()) {
      return configIter->second->getNumEntries();
    }
    ESP_WARNING() << "No Subconfig found named :" << name;
    return 0;
  }

  /**
   * @brief Merges configuration pointed to by @p config into this
   * configuration, including all subconfigs.  Passed config overwrites
   * existing data in this config.
   * @param config The source of configuration data we wish to merge into this
   * configuration.
   */
  void overwriteWithConfig(const std::shared_ptr<Configuration>& src) {
    if (src->getNumEntries() == 0) {
      return;
    }
    // copy every element over from src
    for (const auto& elem : src->valueMap_) {
      valueMap_[elem.first] = elem.second;
    }
    // merge subconfigs
    for (const auto& subConfig : configMap_) {
      const auto name = subConfig.first;
      // make if DNE and merge src subconfig
      addSubgroup(name)->overwriteWithConfig(subConfig.second);
    }
  }

  /**
   * @brief Returns a const iterator across the map of values.
   */
  std::pair<ValueMapType::const_iterator, ValueMapType::const_iterator>
  getValuesIterator() const {
    return std::make_pair(valueMap_.cbegin(), valueMap_.cend());
  }

  /**
   * @brief Returns a const iterator across the map of subconfigurations.
   */
  std::pair<ConfigMapType::const_iterator, ConfigMapType::const_iterator>
  getSubconfigIterator() const {
    return std::make_pair(configMap_.cbegin(), configMap_.cend());
  }

  // ==================== load from and save to json =========================

  /**
   * @brief Load values into this Configuration from the passed @p jsonObj. Will
   * recurse for subconfigurations.
   * @param jsonObj The JSON object to read from for the data for this
   * configuration.
   * @return The number of fields successfully read and populated.
   */
  int loadFromJson(const io::JsonGenericValue& jsonObj);

  /**
   * @brief Build and return a json object holding the values and nested objects
   * holding the subconfigs of this Configuration.
   */
  io::JsonGenericValue writeToJsonObject(io::JsonAllocator& allocator) const;

  /**
   * @brief Populate a json object with all the first-level values held in this
   * configuration.  May be overridden to handle special cases for root-level
   * configuration of Attributes classes derived from Configuration.
   */
  virtual void writeValuesToJson(io::JsonGenericValue& jsonObj,
                                 io::JsonAllocator& allocator) const;

  /**
   * @brief Populate a json object with all the data from the subconfigurations,
   * held in json sub-objects, for this Configuration.
   */
  virtual void writeSubconfigsToJson(io::JsonGenericValue& jsonObj,
                                     io::JsonAllocator& allocator) const;

  /**
   * @brief Take the passed @p key and query the config value for that key,
   * writing it to @p jsonName within the passed jsonObj.
   * @param key The key of the data in the configuration
   * @param jsonName The tag to use in the json file
   * @param jsonObj The json object to write to
   * @param allocator The json allocator to use to build the json object
   */
  void writeValueToJson(const char* key,
                        const char* jsonName,
                        io::JsonGenericValue& jsonObj,
                        io::JsonAllocator& allocator) const;

  void writeValueToJson(const char* key,
                        io::JsonGenericValue& jsonObj,
                        io::JsonAllocator& allocator) const {
    writeValueToJson(key, key, jsonObj, allocator);
  }

  /**
   * @brief Return all the values in this cfg in a formatted string. Subconfigs
   * will be displaced by a tab.
   * @param newLineStr The string to put at the end of each newline. As
   * subconfigs are called, add a tab to this.
   */
  std::string getAllValsAsString(const std::string& newLineStr = "\n") const;

 protected:
  /**
   * @brief Friend function.  Checks if passed @p key is contained in @p
   * config. Returns the highest level where @p key was found
   * @param config The configuration to search for passed key
   * @param key The key to look for
   * @param parentLevel The parent level to the current iteration.  If
   * iteration finds @p key, it will return parentLevel+1
   * @param breadcrumb [out] List of keys to subconfigs to get to value.
   * Always should end with @p key.
   * @return The level @p key was found. 0 if not found (so can be treated
   * as bool)
   */
  static int findValueInternal(const Configuration& config,
                               const std::string& key,
                               int parentLevel,
                               std::vector<std::string>& breadcrumb);

  /**
   * @brief Populate the passed cfg with all the values this map holds, along
   * with the values any subgroups/sub-Configs it may hold
   */
  void putAllValuesInConfigGroup(Cr::Utility::ConfigurationGroup& cfg) const {
    // put ConfigVal values in map
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
  std::shared_ptr<Configuration> addSubgroup(const std::string& name) {
    // Attempt to insert an empty pointer
    auto result = configMap_.emplace(name, std::shared_ptr<Configuration>{});
    // If name not already present (insert succeeded) then add new
    // configuration
    if (result.second) {
      result.first->second = std::make_shared<Configuration>();
    }
    return result.first->second;
  }

  // Map to hold configurations as subgroups
  ConfigMapType configMap_{};

  // Map that haolds all config values
  ValueMapType valueMap_{};

  ESP_SMART_POINTERS(Configuration)
};  // class Configuration

MAGNUM_EXPORT Mn::Debug& operator<<(Mn::Debug& debug,
                                    const Configuration& value);

/**
 * @brief Retrieves a shared pointer to a copy of the subConfig @ref
 * esp::core::Configuration that has the passed @p name . This will create a
 * pointer to a new sub-configuration if none exists already with that name,
 * but will not add this configuration to this Configuration's internal
 * storage.
 *
 * @param name The name of the configuration to retrieve.
 * @return A pointer to a copy of the configuration having the requested
 * name, or a pointer to an empty configuration.
 */

template <>
std::shared_ptr<Configuration> Configuration::getSubconfigCopy<Configuration>(
    const std::string& name) const;

template <>
std::shared_ptr<Configuration> Configuration::editSubconfig<Configuration>(
    const std::string& name);

template <>
void Configuration::setSubconfigPtr<Configuration>(
    const std::string& name,
    std::shared_ptr<Configuration>& configPtr);

}  // namespace config
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_CONFIGURATION_H_
