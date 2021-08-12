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
        doubleMap_(otr.doubleMap_),
        stringMap_(otr.stringMap_),
        vecMap_(otr.vecMap_),
        quatMap_(otr.quatMap_),
        radMap_(otr.radMap_) {
    configMap_.reserve(otr.configMap_.size());
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
    // vecMap_.reserve(otr.vecMap_.size());
    // for (const auto& entry : otr.vecMap_) {
    //   vecMap_[entry.first] = Magnum::Vector3{entry.second};
    // }
    // quatMap_.reserve(otr.quatMap_.size());
    // for (const auto& entry : otr.quatMap_) {
    //   quatMap_[entry.first] = Magnum::Quaternion{entry.second};
    // }
    // radMap_.reserve(otr.radMap_.size());
    // for (const auto& entry : otr.radMap_) {
    //   radMap_[entry.first] = Magnum::Rad{entry.second};
    // }
    numEntries = calcNumEntries();
  }

  // virtual destructor set to that pybind11 recognizes attributes inheritance
  // from configuration to be polymorphic

  virtual ~Configuration() = default;

  bool hasBool(const std::string& key) const {
    return checkMapForKey(key, boolMap_);
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

  // ****************** Getters ******************
  bool getBool(const std::string& key) const {
    return getValFromMap(key, boolMap_);
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

  // ****************** Setters ******************
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

  // ****************** String Conversion ******************
  std::string getBoolAsString(const std::string& key) const {
    const bool val = getValFromMap(key, boolMap_);
    return (val ? "True" : "False");
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

  // ****************** Key List Retrieval ******************
  std::vector<std::string> getBoolKeys() const {
    return getKeysFromMap(boolMap_);
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

  // ****************** Value removal ******************
  bool removeBool(const std::string& key) {
    return removeValFromMap(key, boolMap_);
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
  std::shared_ptr<Configuration> removeSubconfig(const std::string& key) {
    return removeValFromMap(key, configMap_);
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
   * @return A pointer to a copy of the configuration having the requested name,
   * or a pointer to an empty configuration.
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
   * @return The actual pointer to the configuration having the requested name.
   */
  std::shared_ptr<Configuration> editSubConfig(const std::string& name) {
    makeNewSubgroup(name);
    return configMap_.at(name);
  }

  /**
   * @brief move specified subgroup config into configMap at desired name
   */
  void setSubConfigPtr(const std::string& name,
                       std::shared_ptr<Configuration>& configPtr) {
    // overwrite if exists already, move to minimize copies
    if (configMap_.count(name) == 0) {
      ++numEntries;
    }
    configMap_[name] = std::move(configPtr);
  }  // setSubConfigPtr

  int getNumSubConfigs(const std::string& name) const {
    if (checkMapForKey(name, configMap_)) {
      return configMap_.at(name)->getNumEntries();
    }
    ESP_WARNING() << "No SubConfig found named :" << name;
    return 0;
  }

  /**
   * @brief Return number of entries in this configuration. This only counts
   * subconfiguration entries as a single entry.
   */
  int getNumEntries() const {
    if (numEntries >= 0) {
      // if numEntries is a legal value, pass it.
      return numEntries;
    }
    // if an impossible value, recalculate
    return calcNumEntries();
  }

  bool hasValues() const { return getNumEntries() > 0; }

  /**
   * @brief Checks if passed @p key is contained in this configuration. Returns
   * the highest level where @p key was found, or 0 if not found
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
   * @brief Merges configuration pointed to by @p config into this
   * configuration, including all subconfigs.  Passed config overwrites existing
   * data in this config.
   * @param config The source of configuration data we wish to merge into this
   * configuration.
   */
  void overwriteWithConfig(const std::shared_ptr<Configuration>& src) {
    if (!src->hasValues()) {
      return;
    }
    mergeMaps(src->boolMap_, boolMap_);
    mergeMaps(src->intMap_, intMap_);
    mergeMaps(src->doubleMap_, doubleMap_);
    mergeMaps(src->stringMap_, stringMap_);
    mergeMaps(src->vecMap_, vecMap_);
    mergeMaps(src->quatMap_, quatMap_);
    mergeMaps(src->radMap_, radMap_);
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
    if ((intMap_.count(key) > 0) || (boolMap_.count(key) > 0) ||
        (doubleMap_.count(key) > 0) || (stringMap_.count(key) > 0) ||
        (vecMap_.count(key) > 0) || (quatMap_.count(key) > 0) ||
        (radMap_.count(key) > 0)) {
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
    putMapValsInConfigGroup(cfg, boolMap_);
    putMapValsInConfigGroup(cfg, intMap_);
    putMapValsInConfigGroup(cfg, doubleMap_);
    putMapValsInConfigGroup(cfg, stringMap_);
    putMapValsInConfigGroup(cfg, vecMap_);
    putMapValsInConfigGroup(cfg, quatMap_);
    putMapValsInConfigGroup(cfg, radMap_);

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
    // adding a subgroup entry, increment numEntries
    ++numEntries;
    configMap_[name] = std::make_shared<Configuration>();
    return true;
  }

  /**
   * @brief Check if passed @p map contains specified key
   */
  template <typename T>
  bool checkMapForKey(const std::string& key,
                      const std::unordered_map<std::string, T>& map) const {
    return map.count(key) > 0;
  }

  /**
   * @brief Add passed value to passed map.  Increment @ref numEntries if @p
   * key not present in map.
   * @tparam The type of the value to add to the map.
   * @param key The key of the value to add
   * @param val The value to add
   * @param map The map to be added to
   */
  template <typename T>
  void addValToMap(const std::string& key,
                   const T& val,
                   std::unordered_map<std::string, T>& map) {
    if (!checkMapForKey(key, map)) {
      ++numEntries;
    } else {
      ESP_DEBUG() << "Key :" << key
                  << "already present in map of type <std::string,"
                  << typeid(T).name() << ">.  Overwriting value.";
    }
    map[key] = val;
  }

  /**
   * @brief Retrieve a copy of the value requested, if it is in the map.  If
   * not, gives an error.
   * @tparam The type of the value to get from the map.
   * @param key The key of the value to get
   * @param map The map to be queried for @p key.
   * @return The value in map at @p key, or an empty value.
   */
  template <typename T>
  T getValFromMap(const std::string& key,
                  const std::unordered_map<std::string, T>& map) const {
    if (checkMapForKey(key, map)) {
      // return a copy
      return T{map.at(key)};
    }
    ESP_ERROR() << "Key :" << key << "not present in map of type <std::string,"
                << typeid(T).name() << ">";
    return {};
  }

  /**
   * @brief Retrieve and remove the value requested, if it is in the map.  If
   * not, gives a warning.
   * @tparam The type of the value to get from the map.
   * @param key The key of the value to get
   * @param map The map to be queried for @p key.
   * @return The value in map at @p key that has been removed, or an empty
   * value.
   */
  template <typename T>
  T removeValFromMap(const std::string& key,
                     std::unordered_map<std::string, T>& map) {
    if (checkMapForKey(key, map)) {
      T val = map.at(key);
      map.erase(key);
      --numEntries;
      if (numEntries < 0) {
        // should never remove more entries than have been entered
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

  /**
   * @brief Retrieve a vector of the keys of @p map.
   */
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

  /**
   * @brief This will merge 2 maps, with the @p srcMap being copied into/over
   * the @p destMap .
   * @tparam the type of object held in both maps
   * @param srcMap The map to be copied from
   * @param destMap The map to be merged into/overwritten
   */
  template <typename T>
  void mergeMaps(const std::unordered_map<std::string, T>& srcMap,
                 std::unordered_map<std::string, T>& destMap) {
    for (const auto& elem : srcMap) {
      destMap[elem.first] = elem.second;
    }
  }

  /**
   * @brief Populate the passed @ref Corrade::Utility::ConfigurationGroup with
   * the values in this Configuration.  Do not use directly for
   * subgroups/subconfig.
   * @tparam The type of stored value in the map being read.
   */

  template <typename T>
  void putMapValsInConfigGroup(
      Corrade::Utility::ConfigurationGroup& cfg,
      const std::unordered_map<std::string, T>& map) const {
    for (const auto& entry : map) {
      cfg.setValue(entry.first, entry.second);
    }
  }

  /**
   * @brief calculate the number of entries in this Configuration at this
   * level.
   */
  int calcNumEntries() const {
    return configMap_.size() + intMap_.size() + boolMap_.size() +
           doubleMap_.size() + stringMap_.size() + vecMap_.size() +
           quatMap_.size() + radMap_.size();
  }

  // number of entries added to this configuration
  int numEntries = 0;

  // Map to hold configurations as subgroups
  std::unordered_map<std::string, std::shared_ptr<Configuration>> configMap_{};

  // Maps to hold various simple types
  std::unordered_map<std::string, int> intMap_{};
  std::unordered_map<std::string, bool> boolMap_{};
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
