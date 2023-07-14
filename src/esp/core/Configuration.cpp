// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Configuration.h"
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Math/ConfigurationValue.h>
#include "esp/core/Check.h"
#include "esp/io/Json.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace core {
namespace config {

namespace {

// enum class hash function - uses enum value as hash
struct ConfigStoredTypeHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

/**
 * @brief Constant map to provide mappings from @ref ConfigStoredType enum tags
 * to string.  All supported types should have mappings in this map
 */
const std::unordered_map<ConfigStoredType, std::string, ConfigStoredTypeHash>
    ConfigTypeNamesMap = {{ConfigStoredType::Unknown, "Unknown"},
                          {ConfigStoredType::Boolean, "bool"},
                          {ConfigStoredType::Integer, "int"},
                          {ConfigStoredType::Double, "double"},
                          {ConfigStoredType::MagnumVec2, "Mn::Vector2"},
                          {ConfigStoredType::MagnumVec3, "Mn::Vector3"},
                          {ConfigStoredType::MagnumVec4, "Mn::Vector4"},
                          {ConfigStoredType::MagnumMat3, "Mn::Matrix3"},
                          {ConfigStoredType::MagnumQuat, "Mn::Quaternion"},
                          {ConfigStoredType::MagnumRad, "Mn::Rad"},
                          {ConfigStoredType::String, "std::string"}};

// force this functionality to remain local to this file.

// free functions for non-trivial types control.
template <class T>
void copyConstructorFunc(
    const char* const src,
    char* const dst) {  // NOLINT(readability-non-const-parameter)
  new (dst) T{*reinterpret_cast<const T*>(src)};
}
template <class T>
void moveConstructorFunc(
    char* const src,    // NOLINT(readability-non-const-parameter)
    char* const dst) {  // NOLINT(readability-non-const-parameter)
  new (dst) T{std::move(*reinterpret_cast<T*>(src))};
}
template <class T>
void destructorFunc(
    char* const src) {  // NOLINT(readability-non-const-parameter)
  reinterpret_cast<T*>(src)->~T();
}

/**
 * @brief This struct handles non-trivial types by providing a copy constructor,
 * a move constructor and a destructor as function pointers.
 */
struct NonTrivialTypeHandler {
  void (*copier)(const char* const, char* const);
  void (*mover)(char* const, char* const);
  void (*destructor)(char* const);

  template <class T>
  static constexpr NonTrivialTypeHandler make() {
    return {copyConstructorFunc<T>, moveConstructorFunc<T>, destructorFunc<T>};
  }
};

/**
 * @brief Table of @ref NonTrivialTypeHandler s for the types this @ref
 * ConfigValue support. There needs to be an entry in this table for each
 * non-trivial type specified in @ref ConfigStoredType enum, following the
 * pattern for strings.
 */
constexpr NonTrivialTypeHandler nonTrivialTypeHandlers[]{
    NonTrivialTypeHandler::make<std::string>()};

NonTrivialTypeHandler nonTrivialConfigStoredTypeHandlerFor(
    ConfigStoredType type) {
  const std::size_t i = int(type) - int(ConfigStoredType::_nonTrivialTypes);
  CORRADE_INTERNAL_ASSERT(i <
                          Cr::Containers::arraySize(nonTrivialTypeHandlers));
  return nonTrivialTypeHandlers[i];
}

}  // namespace

std::string getNameForStoredType(const ConfigStoredType& value) {
  auto valName = ConfigTypeNamesMap.find(value);
  if (valName != ConfigTypeNamesMap.end()) {
    return valName->second;
  }
  // this should happen only if newly supported type has not been added to
  // ConfigTypeNamesMap
  return Cr::Utility::formatString(
      "ConfigStoredType with value {} not currently supported fully as a type "
      "for a ConfigValue",
      static_cast<int>(value));
}

ConfigValue::ConfigValue(const ConfigValue& otr) {
  copyValueFrom(otr);
}  // copy ctor

ConfigValue::ConfigValue(ConfigValue&& otr) noexcept {
  moveValueFrom(std::move(otr));
}  // move ctor

ConfigValue::~ConfigValue() {
  deleteCurrentValue();
}  // destructor

void ConfigValue::copyValueFrom(const ConfigValue& otr) {
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).copier(otr._data, _data);
  } else {
    std::memcpy(_data, otr._data, sizeof(_data));
  }
}

void ConfigValue::moveValueFrom(ConfigValue&& otr) {
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).mover(otr._data, _data);
  } else {
    // moving character buffer ends up not being much better than copy
    std::memcpy(_data, otr._data, sizeof(_data));
  }
}

void ConfigValue::deleteCurrentValue() {
  if (isConfigStoredTypeNonTrivial(_type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).destructor(_data);
  }
  _type = ConfigStoredType::Unknown;
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  if (this != &otr) {
    deleteCurrentValue();
    copyValueFrom(otr);
  }
  return *this;
}  // ConfigValue::operator=

ConfigValue& ConfigValue::operator=(ConfigValue&& otr) noexcept {
  deleteCurrentValue();
  moveValueFrom(std::move(otr));
  return *this;
}  // ConfigValue::operator=

std::string ConfigValue::getAsString() const {
  switch (_type) {
    case ConfigStoredType::Unknown: {
      return "Undefined value/Unknown type";
    }
    case ConfigStoredType::Boolean: {
      return (get<bool>() ? "True" : "False");
    }
    case ConfigStoredType::Integer: {
      return std::to_string(get<int>());
    }
    case ConfigStoredType::Double: {
      return std::to_string(get<double>());
    }
    case ConfigStoredType::String: {
      return get<std::string>();
    }

    case ConfigStoredType::MagnumVec2: {
      auto v = get<Mn::Vector2>();
      return Cr::Utility::formatString("[{} {}]", v.x(), v.y());
    }
    case ConfigStoredType::MagnumVec3: {
      auto v = get<Mn::Vector3>();
      return Cr::Utility::formatString("[{} {} {}]", v.x(), v.y(), v.z());
    }
    case ConfigStoredType::MagnumVec4: {
      auto v = get<Mn::Vector4>();
      return Cr::Utility::formatString("[{} {} {} {}]", v.x(), v.y(), v.z(),
                                       v.w());
    }
    case ConfigStoredType::MagnumMat3: {
      auto m = get<Mn::Matrix3>();
      std::string res = "[";
      for (int i = 0; i < m.Size; ++i) {
        auto v = m.row(i);
        Cr::Utility::formatInto(res, res.length(), "[{} {} {}]", v.x(), v.y(),
                                v.z());
      }
      Cr::Utility::formatInto(res, res.length(), "]");
      return res;
    }
    case ConfigStoredType::MagnumQuat: {
      auto q = get<Mn::Quaternion>();
      auto qv = q.vector();
      return Cr::Utility::formatString("{} [{} {} {}]", q.scalar(), qv.x(),
                                       qv.y(), qv.z());
    }
    case ConfigStoredType::MagnumRad: {
      auto r = get<Mn::Rad>();
      return std::to_string(r.operator float());
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in ConfigValue::getAsString", "");
  }  // switch
}  // ConfigValue::getAsString

io::JsonGenericValue ConfigValue::writeToJsonObject(
    io::JsonAllocator& allocator) const {
  // unknown is checked before this function is called, so does not need support
  switch (getType()) {
    case ConfigStoredType::Boolean: {
      return io::toJsonValue(get<bool>(), allocator);
    }
    case ConfigStoredType::Integer: {
      return io::toJsonValue(get<int>(), allocator);
    }
    case ConfigStoredType::Double: {
      return io::toJsonValue(get<double>(), allocator);
    }
    case ConfigStoredType::MagnumVec2: {
      return io::toJsonValue(get<Mn::Vector2>(), allocator);
    }
    case ConfigStoredType::MagnumVec3: {
      return io::toJsonValue(get<Mn::Vector3>(), allocator);
    }
    case ConfigStoredType::MagnumVec4: {
      return io::toJsonValue(get<Mn::Vector4>(), allocator);
    }
    case ConfigStoredType::MagnumMat3: {
      return io::toJsonValue(get<Mn::Matrix3>(), allocator);
    }
    case ConfigStoredType::MagnumQuat: {
      return io::toJsonValue(get<Mn::Quaternion>(), allocator);
    }
    case ConfigStoredType::MagnumRad: {
      auto r = get<Mn::Rad>();
      return io::toJsonValue((r.operator float()), allocator);
    }
    case ConfigStoredType::String: {
      return io::toJsonValue(get<std::string>(), allocator);
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in io::toJsonValue<ConfigValue>", {});
  }
}  // ConfigValue::cfgValToJson

bool ConfigValue::putValueInConfigGroup(
    const std::string& key,
    Cr::Utility::ConfigurationGroup& cfg) const {
  switch (_type) {
    case ConfigStoredType::Unknown:
      return false;
    case ConfigStoredType::Boolean:
      return cfg.setValue(key, get<bool>());
    case ConfigStoredType::Integer:
      return cfg.setValue(key, get<int>());
    case ConfigStoredType::Double:
      return cfg.setValue(key, get<double>());
    case ConfigStoredType::String:
      return cfg.setValue(key, get<std::string>());
    case ConfigStoredType::MagnumVec2:
      return cfg.setValue(key, get<Mn::Vector2>());
    case ConfigStoredType::MagnumVec3:
      return cfg.setValue(key, get<Mn::Vector3>());
    case ConfigStoredType::MagnumVec4:
      return cfg.setValue(key, get<Mn::Vector4>());
    case ConfigStoredType::MagnumMat3:
      return cfg.setValue(key, get<Mn::Matrix3>());
    case ConfigStoredType::MagnumQuat:
      return cfg.setValue(key, get<Mn::Quaternion>());
    case ConfigStoredType::MagnumRad:
      return cfg.setValue(key, get<Mn::Rad>());
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in ConfigValue::putValueInConfigGroup",
          false);
  }  // switch
}  // ConfigValue::putValueInConfigGroup

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigStoredType& value) {
  return debug << "Type:" << getNameForStoredType(value);
}

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue& value) {
  return debug << "ConfigValue :(" << Mn::Debug::nospace << value.getAsString()
               << "|" << value.getType() << Mn::Debug::nospace << ")";
}

int Configuration::loadFromJson(const io::JsonGenericValue& jsonObj) {
  // count number of valid user config settings found
  int numConfigSettings = 0;
  for (rapidjson::Value::ConstMemberIterator it = jsonObj.MemberBegin();
       it != jsonObj.MemberEnd(); ++it) {
    // for each key, attempt to parse
    const std::string key{it->name.GetString()};
    const auto& obj = it->value;
    // increment, assuming is valid object
    ++numConfigSettings;

    if (obj.IsDouble()) {
      set(key, obj.GetDouble());
    } else if (obj.IsNumber()) {
      set(key, obj.GetInt());
    } else if (obj.IsString()) {
      set(key, obj.GetString());
    } else if (obj.IsBool()) {
      set(key, obj.GetBool());
    } else if (obj.IsArray() && obj.Size() > 0) {
      // non-empty array of numeric values
      if (obj[0].IsNumber()) {
        // numeric vector, quaternion or matrix
        if (obj.Size() == 2) {
          // All size 2 numeric arrays are mapped to Mn::Vector2
          Mn::Vector2 val{};
          if (io::fromJsonValue(obj, val)) {
            set(key, val);
          }
        } else if (obj.Size() == 3) {
          // All size 3 numeric arrays are mapped to Mn::Vector3
          Mn::Vector3 val{};
          if (io::fromJsonValue(obj, val)) {
            set(key, val);
          }
        } else if (obj.Size() == 4) {
          // JSON numeric array of size 4 can be either vector4, quaternion or
          // color4, so must get type of object that exists with key
          // NOTE : to properly make use of vector4 and color4 configValues
          // loaded from JSON, the owning Configuration must be
          // pre-initialized in its constructor with a default value at the
          // target key. Otherwise for backwards compatibility, we default to
          // reading a quaternion

          // Check if this configuration has pre-defined field with given key
          if (hasValue(key)) {
            ConfigStoredType valType = get(key).getType();
            if (valType == ConfigStoredType::MagnumQuat) {
              // if predefined object is neither
              Mn::Quaternion val{};
              if (io::fromJsonValue(obj, val)) {
                set(key, val);
              }
            } else if (valType == ConfigStoredType::MagnumVec4) {
              // if object exists already @ key and its type is Vector4
              Mn::Vector4 val{};
              if (io::fromJsonValue(obj, val)) {
                set(key, val);
              }
            } else {
              // unknown predefined type of config of size 4
              // this indicates incomplete implementation of size 4
              // configuration type.
              // decrement count for key:obj due to not being handled vector
              --numConfigSettings;

              ESP_WARNING()
                  << "Config cell in JSON document contains key" << key
                  << "referencing an existing configuration element of size "
                     "4 of unknown type:"
                  << getNameForStoredType(valType) << "so skipping.";
            }
          } else {
            // This supports fields that do not yet exist.
            // Check if label contains substrings inferring expected type,
            // otherwise assume this field is a mn::Vector4

            auto lcaseKey = Cr::Utility::String::lowercase(key);
            // labels denoting quaternions
            if ((lcaseKey.find("quat") != std::string::npos) ||
                (lcaseKey.find("rotat") != std::string::npos) ||
                (lcaseKey.find("orient") != std::string::npos)) {
              // object label contains quaternion tag, treat as a
              // Mn::Quaternion
              Mn::Quaternion val{};
              if (io::fromJsonValue(obj, val)) {
                set(key, val);
              }
            } else {
              // if unrecognized label for user-defined field, default the
              // value to be treated as a Mn::Vector4
              Mn::Vector4 val{};
              if (io::fromJsonValue(obj, val)) {
                set(key, val);
              }
            }
          }
        } else if (obj.Size() == 9) {
          // assume is 3x3 matrix
          Mn::Matrix3 mat{};
          if (io::fromJsonValue(obj, mat)) {
            set(key, mat);
          }
        } else {
          // decrement count for key:obj due to not being handled vector
          --numConfigSettings;
          // TODO support numeric array in JSON
          ESP_WARNING()
              << "Config cell in JSON document contains key" << key
              << "referencing an unsupported numeric array of length :"
              << obj.Size() << "so skipping.";
        }
      } else if (obj[0].IsString()) {
        // Array of strings

        // create a new subgroup
        std::shared_ptr<core::config::Configuration> subGroupPtr =
            getSubconfigCopy<core::config::Configuration>(key);

        for (size_t i = 0; i < obj.Size(); ++i) {
          std::string dest;
          if (!io::fromJsonValue(obj[i], dest)) {
            ESP_ERROR() << "Failed to parse array element" << i
                        << "in non-numeric list of values keyed by" << key
                        << "so skipping this value.";
          } else {
            const std::string subKey =
                Cr::Utility::formatString("{}_{:.02d}", key, i);
            subGroupPtr->set(subKey, dest);
            // Increment for each sub-config object
            ++numConfigSettings;
          }
        }

        // Incremented numConfigSettings for this config already.
        // save subgroup's subgroup configuration in original config
        setSubconfigPtr<core::config::Configuration>(key, subGroupPtr);
      }
    } else if (obj.IsObject()) {
      // support nested objects
      // create a new subgroup
      std::shared_ptr<core::config::Configuration> subGroupPtr =
          getSubconfigCopy<core::config::Configuration>(key);
      numConfigSettings += subGroupPtr->loadFromJson(obj);
      // save subgroup's subgroup configuration in original config
      setSubconfigPtr<core::config::Configuration>(key, subGroupPtr);
      //
    } else {
      // TODO support other types?
      // decrement count for key:obj due to not being handled type
      --numConfigSettings;
      ESP_WARNING() << "Config cell in JSON document contains key" << key
                    << "referencing an unknown/unparsable value type, so "
                       "skipping this key.";
    }
  }
  return numConfigSettings;
}  // Configuration::loadFromJson

void Configuration::writeValueToJson(const char* key,
                                     const char* jsonName,
                                     io::JsonGenericValue& jsonObj,
                                     io::JsonAllocator& allocator) const {
  // Create Generic value for key, using allocator, to make sure its a copy
  // and lives long enough
  io::JsonGenericValue name{jsonName, allocator};
  auto jsonVal = get(key).writeToJsonObject(allocator);
  jsonObj.AddMember(name, jsonVal, allocator);
}

void Configuration::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                      io::JsonAllocator& allocator) const {
  // iterate through all values
  // pair of begin/end const iterators to all values
  auto valIterPair = getValuesIterator();
  for (auto& valIter = valIterPair.first; valIter != valIterPair.second;
       ++valIter) {
    if (valIter->second.isValid()) {
      // Create Generic value for key, using allocator, to make sure its a copy
      // and lives long enough
      io::JsonGenericValue name{valIter->first.c_str(), allocator};
      auto jsonVal = valIter->second.writeToJsonObject(allocator);
      jsonObj.AddMember(name, jsonVal, allocator);
    } else {
      ESP_VERY_VERBOSE()
          << "Unitialized ConfigValue in Configuration @ key ["
          << valIter->first
          << "], so nothing will be written to JSON for this key.";
    }
  }  // iterate through all values
}  // Configuration::writeValuesToJson

void Configuration::writeSubconfigsToJson(io::JsonGenericValue& jsonObj,
                                          io::JsonAllocator& allocator) const {
  // iterate through subconfigs
  // pair of begin/end const iterators to all subconfigurations
  auto cfgIterPair = getSubconfigIterator();
  for (auto& cfgIter = cfgIterPair.first; cfgIter != cfgIterPair.second;
       ++cfgIter) {
    // only save if subconfig has entries
    if (cfgIter->second->getNumEntries() > 0) {
      // Create Generic value for key, using allocator, to make sure its a copy
      // and lives long enough
      io::JsonGenericValue name{cfgIter->first.c_str(), allocator};
      io::JsonGenericValue subObj =
          cfgIter->second->writeToJsonObject(allocator);
      jsonObj.AddMember(name, subObj, allocator);
    } else {
      ESP_VERY_VERBOSE()
          << "Unitialized/empty Subconfig in Configuration @ key ["
          << cfgIter->first
          << "], so nothing will be written to JSON for this key.";
    }
  }  // iterate through all configurations

}  // Configuration::writeSubconfigsToJson

io::JsonGenericValue Configuration::writeToJsonObject(
    io::JsonAllocator& allocator) const {
  io::JsonGenericValue jsonObj(rapidjson::kObjectType);
  // iterate through all values - always call base version - this will only
  // ever be called from subconfigs.
  writeValuesToJson(jsonObj, allocator);
  // iterate through subconfigs
  writeSubconfigsToJson(jsonObj, allocator);

  return jsonObj;
}  // writeToJsonObject

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
    const std::string& name) const {
  auto configIter = configMap_.find(name);
  if (configIter != configMap_.end()) {
    // if exists return copy, so that consumers can modify it freely
    return std::make_shared<Configuration>(*configIter->second);
  }
  return std::make_shared<Configuration>();
}

template <>
std::shared_ptr<Configuration> Configuration::editSubconfig<Configuration>(
    const std::string& name) {
  // retrieve existing (or create new) subgroup, with passed name
  return addSubgroup(name);
}

template <>
void Configuration::setSubconfigPtr<Configuration>(
    const std::string& name,
    std::shared_ptr<Configuration>& configPtr) {
  configMap_[name] = std::move(configPtr);
}  // setSubconfigPtr

int Configuration::findValueInternal(const Configuration& config,
                                     const std::string& key,
                                     int parentLevel,
                                     std::vector<std::string>& breadcrumb) {
  int curLevel = parentLevel + 1;
  if (config.valueMap_.count(key) > 0) {
    // Found at this level, access directly via key to get value
    breadcrumb.push_back(key);
    return curLevel;
  }

  // not found by here in data maps, check subconfigs, to see what level
  for (const auto& subConfig : config.configMap_) {
    if (subConfig.first == key) {
      // key matches name of subconfiguration
      breadcrumb.push_back(key);
      return curLevel;
    }
    // add subconfig key to breadcrumb
    breadcrumb.push_back(subConfig.first);
    // search this subconfiguration
    int resLevel =
        findValueInternal(*subConfig.second, key, curLevel, breadcrumb);
    // if found, will be greater than curLevel
    if (resLevel > curLevel) {
      return resLevel;
    }
    // remove subconfig key from breadcrumb
    breadcrumb.pop_back();
  }
  // if not found, return lowest level having been checked
  return parentLevel;
}

std::vector<std::string> Configuration::findValue(
    const std::string& key) const {
  std::vector<std::string> breadcrumbs{};
  // this will make room for some layers without realloc.
  breadcrumbs.reserve(10);
  findValueInternal(*this, key, 0, breadcrumbs);
  return breadcrumbs;
}

Configuration& Configuration::operator=(const Configuration& otr) {
  if (this != &otr) {
    configMap_.clear();
    valueMap_ = otr.valueMap_;
    for (const auto& entry : otr.configMap_) {
      configMap_[entry.first] = std::make_shared<Configuration>(*entry.second);
    }
  }
  return *this;
}

std::string Configuration::getAllValsAsString(
    const std::string& newLineStr) const {
  std::string res{};
  if (getNumEntries() == 0) {
    return newLineStr + "<empty>";
  }

  for (const auto& entry : valueMap_) {
    Cr::Utility::formatInto(res, res.length(), "{}{}:{}", newLineStr,
                            entry.first, entry.second.getAsString());
  }
  const std::string subCfgNewLineStr = newLineStr + "\t";
  for (const auto& subConfig : configMap_) {
    Cr::Utility::formatInto(
        res, res.length(), "{}Subconfig {}:{}", newLineStr, subConfig.first,
        subConfig.second->getAllValsAsString(subCfgNewLineStr));
  }
  return res;
}  // Configuration::getAllValsAsString

Mn::Debug& operator<<(Mn::Debug& debug, const Configuration& cfg) {
  return debug << cfg.getAllValsAsString();
}

}  // namespace config
}  // namespace core
}  // namespace esp
