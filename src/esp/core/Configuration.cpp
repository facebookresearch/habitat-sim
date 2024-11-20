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
struct ConfigValTypeHash {
  template <typename T>
  std::size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

/**
 * @brief Constant map to provide mappings from @ref ConfigValType enum tags
 * to string.  All supported types should have mappings in this map
 */
const std::unordered_map<ConfigValType, std::string, ConfigValTypeHash>
    ConfigTypeNamesMap = {{ConfigValType::Unknown, "Unknown"},
                          {ConfigValType::Boolean, "bool"},
                          {ConfigValType::Integer, "int"},
                          {ConfigValType::MagnumRad, "Mn::Rad"},
                          {ConfigValType::MagnumDeg, "Mn::Deg"},
                          {ConfigValType::Double, "double"},
                          {ConfigValType::MagnumVec2, "Mn::Vector2"},
                          {ConfigValType::MagnumVec2i, "Mn::Vector2i"},
                          {ConfigValType::MagnumVec3, "Mn::Vector3"},
                          {ConfigValType::MagnumVec4, "Mn::Vector4"},
                          {ConfigValType::MagnumQuat, "Mn::Quaternion"},
                          {ConfigValType::MagnumMat3, "Mn::Matrix3"},
                          {ConfigValType::MagnumMat4, "Mn::Matrix4"},
                          {ConfigValType::String, "std::string"}};

// force this functionality to remain local to this file.

// free functions for non-trivial types control.
// ConfigValue._data is a T* for these types.
template <class T>
void copyConstructorFunc(
    const char* const src,
    char* const dst) {  // NOLINT(readability-non-const-parameter)
  const T* const* tmpSrc = reinterpret_cast<const T* const*>(src);
  T** tmpDst = reinterpret_cast<T**>(dst);
  if (*tmpSrc != nullptr) {
    *tmpDst = new T(**tmpSrc);
  }
}
template <class T>
void moveConstructorFunc(
    char* const src,    // NOLINT(readability-non-const-parameter)
    char* const dst) {  // NOLINT(readability-non-const-parameter)
  T** tmpSrc = reinterpret_cast<T**>(src);
  new (dst) T* {std::move(*tmpSrc)};
}
template <class T>
void destructorFunc(
    char* const src) {  // NOLINT(readability-non-const-parameter)
  T** tmpSrc = reinterpret_cast<T**>(src);
  if (*tmpSrc != nullptr) {
    delete *tmpSrc;
    *tmpSrc = nullptr;
  }
}
template <class T>
bool comparisonFunc(const char* const a, const char* const b) {
  return **reinterpret_cast<const T* const*>(a) ==
         **reinterpret_cast<const T* const*>(b);
}

/**
 * @brief This struct handles pointer-to-data typed (i.e.non-trivial types)
 * ConfigValues by providing a copy constructor, a move constructor and a
 * destructor as function pointers that populate the _data array with the
 * pointer to the instantiated object.
 */
struct PointerBasedTypeHandler {
  void (*copier)(const char* const, char* const);
  void (*mover)(char* const, char* const);
  void (*destructor)(char* const);
  bool (*comparator)(const char* const, const char* const);

  template <class T>
  static constexpr PointerBasedTypeHandler make() {
    return {copyConstructorFunc<T>, moveConstructorFunc<T>, destructorFunc<T>,
            comparisonFunc<T>};
  }
};

/**
 * @brief Table of @ref PointerBasedTypeHandler s for the types the @ref
 * ConfigValue supports.
 *
 * This array will be indexed by consuming ConfigValType -
 * int(ConfigValType::_storedAsAPointer).
 *
 * There needs to be an entry in this table for each pointer-based data
 * type, in sequence as specified in @ref ConfigValType enum following
 * _storedAsAPointer, and each non-trivial type specified in @ref ConfigValType
 * enum, following the pattern for strings (all non-trivial types are by default
 * pointer-based)
 */
constexpr PointerBasedTypeHandler pointerBasedTypeHandlers[]{
    //_storedAsAPointer start
    PointerBasedTypeHandler::make<Mn::Vector3>(),
    PointerBasedTypeHandler::make<Mn::Vector4>(),
    PointerBasedTypeHandler::make<Mn::Quaternion>(),
    PointerBasedTypeHandler::make<Mn::Matrix3>(),
    PointerBasedTypeHandler::make<Mn::Matrix4>(),
    //_nonTrivialTypes start
    PointerBasedTypeHandler::make<std::string>(),
};

PointerBasedTypeHandler pointerBasedConfigTypeHandlerFor(ConfigValType type) {
  const std::size_t i = int(type) - int(ConfigValType::_storedAsAPointer);
  CORRADE_INTERNAL_ASSERT(i <
                          Cr::Containers::arraySize(pointerBasedTypeHandlers));
  return pointerBasedTypeHandlers[i];
}

}  // namespace

std::string getNameForStoredType(const ConfigValType& value) {
  auto valName = ConfigTypeNamesMap.find(value);
  if (valName != ConfigTypeNamesMap.end()) {
    return valName->second;
  }
  // this should happen only if newly supported type has not been added to
  // ConfigTypeNamesMap
  return Cr::Utility::formatString(
      "ConfigValType with value {} not currently supported fully as a type "
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
  _typeAndFlags = otr._typeAndFlags;
  if (isConfigValTypePointerBased(otr.getType())) {
    pointerBasedConfigTypeHandlerFor(getType()).copier(otr._data, _data);
  } else {
    std::memcpy(_data, otr._data, sizeof(_data));
  }
}

void ConfigValue::moveValueFrom(ConfigValue&& otr) {
  // set new type
  _typeAndFlags = otr._typeAndFlags;
  if (isConfigValTypePointerBased(otr.getType())) {
    pointerBasedConfigTypeHandlerFor(getType()).mover(otr._data, _data);
  } else {
    // moving character buffer ends up not being much better than copy
    std::memcpy(_data, otr._data, sizeof(_data));
  }
}

void ConfigValue::deleteCurrentValue() {
  if (isConfigValTypePointerBased(getType())) {
    pointerBasedConfigTypeHandlerFor(getType()).destructor(_data);
  }
  setType(ConfigValType::Unknown);
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  if ((this != &otr) && isSafeToDeconstruct(otr)) {
    deleteCurrentValue();
    copyValueFrom(otr);
  }
  return *this;
}  // ConfigValue::operator=

ConfigValue& ConfigValue::operator=(ConfigValue&& otr) noexcept {
  if (isSafeToDeconstruct(otr)) {
    deleteCurrentValue();
  }
  moveValueFrom(std::move(otr));

  return *this;
}  // ConfigValue::operator=

bool operator==(const ConfigValue& a, const ConfigValue& b) {
  // Verify types are equal
  if (a._typeAndFlags != b._typeAndFlags) {
    return false;
  }
  const auto dataType = a.getType();
  // Pointer-backed data types need to have _data dereffed
  if (isConfigValTypePointerBased(dataType)) {
    return pointerBasedConfigTypeHandlerFor(dataType).comparator(a._data,
                                                                 b._data);
  }

  // By here we know the type is a trivial type and that the types for both
  // values are equal
  if (a.reqsFuzzyCompare()) {
    // Type is specified to require fuzzy comparison
    switch (dataType) {
      case ConfigValType::Double: {
        return Mn::Math::equal(a.get<double>(), b.get<double>());
      }
      default: {
        CORRADE_ASSERT_UNREACHABLE(
            "Unknown/unsupported Type in ConfigValue::operator==()", "");
      }
    }
  }

  // Trivial non-fuzzy-comparison-requiring type : a._data holds the actual
  // value _data array will always hold only legal data, since a ConfigValue
  // should never change type.
  return std::equal(std::begin(a._data), std::end(a._data),
                    std::begin(b._data));

}  // ConfigValue::operator==

bool operator!=(const ConfigValue& a, const ConfigValue& b) {
  return !(a == b);
}

std::string ConfigValue::getAsString() const {
  switch (getType()) {
    case ConfigValType::Unknown: {
      return "Undefined value/Unknown type";
    }
    case ConfigValType::Boolean: {
      return (get<bool>() ? "True" : "False");
    }
    case ConfigValType::Integer: {
      return std::to_string(get<int>());
    }
    case ConfigValType::MagnumRad: {
      auto r = get<Mn::Rad>();
      return std::to_string(r.operator float());
    }
    case ConfigValType::MagnumDeg: {
      auto r = get<Mn::Deg>();
      return std::to_string(r.operator float());
    }
    case ConfigValType::Double: {
      return Cr::Utility::formatString("{}", get<double>());
    }
    case ConfigValType::String: {
      return get<std::string>();
    }
    case ConfigValType::MagnumVec2: {
      auto v = get<Mn::Vector2>();
      return Cr::Utility::formatString("[{} {}]", v.x(), v.y());
    }
    case ConfigValType::MagnumVec2i: {
      auto v = get<Mn::Vector2i>();
      return Cr::Utility::formatString("[{} {}]", v.x(), v.y());
    }
    case ConfigValType::MagnumVec3: {
      auto v = get<Mn::Vector3>();
      return Cr::Utility::formatString("[{} {} {}]", v.x(), v.y(), v.z());
    }
    case ConfigValType::MagnumVec4: {
      auto v = get<Mn::Vector4>();
      return Cr::Utility::formatString("[{} {} {} {}]", v.x(), v.y(), v.z(),
                                       v.w());
    }
    case ConfigValType::MagnumMat3: {
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
    case ConfigValType::MagnumMat4: {
      auto m = get<Mn::Matrix4>();
      std::string res = "[";
      for (int i = 0; i < m.Size; ++i) {
        auto v = m.row(i);
        Cr::Utility::formatInto(res, res.length(), "[{} {} {} {}]", v.x(),
                                v.y(), v.z(), v.w());
      }
      Cr::Utility::formatInto(res, res.length(), "]");
      return res;
    }
    case ConfigValType::MagnumQuat: {
      auto q = get<Mn::Quaternion>();
      auto qv = q.vector();
      return Cr::Utility::formatString("{} [{} {} {}]", q.scalar(), qv.x(),
                                       qv.y(), qv.z());
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in ConfigValue::getAsString", "");
  }  // switch
}  // ConfigValue::getAsString

io::JsonGenericValue ConfigValue::writeToJsonObject(
    io::JsonAllocator& allocator) const {
  // unknown is checked before this function is called, so does not need
  // support
  switch (getType()) {
    case ConfigValType::Boolean: {
      return io::toJsonValue(get<bool>(), allocator);
    }
    case ConfigValType::Integer: {
      return io::toJsonValue(get<int>(), allocator);
    }
    case ConfigValType::MagnumRad: {
      auto r = get<Mn::Rad>();
      return io::toJsonValue((r.operator float()), allocator);
    }
    case ConfigValType::MagnumDeg: {
      auto r = get<Mn::Deg>();
      return io::toJsonValue((r.operator float()), allocator);
    }
    case ConfigValType::Double: {
      return io::toJsonValue(get<double>(), allocator);
    }
    case ConfigValType::MagnumVec2: {
      return io::toJsonValue(get<Mn::Vector2>(), allocator);
    }
    case ConfigValType::MagnumVec2i: {
      return io::toJsonValue(get<Mn::Vector2i>(), allocator);
    }
    case ConfigValType::MagnumVec3: {
      return io::toJsonValue(get<Mn::Vector3>(), allocator);
    }
    case ConfigValType::MagnumVec4: {
      return io::toJsonValue(get<Mn::Vector4>(), allocator);
    }
    case ConfigValType::MagnumMat3: {
      return io::toJsonValue(get<Mn::Matrix3>(), allocator);
    }
    case ConfigValType::MagnumMat4: {
      return io::toJsonValue(get<Mn::Matrix4>(), allocator);
    }
    case ConfigValType::MagnumQuat: {
      return io::toJsonValue(get<Mn::Quaternion>(), allocator);
    }
    case ConfigValType::String: {
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
  switch (getType()) {
    case ConfigValType::Unknown:
      return false;
    case ConfigValType::Boolean:
      return cfg.setValue(key, get<bool>());
    case ConfigValType::Integer:
      return cfg.setValue(key, get<int>());
    case ConfigValType::MagnumRad:
      return cfg.setValue(key, get<Mn::Rad>());
    case ConfigValType::MagnumDeg:
      return cfg.setValue(key, get<Mn::Deg>());
    case ConfigValType::Double:
      return cfg.setValue(key, get<double>());
    case ConfigValType::String:
      return cfg.setValue(key, get<std::string>());
    case ConfigValType::MagnumVec2:
      return cfg.setValue(key, get<Mn::Vector2>());
    case ConfigValType::MagnumVec2i:
      return cfg.setValue(key, get<Mn::Vector2i>());
    case ConfigValType::MagnumVec3:
      return cfg.setValue(key, get<Mn::Vector3>());
    case ConfigValType::MagnumVec4:
      return cfg.setValue(key, get<Mn::Vector4>());
    case ConfigValType::MagnumMat3:
      return cfg.setValue(key, get<Mn::Matrix3>());
    case ConfigValType::MagnumMat4:
      return cfg.setValue(key, get<Mn::Matrix4>());
    case ConfigValType::MagnumQuat:
      return cfg.setValue(key, get<Mn::Quaternion>());
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "Unknown/unsupported Type in ConfigValue::putValueInConfigGroup",
          false);
  }  // switch
}  // ConfigValue::putValueInConfigGroup

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValType& value) {
  return debug << "Type:" << getNameForStoredType(value);
}

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue& value) {
  return debug << "ConfigValue :(" << Mn::Debug::nospace << value.getAsString()
               << "|" << value.getType() << Mn::Debug::nospace << ")";
}

int Configuration::loadOneConfigFromJson(int numConfigSettings,
                                         const std::string& key,
                                         const io::JsonGenericValue& jsonObj) {
  // increment, assuming is valid object
  ++numConfigSettings;
  if (jsonObj.IsDouble()) {
    set(key, jsonObj.GetDouble());
  } else if (jsonObj.IsNumber()) {
    set(key, jsonObj.GetInt());
  } else if (jsonObj.IsString()) {
    set(key, jsonObj.GetString());
  } else if (jsonObj.IsBool()) {
    set(key, jsonObj.GetBool());
  } else if (jsonObj.IsArray() && jsonObj.Size() > 0) {
    // non-empty array of numeric values - first attempt to put them into a
    // magnum vector or matrix
    if (jsonObj[0].IsNumber()) {
      // numeric vector, quaternion or matrix
      if (jsonObj.Size() == 2) {
        // Map numeric/integers to
        if (jsonObj[0].IsDouble()) {
          // All size 2 double-based numeric arrays are mapped to Mn::Vector2
          Mn::Vector2 val{};
          if (io::fromJsonValue(jsonObj, val)) {
            set(key, val);
          }
        } else {
          // All size 2 non-double-based numeric arrays are mapped to
          // Mn::Vector2i
          Mn::Vector2i val{};
          if (io::fromJsonValue(jsonObj, val)) {
            set(key, val);
          }
        }
      } else if (jsonObj.Size() == 3) {
        // All size 3 numeric arrays are mapped to Mn::Vector3
        Mn::Vector3 val{};
        if (io::fromJsonValue(jsonObj, val)) {
          set(key, val);
        }
      } else if (jsonObj.Size() == 4) {
        // JSON numeric array of size 4 can be either vector4, quaternion or
        // color4, so must get type of object that exists with key
        // NOTE : to properly make use of vector4 and color4 configValues
        // loaded from JSON, the owning Configuration must be
        // pre-initialized in its constructor with a default value at the
        // target key.

        // Check if this configuration has pre-defined field with given key
        if (hasValue(key)) {
          ConfigValType valType = get(key).getType();
          if (valType == ConfigValType::MagnumQuat) {
            // if predefined object is neither
            Mn::Quaternion val{};
            if (io::fromJsonValue(jsonObj, val)) {
              set(key, val);
            }
          } else if (valType == ConfigValType::MagnumVec4) {
            // if object exists already @ key and its type is Vector4
            Mn::Vector4 val{};
            if (io::fromJsonValue(jsonObj, val)) {
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
            if (io::fromJsonValue(jsonObj, val)) {
              set(key, val);
            }
          } else {
            // if unrecognized label for user-defined field, default the
            // value to be treated as a Mn::Vector4
            Mn::Vector4 val{};
            if (io::fromJsonValue(jsonObj, val)) {
              set(key, val);
            }
          }
        }
      } else if (jsonObj.Size() == 9) {
        // assume is 3x3 matrix
        Mn::Matrix3 mat{};
        if (io::fromJsonValue(jsonObj, mat)) {
          set(key, mat);
        }
      } else if (jsonObj.Size() == 16) {
        // assume is 4x4 matrix
        Mn::Matrix4 mat{};
        if (io::fromJsonValue(jsonObj, mat)) {
          set(key, mat);
        }
      } else {
        // The array does not match any currently supported magnum
        // objects, so place in indexed subconfig of values.
        // decrement count by 1 - the recursive subgroup load will count all
        // the values.
        --numConfigSettings;
        // create a new subgroup
        std::shared_ptr<core::config::Configuration> subGroupPtr =
            editSubconfig<core::config::Configuration>(key);
        // load array into subconfig
        numConfigSettings += subGroupPtr->loadFromJsonArray(jsonObj);
      }
      // value in array is a number of specified length, else it is a string,
      // an object or a nested array
    } else {
      // decrement count by 1 - the recursive subgroup load will count all the
      // values.
      --numConfigSettings;
      // create a new subgroup
      std::shared_ptr<core::config::Configuration> subGroupPtr =
          editSubconfig<core::config::Configuration>(key);
      // load array into subconfig
      numConfigSettings += subGroupPtr->loadFromJsonArray(jsonObj);
    }
  } else if (jsonObj.IsObject()) {
    // support nested objects
    // create a new subgroup
    // decrement count by 1 - the recursive subgroup load will count all the
    // values.
    --numConfigSettings;
    std::shared_ptr<core::config::Configuration> subGroupPtr =
        editSubconfig<core::config::Configuration>(key);
    numConfigSettings += subGroupPtr->loadFromJson(jsonObj);
    //
  } else {
    // TODO support other types?
    // decrement count for key:obj due to not being handled type
    --numConfigSettings;
    ESP_WARNING() << "Config cell in JSON document contains key" << key
                  << "referencing an unknown/unparsable value type, so "
                     "skipping this key.";
  }
  return numConfigSettings;
}  // Configuration::loadOneConfigFromJson

int Configuration::loadFromJsonArray(const io::JsonGenericValue& jsonObj) {
  // Passed config is found to be a json array, so load every value into this
  // configuration with key as string version of index.
  int numConfigSettings = 0;
  for (size_t i = 0; i < jsonObj.Size(); ++i) {
    const std::string subKey = Cr::Utility::formatString("{:.03d}", i);
    numConfigSettings =
        loadOneConfigFromJson(numConfigSettings, subKey, jsonObj[i]);
  }
  return numConfigSettings;
}  // Configuration::loadFromJsonArray

int Configuration::loadFromJson(const io::JsonGenericValue& jsonObj) {
  // count number of valid user config settings found
  int numConfigSettings = 0;
  if (jsonObj.IsArray()) {
    // load array into this Configuration
    numConfigSettings = loadFromJsonArray(jsonObj);
  } else {
    for (rapidjson::Value::ConstMemberIterator it = jsonObj.MemberBegin();
         it != jsonObj.MemberEnd(); ++it) {
      // for each key, attempt to parse
      const std::string key{it->name.GetString()};
      // load value and attach it to given key
      numConfigSettings =
          loadOneConfigFromJson(numConfigSettings, key, it->value);
    }
  }
  return numConfigSettings;
}  // Configuration::loadFromJson

void Configuration::writeValueToJsonInternal(
    const ConfigValue& configValue,
    const char* jsonName,
    io::JsonGenericValue& jsonObj,
    io::JsonAllocator& allocator) const {
  io::JsonGenericValue name{jsonName, allocator};
  auto jsonVal = configValue.writeToJsonObject(allocator);
  jsonObj.AddMember(name, jsonVal, allocator);
}

void Configuration::writeValuesToJson(io::JsonGenericValue& jsonObj,
                                      io::JsonAllocator& allocator) const {
  // iterate through all values
  // pair of begin/end const iterators to all values
  auto valIterPair = getValuesIterator();
  for (auto& valIter = valIterPair.first; valIter != valIterPair.second;
       ++valIter) {
    if (!valIter->second.isValid()) {
      ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
          << "Unitialized ConfigValue in Configuration @ key `"
          << valIter->first
          << "`, so nothing will be written to JSON for this key.";

    } else if (valIter->second.shouldWriteToFile()) {
      // Create Generic value for key, using allocator, to make sure its a
      // copy and lives long enough
      writeValueToJsonInternal(valIter->second, valIter->first.c_str(), jsonObj,
                               allocator);
    } else {
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
    // only save if subconfig tree has value entries
    if (cfgIter->second->getConfigTreeNumValues() > 0) {
      // Create Generic value for key, using allocator, to make sure its a
      // copy and lives long enough
      io::JsonGenericValue name{cfgIter->first.c_str(), allocator};
      io::JsonGenericValue subObj =
          cfgIter->second->writeToJsonObject(allocator);
      jsonObj.AddMember(name, subObj, allocator);
    } else {
      ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
          << "Unitialized/empty Subconfig in Configuration @ key `"
          << cfgIter->first
          << "`, so nothing will be written to JSON for this key.";
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

template <>

std::vector<float> Configuration::getSubconfigValsOfTypeInVector(
    const std::string& subCfgName) const {
  const ConfigValType desiredType = configValTypeFor<double>();
  const auto subCfg = getSubconfigView(subCfgName);
  const auto& subCfgTags = subCfg->getKeysByType(desiredType, true);
  std::vector<float> res;
  res.reserve(subCfgTags.size());
  for (const auto& tag : subCfgTags) {
    res.emplace_back(static_cast<float>(subCfg->get<double>(tag)));
  }
  return res;
}  // getSubconfigValsOfTypeInVector float specialization

template <>
void Configuration::setSubconfigValsOfTypeInVector(
    const std::string& subCfgName,
    const std::vector<float>& values) {
  auto subCfg = editSubconfig<Configuration>(subCfgName);
  // remove existing values in subconfig of specified type
  subCfg->removeAllOfType<double>();
  // add new values, building string key from index in values array of each
  // value.
  for (std::size_t i = 0; i < values.size(); ++i) {
    const std::string& key = Cr::Utility::formatString("{:.03d}", i);
    subCfg->set(key, values[i]);
  }
}  // setSubconfigValsOfTypeInVector float specialization

/**
 * @brief Retrieves a shared pointer to a copy of the subConfig @ref
 * esp::core::config::Configuration that has the passed @p name . This will
 * create a pointer to a new sub-configuration if none exists already with
 * that name, but will not add this configuration to this Configuration's
 * internal storage.
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
  return addOrEditSubgroup<Configuration>(name).first->second;
}

template <>
void Configuration::setSubconfigPtr<Configuration>(
    const std::string& name,
    std::shared_ptr<Configuration>& configPtr) {
  configMap_[name] = std::move(configPtr);
}

int Configuration::rekeyAllValues() {
  // Get all sorted key-value pairs of values
  std::map<std::string, ConfigValue> sortedValMap(valueMap_.begin(),
                                                  valueMap_.end());
  // clear out existing value map - subconfigs are not touched.
  valueMap_.clear();
  // place sorted values with newly constructed keys
  int keyIter = 0;
  for (auto it = sortedValMap.cbegin(); it != sortedValMap.cend(); ++it) {
    std::string newKey = Cr::Utility::formatString("{:.03d}", keyIter++);
    valueMap_[newKey] = it->second;
  }
  return keyIter;
}  // Configuration::rekeyAllValues()

int Configuration::rekeySubconfigValues(const std::string& subconfigKey) {
  std::pair<ConfigMapType::iterator, bool> rekeySubconfigEntry =
      addOrEditSubgroup<Configuration>(subconfigKey);
  // check if subconfig existed already - result from addOrEditSubgroup would
  // be false if add failed.
  if (rekeySubconfigEntry.second) {
    // Subconfig did not exist, was created by addOrEdit, so delete and return
    // 0.
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "No subconfig found with key `" << subconfigKey
        << "` so rekeying aborted.";
    configMap_.erase(rekeySubconfigEntry.first);
    return 0;
  }
  // retrieve subconfig
  auto rekeySubconfig = rekeySubconfigEntry.first->second;
  if (rekeySubconfig->getNumValues() == 0) {
    // Subconfig exists but has no values defined, so no rekeying possible
    ESP_DEBUG(Mn::Debug::Flag::NoSpace)
        << "Subconfig with key `" << subconfigKey
        << "` has no values, so no rekeying accomplished.";
    return 0;
  }
  // rekey subconfig and return the number of values affected
  return rekeySubconfig->rekeyAllValues();

}  // Configuration::rekeySubconfig

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

void Configuration::overwriteWithConfig(
    const std::shared_ptr<const Configuration>& src) {
  if (src->getNumEntries() == 0) {
    return;
  }
  // copy every element over from src
  for (const auto& elem : src->valueMap_) {
    valueMap_[elem.first] = elem.second;
  }
  // merge subconfigs
  for (const auto& subConfig : src->configMap_) {
    const auto name = subConfig.first;
    // make if DNE and merge src subconfig
    addOrEditSubgroup<Configuration>(name).first->second->overwriteWithConfig(
        subConfig.second);
  }
}  // Configuration::overwriteWithConfig

void Configuration::filterFromConfig(
    const std::shared_ptr<const Configuration>& src) {
  if (src->getNumEntries() == 0) {
    return;
  }
  // filter out every element that is present with the same value in both src
  // and this.
  for (const auto& elem : src->valueMap_) {
    ValueMapType::const_iterator mapIter = valueMap_.find(elem.first);
    // if present and has the same data, erase this configuration's data
    if ((mapIter != valueMap_.end()) && (mapIter->second == elem.second)) {
      valueMap_.erase(mapIter);
    }
  }
  // repeat process on all subconfigs of src that are present in this.
  for (const auto& subConfig : src->configMap_) {
    // find if this has subconfig of same name
    ConfigMapType::iterator mapIter = configMap_.find(subConfig.first);
    if (mapIter != configMap_.end()) {
      mapIter->second->filterFromConfig(subConfig.second);
      // remove the subconfig if it has no entries after filtering
      if (mapIter->second->getNumEntries() == 0) {
        configMap_.erase(mapIter);
      }
    }
  }
}  // Configuration::filterFromConfig

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

bool operator==(const Configuration& a, const Configuration& b) {
  if ((a.getNumSubconfigs() != b.getNumSubconfigs()) ||
      (a.getNumVisibleValues() != b.getNumVisibleValues())) {
    return false;
  }
  for (const auto& entry : a.configMap_) {
    const auto bEntry = b.configMap_.find(entry.first);
    if ((bEntry == b.configMap_.end()) ||
        (*(bEntry->second) != *(entry.second))) {
      // not found or not the same value
      return false;
    }
  }
  for (const auto& entry : a.valueMap_) {
    if (entry.second.isHiddenVal()) {
      // Don't compare hidden values
      continue;
    }
    const auto bEntry = b.valueMap_.find(entry.first);
    if ((bEntry == b.valueMap_.end()) || (bEntry->second != entry.second)) {
      // not found or not the same value
      return false;
    }
  }
  return true;
}

bool operator!=(const Configuration& a, const Configuration& b) {
  return !(a == b);
}

}  // namespace config
}  // namespace core
}  // namespace esp
