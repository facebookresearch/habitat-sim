// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Configuration.h"
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include <Magnum/Math/ConfigurationValue.h>
#include "esp/core/Check.h"

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
                          {ConfigStoredType::MagnumVec3, "Magnum::Vector3"},
                          {ConfigStoredType::MagnumQuat, "Magnum::Quaternion"},
                          {ConfigStoredType::MagnumRad, "Magnum::Rad"},
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
    ConfigStoredType type) {  // eugh, long name
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
  _type = ConfigStoredType::Unknown;
  if (isConfigStoredTypeNonTrivial(_type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).destructor(_data);
  }
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  if (this != &otr) {
    deleteCurrentValue();
    copyValueFrom(otr);
  }
  return *this;
}  // namespace config

ConfigValue& ConfigValue::operator=(ConfigValue&& otr) noexcept {
  deleteCurrentValue();
  moveValueFrom(std::move(otr));
  return *this;
}

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
    case ConfigStoredType::MagnumVec3: {
      auto v = get<Mn::Vector3>();
      return Cr::Utility::formatString("[{} {} {}]", v.x(), v.y(), v.z());
    }
    case ConfigStoredType::MagnumQuat: {
      auto q = get<Mn::Quaternion>();
      auto qv = q.vector();
      return Cr::Utility::formatString("[{} {} {}] {}", qv.x(), qv.y(), qv.z(),
                                       q.scalar());
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
    case ConfigStoredType::MagnumVec3:
      return cfg.setValue(key, get<Mn::Vector3>());
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

}  // namespace config
}  // namespace core
}  // namespace esp
