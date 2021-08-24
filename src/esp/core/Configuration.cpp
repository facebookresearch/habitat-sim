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

struct NonTrivialTypeHandler {
  void (*copier)(const char* const, char* const);
  void (*mover)(char* const, char* const);
  void (*destructor)(char* const);

  template <class T>
  static constexpr NonTrivialTypeHandler make() {
    return {copyConstructorFunc<T>, moveConstructorFunc<T>, destructorFunc<T>};
  }
};

}  // namespace
constexpr NonTrivialTypeHandler nonTrivialTypeHandlers[]{
    NonTrivialTypeHandler::make<std::string>()};

NonTrivialTypeHandler nonTrivialConfigStoredTypeHandlerFor(
    ConfigStoredType type) {  // eugh, long name
  const std::size_t i = int(type) - int(ConfigStoredType::_nonTrivialTypes);
  CORRADE_INTERNAL_ASSERT(i <
                          Cr::Containers::arraySize(nonTrivialTypeHandlers));
  return nonTrivialTypeHandlers[i];
}

ConfigValue::ConfigValue(const ConfigValue& otr) {
  copyValueInto(otr);
}

ConfigValue::ConfigValue(ConfigValue&& otr) noexcept {
  copyValueInto(otr);
}

ConfigValue::~ConfigValue() {
  deleteCurrentValue();
}

void ConfigValue::copyValueInto(const ConfigValue& otr) {
  std::memcpy(_data, otr._data, sizeof(_data));
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).copier(otr._data, _data);
  }
}

void ConfigValue::moveValueInto(ConfigValue&& otr) {
  // moving character buffer ends up not being much better than copy
  std::memcpy(_data, otr._data, sizeof(_data));
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).mover(otr._data, _data);
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
    copyValueInto(otr);
  }
  return *this;
}  // namespace config

ConfigValue& ConfigValue::operator=(ConfigValue&& otr) noexcept {
  deleteCurrentValue();
  moveValueInto(std::move(otr));
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
      ESP_CHECK(true, "Unknown/unsupported Type in ConfigValue::getAsString.");
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
      ESP_CHECK(
          true,
          "Unknown/unsupported Type in ConfigValue::putValueInConfigGroup.");
  }  // switch
  return false;
}  // ConfigValue::putValueInConfigGroup

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigStoredType& value) {
  debug << "Type";
  switch (value) {
/* LCOV_EXCL_START */
#define _s(value)               \
  case ConfigStoredType::value: \
    return debug << ":" #value;
    _s(Unknown) _s(Boolean) _s(Integer) _s(Double) _s(MagnumVec3) _s(MagnumQuat)
        _s(MagnumRad) _s(String)
#undef _s
    /* LCOV_EXCL_STOP */
  }
  return debug << ":" << static_cast<int>(value)
               << "not supported in debug stream";
}

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue& value) {
  return debug << "ConfigValue :(" << Mn::Debug::nospace << value.getAsString()
               << "|" << value.getType() << Mn::Debug::nospace << ")";
}

}  // namespace config
}  // namespace core
}  // namespace esp
