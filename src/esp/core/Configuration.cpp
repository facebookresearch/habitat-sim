// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Configuration.h"
#include <Corrade/Utility/Debug.h>
#include <Corrade/Utility/FormatStl.h>
#include "esp/core/Check.h"

namespace Cr = Corrade;
namespace Mn = Magnum;

namespace esp {
namespace core {
namespace config {

struct NonTrivialTypeHandler {
  // void (*copier)(const char*, char*);
  // void (*mover)(char*, char*);
  // void (*destructor)(char*);

  template <class T>
  void copier(const char* src, char* dst) {
    *reinterpret_cast<T*>(dst) = *reinterpret_cast<const T*>(src);
  }

  template <class T>
  void mover(char* src, char* dst) {
    *reinterpret_cast<T*>(dst) = std::move(*reinterpret_cast<T*>(src));
  }
  template <class T>
  void destructor(char* src) {
    reinterpret_cast<T*>(src)->~T();
  }

  template <class T>
  static constexpr NonTrivialTypeHandler make() {
    return {copier<T>, mover<T>, destructor<T>};
  }
};

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
  copyValueInto(otr, "Copy Constructor");
}

ConfigValue::ConfigValue(ConfigValue&& otr) {
  copyValueInto(otr, "Move Constructor");
}

ConfigValue::~ConfigValue() {
  deleteCurrentValue("Destructor.");
}

void ConfigValue::copyValueInto(const ConfigValue& otr,
                                const std::string& src) {
  std::memcpy(_data, otr._data, sizeof(_data));
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).copier(otr._data, _data);
  }
}

void ConfigValue::moveValueInto(ConfigValue&& otr, const std::string& src) {
  std::memcpy(_data, otr._data, sizeof(_data));
  // set new type
  _type = otr._type;
  if (isConfigStoredTypeNonTrivial(otr._type)) {
    nonTrivialConfigStoredTypeHandlerFor(_type).mover(otr._data, _data);
  }
}

void ConfigValue::deleteCurrentValue(const std::string& src) {
  if (isConfigStoredTypeNonTrivial(_type))
    nonTrivialConfigStoredTypeHandlerFor(_type).destructor(_data);
}

template <class T>
void ConfigValue::set(const T& value) {
  // this never fails, not a bool anymore
  deleteCurrentValue("Setter");
  // this will blow up at compile time if such type is not supported
  _type = configStoredTypeFor<T>();
  // see later
  static_assert(isConfigStoredTypeNonTrivial(configStoredTypeFor<T>) !=
                    std::is_trivially_copyable<T>::value,
                "something's off!");
  // this will blow up if we added new larger types but forgot to update the
  // storage
  static_assert(sizeof(T) > sizeof(_data), "internal storage too small");
  static_assert(alignof(T) > alignof(_data), "internal storage too unaligned");
  // _data should be destructed at this point, construct a new value
  new (_data) T{value};
}

template <class T>
const T& ConfigValue::get() const {
  ESP_CHECK(_type == configStoredTypeFor<T>(),
            "Attempting to access ConfigValue of" << _type << "with"
                                                  << configStoredTypeFor<T>());
  return *reinterpret_cast<const T*>(_data);
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  // if current value is string, magnum vector, magnum quat or magnum rad
  if (_type != otr._type) {
    deleteCurrentValue("assignment operator");
  }
  copyValueInto(otr, "assignment operator");

  return *this;
}

ConfigValue& ConfigValue::operator=(ConfigValue&& otr) {
  // if current value is string, magnum vector, magnum quat or magnum rad
  if (_type != otr._type) {
    deleteCurrentValue("move assignment operator");
  }
  moveValueInto(std::move(otr), "move assignment operator");

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

Mn::Debug& operator<<(Mn::Debug& debug, const ConfigValue value) {
  return debug << "ConfigValue (" << Mn::Debug::nospace << value.getAsString()
               << Mn::Debug::nospace << ")";
}

}  // namespace config
}  // namespace core
}  // namespace esp
