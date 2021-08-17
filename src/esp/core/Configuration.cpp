// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Configuration.h"
#include "esp/core/Check.h"

namespace esp {
namespace core {

ConfigValue::ConfigValue(const ConfigValue& otr) {
  copyValueInto(otr, "copy constructor");
}

ConfigValue::~ConfigValue() {
  deleteCurrentValue("ConfigValue destructor.");
}

void ConfigValue::copyValueInto(const ConfigValue& otr,
                                const std::string& src) {
  switch (otr.type) {
    case ConfigStoredType::Boolean:
      b = otr.b;
      break;
    case ConfigStoredType::Integer:
      i = otr.i;
      break;
    case ConfigStoredType::Double:
      d = otr.d;
      break;
    case ConfigStoredType::String:
      // placement new
      new (&s) auto(otr.s);
      break;
    case ConfigStoredType::MagnumVec3:
      new (&v) auto(otr.v);
      break;
    case ConfigStoredType::MagnumQuat:
      new (&q) auto(otr.q);
      break;
    case ConfigStoredType::MagnumRad:
      new (&r) auto(otr.r);
      break;
    default:
      ESP_CHECK(true, "Unknown/Unsupported Type in ConfigValue " + src);
  }  // switch
  // set new type
  type = otr.type;
}

void ConfigValue::deleteCurrentValue(const std::string& src) {
  switch (type) {
    case ConfigStoredType::Boolean:
    case ConfigStoredType::Integer:
    case ConfigStoredType::Double:
      // trivially destructible
      break;
    case ConfigStoredType::String:
      s.~basic_string();
      break;
    case ConfigStoredType::MagnumVec3:
      v.~Vector3();
      break;
    case ConfigStoredType::MagnumQuat:
      q.~Quaternion();
      break;
    case ConfigStoredType::MagnumRad:
      r.~Rad();
      break;
    default:
      ESP_CHECK(true, "Unknown/unsupported Type in " + src);
  }  // switch
  i = 0;
  type = ConfigStoredType::Integer;
}

void ConfigValue::set(int _i) {
  checkTypeAndDest(ConfigStoredType::Integer);
  i = _i;
  type = ConfigStoredType::Integer;
}

void ConfigValue::set(bool _b) {
  checkTypeAndDest(ConfigStoredType::Boolean);
  b = _b;
  type = ConfigStoredType::Boolean;
}

void ConfigValue::set(double _d) {
  checkTypeAndDest(ConfigStoredType::Double);
  d = _d;
  type = ConfigStoredType::Double;
}
void ConfigValue::set(const char* _c) {
  if (checkTypeAndDest(ConfigStoredType::String)) {
    s = std::string(_c);
  } else {
    new (&s) std::string(_c);
    type = ConfigStoredType::String;
  }
}

void ConfigValue::set(const std::string& _s) {
  if (checkTypeAndDest(ConfigStoredType::String)) {
    s = _s;
  } else {
    new (&s) auto(_s);
    type = ConfigStoredType::String;
  }
}

void ConfigValue::set(const Magnum::Vector3& _v) {
  if (checkTypeAndDest(ConfigStoredType::MagnumVec3)) {
    v = _v;
  } else {
    new (&v) auto(_v);
    type = ConfigStoredType::MagnumVec3;
  }
}

void ConfigValue::set(const Magnum::Quaternion& _q) {
  if (checkTypeAndDest(ConfigStoredType::MagnumQuat)) {
    q = _q;
  } else {
    new (&q) auto(_q);
    type = ConfigStoredType::MagnumQuat;
  }
}

void ConfigValue::set(const Magnum::Rad& _r) {
  if (checkTypeAndDest(ConfigStoredType::MagnumRad)) {
    r = _r;
  } else {
    new (&r) auto(_r);
    type = ConfigStoredType::MagnumRad;
  }
}

bool ConfigValue::checkTypeAndDest(const ConfigStoredType& checkType) {
  if (type == checkType) {
    return true;
  }
  // if this object's type is not trivially constructible, call its destructor
  deleteCurrentValue("checkTypeAndDest function");
  return false;
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  // if current value is string, magnum vector, magnum quat or magnum rad
  if (type != otr.type) {
    deleteCurrentValue("assignment operator");
  }
  copyValueInto(otr, "assignment operator");

  return *this;
}

std::string ConfigValue::getAsString() const {
  switch (type) {
    case ConfigStoredType::Boolean:
      return (b ? "True" : "False");
    case ConfigStoredType::Integer:
      return std::to_string(i);
    case ConfigStoredType::Double:
      return std::to_string(d);
    case ConfigStoredType::String:
      return s;
    case ConfigStoredType::MagnumVec3: {
      std::string begin = "[";
      return begin.append(std::to_string(v.x()))
          .append(",")
          .append(std::to_string(v.y()))
          .append(",")
          .append(std::to_string(v.z()))
          .append("]");
    }
    case ConfigStoredType::MagnumQuat: {
      std::string begin = "[";
      return begin.append(std::to_string(q.vector().x()))
          .append(",")
          .append(std::to_string(q.vector().y()))
          .append(",")
          .append(std::to_string(q.vector().z()))
          .append(",")
          .append(std::to_string(q.scalar()))
          .append("]");
    }
    case ConfigStoredType::MagnumRad:
      return std::to_string(r.operator float());
    default:
      ESP_CHECK(true, "Unknown/unsupported Type in ConfigValue::getAsString.");
  }  // switch
}  // ConfigValue::getAsString

bool ConfigValue::putValueInConfigGroup(
    const std::string& key,
    Corrade::Utility::ConfigurationGroup& cfg) const {
  switch (type) {
    case ConfigStoredType::Boolean:
      return cfg.setValue(key, b);
    case ConfigStoredType::Integer:
      return cfg.setValue(key, i);
    case ConfigStoredType::Double:
      return cfg.setValue(key, d);
    case ConfigStoredType::String:
      return cfg.setValue(key, s);
    case ConfigStoredType::MagnumVec3:
      return cfg.setValue(key, v);
    case ConfigStoredType::MagnumQuat:
      return cfg.setValue(key, q);
    case ConfigStoredType::MagnumRad:
      return cfg.setValue(key, r);
    default:
      ESP_CHECK(
          true,
          "Unknown/unsupported Type in ConfigValue::putValueInConfigGroup.");
  }  // switch
  return false;
}  // ConfigValue::putValueInConfigGroup

// internal check on get
void ConfigValue::getterTypeCheck(const ConfigStoredType& checkType) const {
  // If attempting to get a value that is not the type of this ConfigType,
  // fail
  ESP_CHECK(type == checkType,
            "Attempting to access incorrect type in ConfigValue.");
}

}  // namespace core
}  // namespace esp
