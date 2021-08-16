// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "Configuration.h"
#include "esp/core/Check.h"

namespace esp {
namespace core {

ConfigValue::ConfigValue() : i{0}, type{ConfigStoredType::INT} {}

ConfigValue::ConfigValue(const ConfigValue& otr) {
  switch (otr.type) {
    case ConfigStoredType::BOOL:
      b = otr.b;
      break;
    case ConfigStoredType::INT:
      i = otr.i;
      break;
    case ConfigStoredType::DOUBLE:
      d = otr.d;
      break;
    case ConfigStoredType::STRING:
      // placement new
      new (&s) auto(otr.s);
      break;
    case ConfigStoredType::MN_VEC3:
      new (&v) auto(otr.v);
      break;
    case ConfigStoredType::MN_QUAT:
      new (&q) auto(otr.q);
      break;
    case ConfigStoredType::MN_RAD:
      new (&r) auto(otr.r);
      break;
    default:
      ESP_CHECK(true,
                "Unknown/unsupported Type in ConfigValue copy constructor");
  }  // switch
  // set new type
  type = otr.type;
}

ConfigValue::~ConfigValue() {
  switch (type) {
    case ConfigStoredType::BOOL:
    case ConfigStoredType::INT:
    case ConfigStoredType::DOUBLE:
      // trivially destructible
      break;
    case ConfigStoredType::STRING:
      s.~basic_string();
      break;
    case ConfigStoredType::MN_VEC3:
      v.~Vector3();
      break;
    case ConfigStoredType::MN_QUAT:
      q.~Quaternion();
      break;
    case ConfigStoredType::MN_RAD:
      r.~Rad();
      break;
    default:
      ESP_CHECK(true, "Unknown/unsupported Type in ConfigValue destructor.");
  }  // switch
}

void ConfigValue::set(int _i) {
  checkTypeAndDest(ConfigStoredType::INT);
  i = _i;
  type = ConfigStoredType::INT;
}

void ConfigValue::set(bool _b) {
  checkTypeAndDest(ConfigStoredType::BOOL);
  b = _b;
  type = ConfigStoredType::BOOL;
}

void ConfigValue::set(double _d) {
  checkTypeAndDest(ConfigStoredType::DOUBLE);
  d = _d;
  type = ConfigStoredType::DOUBLE;
}
void ConfigValue::set(const char* _c) {
  if (checkTypeAndDest(ConfigStoredType::STRING)) {
    s = std::string(_c);
  } else {
    new (&s) std::string(_c);
    type = ConfigStoredType::STRING;
  }
}

void ConfigValue::set(const std::string& _s) {
  if (checkTypeAndDest(ConfigStoredType::STRING)) {
    s = _s;
  } else {
    new (&s) auto(_s);
    type = ConfigStoredType::STRING;
  }
}

void ConfigValue::set(const Magnum::Vector3& _v) {
  if (checkTypeAndDest(ConfigStoredType::MN_VEC3)) {
    v = _v;
  } else {
    new (&v) auto(_v);
    type = ConfigStoredType::MN_VEC3;
  }
}

void ConfigValue::set(const Magnum::Quaternion& _q) {
  if (checkTypeAndDest(ConfigStoredType::MN_QUAT)) {
    q = _q;
  } else {
    new (&q) auto(_q);
    type = ConfigStoredType::MN_QUAT;
  }
}

void ConfigValue::set(const Magnum::Rad& _r) {
  if (checkTypeAndDest(ConfigStoredType::MN_RAD)) {
    r = _r;
  } else {
    new (&r) auto(_r);
    type = ConfigStoredType::MN_RAD;
  }
}

bool ConfigValue::checkTypeAndDest(const ConfigStoredType& checkType) {
  if (type == checkType) {
    return true;
  }
  // if this object's type is not trivially constructible, call its destructor
  if (type == ConfigStoredType::STRING) {
    s.~basic_string();
  } else if (type == ConfigStoredType::MN_VEC3) {
    v.~Vector3();
  } else if (type == ConfigStoredType::MN_QUAT) {
    q.~Quaternion();
  } else if (type == ConfigStoredType::MN_RAD) {
    r.~Rad();
  }
  return false;
}

ConfigValue& ConfigValue::operator=(const ConfigValue& otr) {
  // if current value is string, magnum vector, magnum quat or magnum rad
  // check if otr is same type, otherwise clear out type
  if (type == ConfigStoredType::STRING) {
    if (otr.type == ConfigStoredType::STRING) {
      // if both are strings
      s = otr.s;  // usual string assignment
      return *this;
    }
    // other is not string, so destruct string
    s.~basic_string();
  } else if (type == ConfigStoredType::MN_VEC3) {
    if (otr.type == ConfigStoredType::MN_VEC3) {
      v = otr.v;
      return *this;
    }
    v.~Vector3();
  } else if (type == ConfigStoredType::MN_QUAT) {
    if (otr.type == ConfigStoredType::MN_QUAT) {
      q = otr.q;
      return *this;
    }
    q.~Quaternion();
  } else if (type == ConfigStoredType::MN_RAD) {
    if (otr.type == ConfigStoredType::MN_RAD) {
      r = otr.r;
      return *this;
    }
    r.~Rad();
  }

  switch (otr.type) {
    case ConfigStoredType::BOOL:
      b = otr.b;
      break;
    case ConfigStoredType::INT:
      i = otr.i;
      break;
    case ConfigStoredType::DOUBLE:
      d = otr.d;
      break;
    case ConfigStoredType::STRING:
      // placement new
      new (&s) auto(otr.s);
      break;
    case ConfigStoredType::MN_VEC3:
      new (&v) auto(otr.v);
      break;
    case ConfigStoredType::MN_QUAT:
      new (&q) auto(otr.q);
      break;
    case ConfigStoredType::MN_RAD:
      new (&r) auto(otr.r);
      break;
    default:
      ESP_CHECK(true, "Unknown/unsupported Type in ConfigValue operator=.");
  }  // switch
  // set new type
  type = otr.type;
  return *this;
}

std::string ConfigValue::getAsString() const {
  switch (type) {
    case ConfigStoredType::BOOL:
      return (b ? "True" : "False");
    case ConfigStoredType::INT:
      return std::to_string(i);
    case ConfigStoredType::DOUBLE:
      return std::to_string(d);
    case ConfigStoredType::STRING:
      return s;
    case ConfigStoredType::MN_VEC3: {
      std::string begin = "[";
      return begin.append(std::to_string(v.x()))
          .append(",")
          .append(std::to_string(v.y()))
          .append(",")
          .append(std::to_string(v.z()))
          .append("]");
    }
    case ConfigStoredType::MN_QUAT: {
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
    case ConfigStoredType::MN_RAD:
      return std::to_string(r.operator float());
    default:
      ESP_CHECK(true, "Unknown/unsupported Type in ConfigValue::getAsString.");
  }  // switch
}  // ConfigValue::getAsString

bool ConfigValue::putValueInConfigGroup(
    const std::string& key,
    Corrade::Utility::ConfigurationGroup& cfg) const {
  switch (type) {
    case ConfigStoredType::BOOL:
      return cfg.setValue(key, b);
    case ConfigStoredType::INT:
      return cfg.setValue(key, i);
    case ConfigStoredType::DOUBLE:
      return cfg.setValue(key, d);
    case ConfigStoredType::STRING:
      return cfg.setValue(key, s);
    case ConfigStoredType::MN_VEC3:
      return cfg.setValue(key, v);
    case ConfigStoredType::MN_QUAT:
      return cfg.setValue(key, q);
    case ConfigStoredType::MN_RAD:
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
