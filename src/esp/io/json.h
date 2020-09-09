// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

#include <cstdint>
#define RAPIDJSON_NO_INT64DEFINE
#include <rapidjson/document.h>
#include "esp/core/esp.h"

#include <functional>
#include <string>
#include <vector>

#include <Magnum/Magnum.h>
#include <Magnum/Math/Vector3.h>

namespace esp {
namespace io {

typedef rapidjson::Document JsonDocument;
typedef rapidjson::GenericValue<rapidjson::UTF8<> > JsonGenericValue;

//! Parse JSON file and return as JsonDocument object
JsonDocument parseJsonFile(const std::string& file);

//! Parse JSON string and return as JsonDocument object
JsonDocument parseJsonString(const std::string& jsonString);

//! Return string representation of given JsonDocument
std::string jsonToString(const JsonDocument& d);

//! Return Vec3f coordinates representation of given JsonObject of array type
esp::vec3f jsonToVec3f(const JsonGenericValue& jsonArray);

/**
 * @brief Check passed json doc for existence of passed @ref tag as @tparam T.
 * MUST BE SPECIALIZED due to json tag queries relying on named, type-specific
 * getters.
 *
 * @tparam T type of value to be populated.
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
template <typename T>
bool jsonIntoVal(CORRADE_UNUSED const JsonDocument& d,
                 const char* tag,
                 CORRADE_UNUSED T& val) {
  LOG(ERROR) << "Unsupported typename specified for JSON tag " << tag
             << ". Aborting.";
  return false;
}  // jsonIntoVal template definition

/**
 * @brief Check passed json doc for existence of passed @ref tag as
 * float. If present, populate passed @ref val with value. Returns whether tag
 * is found and successfully populated, or not. Logs an error if tag is found
 * but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
template <>
inline bool jsonIntoVal(const JsonDocument& d, const char* tag, float& val) {
  if (d.HasMember(tag)) {
    if (d[tag].IsNumber()) {
      val = d[tag].GetFloat();
      return true;
    }
    LOG(ERROR) << "Invalid float value specified in JSON config at " << tag;
  }
  return false;
}  // jsonIntoFloat

/**
 * @brief Check passed json doc for existence of passed @ref tag as
 * double. If present, populate passed @ref val with value. Returns whether tag
 * is found and successfully populated, or not. Logs an error if tag is found
 * but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */

template <>
inline bool jsonIntoVal(const JsonDocument& d, const char* tag, double& val) {
  if (d.HasMember(tag)) {
    if (d[tag].IsNumber()) {
      val = d[tag].GetDouble();
      return true;
    }
    LOG(ERROR) << "Invalid double value specified in JSON config at " << tag;
  }
  return false;
}  // jsonIntoDouble

/**
 * @brief Check passed json doc for existence of passed @ref tag as
 * int. If present, populate passed @ref val with value. Returns whether tag
 * is found and successfully populated, or not. Logs an error if tag is found
 * but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */

template <>
inline bool jsonIntoVal(const JsonDocument& d, const char* tag, int& val) {
  if (d.HasMember(tag)) {
    if (d[tag].IsNumber()) {
      val = d[tag].GetInt();
      return true;
    }
    LOG(ERROR) << "Invalid int value specified in JSON config at " << tag;
  }
  return false;
}  // jsonIntoInt

/**
 * @brief Check passed json doc for existence of passed @ref tag as
 * boolean. If present, populate passed @ref val with value. Returns whether tag
 * is found and successfully populated, or not. Logs an error if tag is found
 * but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */

template <>
inline bool jsonIntoVal(const JsonDocument& d, const char* tag, bool& val) {
  if (d.HasMember(tag)) {
    if (d[tag].IsBool()) {
      val = d[tag].GetBool();
      return true;
    }
    LOG(ERROR) << "Invalid boolean value specified in JSON config at " << tag;
  }
  return false;
}  // jsonIntoBool

/**
 * @brief Check passed json doc for existence of passed @ref tag as
 * double. If present, populate passed @ref val with
 * value. Returns whether tag is found and successfully populated, or not. Logs
 * an error if tag is found but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */

template <>
inline bool jsonIntoVal(const JsonDocument& d,
                        const char* tag,
                        std::string& val) {
  if (d.HasMember(tag)) {
    if (d[tag].IsString()) {
      val = d[tag].GetString();
      return true;
    }
    LOG(ERROR) << "Invalid string value specified in JSON config at " << tag;
  }
  return false;
}  // jsonIntoString

/**
 * @brief Specialization to handle Magnum::Vector3 values.  Check passed json
 * doc for existence of passed @ref tag as Magnum::Vector3. If present, populate
 * passed @ref val with value. Returns whether tag is found and successfully
 * populated, or not. Logs an error if tag is found but is inappropriate type.
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */

template <>
inline bool jsonIntoVal(const JsonDocument& d,
                        const char* tag,
                        Magnum::Vector3& val) {
  if (d.HasMember(tag) && d[tag].IsArray() && d[tag].Size() == 3) {
    for (rapidjson::SizeType i = 0; i < 3; ++i) {
      if (d[tag][i].IsNumber()) {
        val[i] = d[tag][i].GetDouble();
      } else {
        LOG(ERROR) << " Invalid numeric value specified in JSON config at "
                   << tag << " index :" << i;
        return false;
      }
    }  // build array
    return true;
  }
  return false;
}  // jsonIntoString

/**
 * @brief Check passed json doc for existence of passed @ref jsonTag as value of
 * type @ref T. If present, populate passed @ref setter with value. Returns
 * whether tag is found and successfully populated, or not. Logs an error if tag
 * is found but is inappropriate type.  Should use explicit type cast on
 * function call if @ref setter is specified using std::bind()
 *
 * @tparam T type of destination variable - must be supported type.
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param setter value setter in some object to populate with the data from
 * json.
 * @return whether successful or not
 */
template <typename T>
bool jsonIntoSetter(const JsonDocument& d,
                    const char* tag,
                    std::function<void(T)> setter) {
  T val;
  if (jsonIntoVal(d, tag, val)) {
    setter(val);
    return true;
  }
  return false;
}  // jsonIntoSetter

/**
 * @brief Check passed json doc for existence of passed @ref jsonTag as value of
 * type @ref T, where the consuming setter will treat the value as const. If
 * present, populate passed @ref setter with value. Returns whether tag is found
 * and successfully populated, or not. Logs an error if tag is found but is
 * inappropriate type.  Should use explicit type cast on function call if @ref
 * setter is specified using std::bind()
 *
 * @tparam T type of destination variable - must be supported type.
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param setter value setter in some object to populate with the data from
 * json.
 * @return whether successful or not
 */
template <typename T>
bool jsonIntoConstSetter(const JsonDocument& d,
                         const char* tag,
                         std::function<void(const T)> setter) {
  T val;
  if (jsonIntoVal(d, tag, val)) {
    setter(val);
    return true;
  }
  return false;
}  // jsonIntoArraySetter

template <typename GV, typename T>
void toVector(const GV& arr,
              std::vector<T>* vec,
              const std::function<T(const GV&)>& conv) {
  const unsigned n = arr.Size();
  vec->resize(n);
  for (unsigned i = 0; i < n; i++) {
    (*vec)[i] = conv(arr[i]);
  }
}

template <typename GV>
void toIntVector(const GV& value, std::vector<int>* vec) {
  const auto conv = [](const GV& x) { return x.GetInt(); };
  toVector<GV, int>(value, vec, conv);
}

template <typename GV>
void toInt64Vector(const GV& value, std::vector<int64_t>* vec) {
  const auto conv = [](const GV& x) { return x.GetInt64(); };
  toVector<GV, int64_t>(value, vec, conv);
}

template <typename GV>
void toFloatVector(const GV& value, std::vector<float>* vec) {
  const auto conv = [](const GV& x) {
    return static_cast<float>(x.GetDouble());
  };
  toVector<GV, float>(value, vec, conv);
}

template <typename GV>
void toDoubleVector(const GV& value, std::vector<double>* vec) {
  const auto conv = [](const GV& x) { return x.GetDouble(); };
  toVector<GV, double>(value, vec, conv);
}

}  // namespace io
}  // namespace esp
