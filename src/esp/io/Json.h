// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSON_H_
#define ESP_IO_JSON_H_

#include "JsonAllTypes.h"

#include <cstdint>
#define RAPIDJSON_NO_INT64DEFINE
#include <rapidjson/document.h>
#include <functional>
#include <string>
#include <vector>
#include "esp/core/Configuration.h"
#include "esp/core/Esp.h"

#include <Magnum/Magnum.h>
#include <Magnum/Math/Quaternion.h>
#include <Magnum/Math/Vector3.h>

namespace esp {
namespace io {

typedef rapidjson::Document JsonDocument;

/**
 * @brief Write a Json doc to file
 *
 * @param document an already-populated document object
 * @param file
 * @param usePrettyWriter The pretty writer does nice indenting and spacing but
 * leads to larger filesize.
 * @param maxDecimalPlaces Set this to a positive integer to shorten how
 * floats/doubles are written. Beware loss of precision in your saved data.
 *
 * @return whether successful or not
 */
bool writeJsonToFile(const JsonDocument& document,
                     const std::string& file,
                     bool usePrettyWriter = true,
                     int maxDecimalPlaces = -1);

//! Parse JSON file and return as JsonDocument object
JsonDocument parseJsonFile(const std::string& file);

//! Parse JSON string and return as JsonDocument object
JsonDocument parseJsonString(const std::string& jsonString);

//! Return string representation of given JsonDocument
std::string jsonToString(const JsonDocument& d);

//! Return Vec3f coordinates representation of given JsonObject of array type
esp::vec3f jsonToVec3f(const JsonGenericValue& jsonArray);

/**
 * @brief Recursively load data into a @ref esp::core::Configuration based on a
 * json file.
 * @param jsonObj The source json being read
 * @param configPtr Shared pointer to configuration to populate with the JSON
 * data
 * @return The number of configuration settings successfully read.
 */
int loadJsonIntoConfiguration(
    const JsonGenericValue& jsonObj,
    const std::shared_ptr<esp::core::config::Configuration>& configPtr);

/**
 * @brief Recursively traverse passed @ref esp::core::Configuration to write all
 * values into a JSON document, and save this document to the passed @p
 * filename.
 * @param filename The fully qualified filename to save the document to.
 * @param configPtr Shared pointer to configuration to read into JSON document
 * @return Whether successful or not.
 */
bool writeConfigurationToJsonFile(
    const std::string& filename,
    const std::shared_ptr<esp::core::config::Configuration>& configPtr);

/**
 * @brief Convert a @ref core::config::Configuration to a Json
 * document.  Unable to use toJsonValue<> paradigm due to link errors/cyclic
 * deps.
 * @param config The configuration to build the Json from
 * @param allocator The owning document's allocator.
 * @return The Json object holding the data from the configuration
 */
JsonGenericValue configToJsonValue(
    const std::shared_ptr<core::config::Configuration>& configPtr,
    JsonAllocator& allocator);

/**
 * @brief Convert a @ref core::config::ConfigValue to a
 * Json document.  Unable to use toJsonValue<> paradigm due to link
 * errors/cyclic deps.
 * @param cfgVal The @ref core::config::ConfigValue we want to save in Json
 * @param allocator Allocator for realloc mem.
 */
JsonGenericValue cfgValToJsonValue(const core::config::ConfigValue& cfgVal,
                                   JsonAllocator& allocator);

/**
 * @brief Check passed json doc for existence of passed jsonTag as value of
 * type T. If present, populate passed setter with value. Returns
 * whether tag is found and successfully populated, or not. Logs an error if
 * @p tag is found but is inappropriate type.
 *
 * @tparam T type of destination variable - must be supported type.
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param setter value setter in some object to populate with the data from
 * json.
 * @return whether successful or not
 */
template <typename T>
bool jsonIntoSetter(const JsonGenericValue& d,
                    const char* tag,
                    std::function<void(T)> setter) {
  T val;
  if (readMember(d, tag, val)) {
    setter(val);
    return true;
  }
  return false;
}  // jsonIntoSetter

/**
 * @brief Check passed json doc for existence of passed jsonTag as value of
 * type T, where the consuming setter will treat the value as const. If
 * present, populate passed setter with value. Returns whether @p tag is found
 * and successfully populated, or not. Logs an error if tag is found but is
 * inappropriate type.
 *
 * @tparam T type of destination variable - must be supported type.
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param setter value setter in some object to populate with the data from
 * json.
 * @return whether successful or not
 */
template <typename T>
bool jsonIntoConstSetter(const JsonGenericValue& d,
                         const char* tag,
                         std::function<void(const T)> setter) {
  T val;
  if (readMember(d, tag, val)) {
    setter(val);
    return true;
  }
  return false;
}  // jsonIntoConstSetter

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

#endif  // ESP_IO_JSON_H_
