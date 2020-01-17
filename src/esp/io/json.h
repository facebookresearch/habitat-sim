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
