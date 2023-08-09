// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONSTLTYPES_H_
#define ESP_IO_JSONSTLTYPES_H_

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include <map>
#include "JsonBuiltinTypes.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const std::string& str, JsonAllocator& allocator);

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, std::string& val);

template <typename T_first, typename T_second>
inline JsonGenericValue toJsonValue(const std::pair<T_first, T_second>& val,
                                    JsonAllocator& allocator) {
  esp::io::JsonGenericValue obj(rapidjson::kObjectType);
  esp::io::addMember(obj, "first", val.first, allocator);
  esp::io::addMember(obj, "second", val.second, allocator);
  return obj;
}

template <typename T_first, typename T_second>
inline bool fromJsonValue(const JsonGenericValue& obj,
                          std::pair<T_first, T_second>& val) {
  bool success = true;
  success &= readMember(obj, "first", val.first);
  success &= readMember(obj, "second", val.second);
  return success;
}

// For std::vector, we use rapidjson::kArrayType. For an empty vector, we
// omit the member altogether rather than add an empty array.

template <typename T>
void addMember(JsonGenericValue& value,
               rapidjson::GenericStringRef<char> name,
               const std::vector<T>& vec,
               JsonAllocator& allocator) {
  if (!vec.empty()) {
    addMember(value, name, toJsonArrayHelper(vec.data(), vec.size(), allocator),
              allocator);
  }
}

template <typename T>
bool readMember(const JsonGenericValue& value,
                const char* tag,
                std::vector<T>& vec) {
  CORRADE_INTERNAL_ASSERT(vec.empty());
  JsonGenericValue::ConstMemberIterator itr = value.FindMember(tag);
  if (itr != value.MemberEnd()) {
    const JsonGenericValue& arr = itr->value;
    if (!arr.IsArray()) {
      ESP_ERROR() << "JSON tag" << tag << "is not an array";
      return false;
    }
    vec.reserve(arr.Size());
    for (size_t i = 0; i < arr.Size(); ++i) {
      const auto& itemObj = arr[i];
      T item;
      if (!fromJsonValue(itemObj, item)) {
        vec.clear();  // return an empty container on failure
        ESP_ERROR() << "Failed to parse array element" << i << "in JSON tag"
                    << tag;
        return false;
      }
      vec.emplace_back(std::move(item));
    }
  }
  // if the tag isn't found, the container is left empty and we return success
  return true;
}

/**
 * @brief Specialization to handle reading a JSON object into an
 * std::map<std::string, std::string>.  Check passed json doc for existence of
 * passed @p tag and verify it is an object. If present, populate passed @p val
 * with key-value pairs in cell. Returns whether tag is found and successfully
 * populated, or not. Logs an error if tag is found but is inappropriately
 * configured
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination std::map to be populated
 * @return whether successful or not
 */

template <>
inline bool readMember(const JsonGenericValue& d,
                       const char* tag,
                       std::map<std::string, std::string>& val) {
  JsonGenericValue::ConstMemberIterator jsonIter = d.FindMember(tag);
  if (jsonIter != d.MemberEnd()) {
    if (jsonIter->value.IsObject()) {
      const auto& jCell = jsonIter->value;
      for (rapidjson::Value::ConstMemberIterator it = jCell.MemberBegin();
           it != jCell.MemberEnd(); ++it) {
        const std::string key = it->name.GetString();
        if (it->value.IsString()) {
          val.emplace(key, it->value.GetString());
        } else {
          ESP_ERROR() << "Invalid string value specified in JSON config" << tag
                      << "at" << key << ". Skipping.";
        }
      }  // for each value
      return true;
    } else {  // if member is object
      ESP_ERROR() << "Invalid JSON Object value specified in JSON config at"
                  << tag << "; Unable to populate std::map.";
    }
  }  // if has tag
  return false;
}  //  readMember<std::map<std::string, std::string>>

/**
 * @brief Specialization to handle reading a JSON object into an
 * std::map<std::string, float>.  Check passed json doc for existence of
 * passed @p tag and verify it is an object. If present, populate passed @p val
 * with key-value pairs in cell. Returns whether tag is found and successfully
 * populated, or not. Logs an error if tag is found but is inappropriately
 * configured
 *
 * @param d json document to parse
 * @param tag string tag to look for in json doc
 * @param val destination std::map to be populated
 * @return whether successful or not
 */

template <>
inline bool readMember(const JsonGenericValue& d,
                       const char* tag,
                       std::map<std::string, float>& val) {
  JsonGenericValue::ConstMemberIterator jsonIter = d.FindMember(tag);
  if (jsonIter != d.MemberEnd()) {
    if (jsonIter->value.IsObject()) {
      const auto& jCell = jsonIter->value;
      for (rapidjson::Value::ConstMemberIterator it = jCell.MemberBegin();
           it != jCell.MemberEnd(); ++it) {
        const std::string key = it->name.GetString();
        if (it->value.IsFloat()) {
          val.emplace(key, it->value.GetFloat());
        } else {
          ESP_ERROR() << "Invalid float value specified in JSON map" << tag
                      << "at" << key << ". Skipping.";
        }
      }  // for each value
      return true;
    } else {  // if member is object
      ESP_ERROR() << "Invalid JSON Object value specified in JSON config at"
                  << tag << "; Unable to populate std::map.";
    }
  }  // if has tag
  return false;
}  //  readMember<std::map<std::string, float>>

/**
 * @brief Manage string-keyed map of type @p T to json Object
 * @tparam Type of map value
 */
template <typename T>
void addMember(JsonGenericValue& value,
               const rapidjson::GenericStringRef<char>& name,
               const std::map<std::string, T>& mapVal,
               JsonAllocator& allocator) {
  if (!mapVal.empty()) {
    JsonGenericValue objectData(rapidjson::kObjectType);
    for (const auto& elem : mapVal) {
      rapidjson::GenericStringRef<char> key(elem.first.c_str());
      addMember(objectData, key, toJsonValue(elem.second, allocator),
                allocator);
    }
    addMember(value, name, objectData, allocator);
  }
}

}  // namespace io
}  // namespace esp

#endif
