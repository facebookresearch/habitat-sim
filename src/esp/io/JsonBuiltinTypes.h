// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONBUILTINTYPES_H_
#define ESP_IO_JSONBUILTINTYPES_H_

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "esp/core/Logging.h"

#include <Corrade/Utility/Macros.h>

#include <rapidjson/document.h>

#include <string>
#include <typeinfo>

namespace esp {
namespace io {

// Make these types easier to read/type. We'll use them everywhere.
typedef rapidjson::GenericValue<rapidjson::UTF8<> > JsonGenericValue;
typedef rapidjson::MemoryPoolAllocator<> JsonAllocator;

// template addMember/readMember to match any type. These are declared here,
// before all toJsonValue/fromJsonValue definitions, and they are defined
// later in JsonBuiltinTypes.hpp. The quirky ordering is to avoid some compile
// errors.

template <typename T>
void addMember(JsonGenericValue& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               JsonAllocator& allocator);

template <typename T>
bool readMember(const JsonGenericValue& value, const char* name, T& x);

#if 0  // reference code to produce runtime errors for missing implementations
       // (instead of compile errors)
/**
 * @brief Fallback implementation for fromJsonValue to produce a runtime error
 * for types that haven't implemented fromJsonValue.
 */
template <typename T>
bool fromJsonValue(CORRADE_UNUSED const JsonGenericValue& obj,
                   CORRADE_UNUSED T& val) {
  // If you've already implemented fromJsonValue for your type and you're still
  // hitting this, the underlying issue might be the ordering among json helper
  // definitions.
  ESP_ERROR() << "Unsupported type. Aborting. You need to implement "
                 "fromJsonValue for typeid(T).name() ="
              << typeid(T).name() << ".";
  return false;
}

/**
 * @brief Fallback implementation for toJsonValue to produce a runtime error for
 * types that haven't implemented toJsonValue.
 */
template <typename T>
JsonGenericValue toJsonValue(const T&, JsonAllocator&) {
  // If you've already implemented toJsonValue for your type and you're still
  // hitting this, the underlying issue might be the ordering among json helper
  // definitions.
  ESP_ERROR()
      << "Unsupported type. Aborting. You need to implement toJsonValue "
         "for typeid(T).name() ="
      << typeid(T).name() << ".";
  return JsonGenericValue(rapidjson::kObjectType);
}
#endif

// toJsonValue wrappers for the 7 rapidjson builtin types. A JsonGenericValue
// can be directly constructed from the builtin types.

inline JsonGenericValue toJsonValue(bool x, JsonAllocator&) {
  return JsonGenericValue(x, nullptr);
}

inline JsonGenericValue toJsonValue(int x, JsonAllocator&) {
  return JsonGenericValue(x);
}

inline JsonGenericValue toJsonValue(unsigned x, JsonAllocator&) {
  return JsonGenericValue(x);
}

inline JsonGenericValue toJsonValue(int64_t x, JsonAllocator&) {
  return JsonGenericValue(x);
}

inline JsonGenericValue toJsonValue(uint64_t x, JsonAllocator&) {
  return JsonGenericValue(x);
}

inline JsonGenericValue toJsonValue(double x, JsonAllocator&) {
  return JsonGenericValue(x);
}

inline JsonGenericValue toJsonValue(float x, JsonAllocator&) {
  return JsonGenericValue(x);
}

// fromJsonValue wrappers for the 7 rapidjson builtin types

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, bool& val) {
  if (obj.IsBool()) {
    val = obj.Get<bool>();
    return true;
  }
  ESP_ERROR() << "Invalid boolean value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, int& val) {
  if (obj.IsNumber()) {
    val = obj.Get<int>();
    return true;
  }
  ESP_ERROR() << "Invalid int value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, unsigned& val) {
  if (obj.IsNumber()) {
    val = obj.Get<unsigned>();
    return true;
  }
  ESP_ERROR() << "Invalid unsigned int value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, int64_t& val) {
  if (obj.IsNumber()) {
    val = obj.Get<int64_t>();
    return true;
  }
  ESP_ERROR() << "Invalid int64_t value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, uint64_t& val) {
  if (obj.IsNumber()) {
    val = obj.Get<uint64_t>();
    return true;
  }
  ESP_ERROR() << "Invalid uint64_t value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, double& val) {
  if (obj.IsNumber()) {
    val = obj.GetDouble();
    return true;
  }
  ESP_ERROR() << "Invalid double value";
  return false;
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj string tag to look for in json doc
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, float& val) {
  if (obj.IsNumber()) {
    val = obj.Get<float>();
    return true;
  }
  ESP_ERROR() << "Invalid float value";
  return false;
}

// wrappers intended for enums

template <typename T>
void addMemberAsUint32(JsonGenericValue& value,
                       const rapidjson::GenericStringRef<char>& name,
                       const T& x,
                       JsonAllocator& allocator) {
  static_assert(sizeof(T) == sizeof(uint32_t), "size match");
  uint32_t xAsUint32 = static_cast<uint32_t>(x);
  addMember(value, name, xAsUint32, allocator);
}

template <typename T>
bool readMemberAsUint32(const JsonGenericValue& value, const char* name, T& x) {
  static_assert(sizeof(T) == sizeof(uint32_t), "size match");
  uint32_t xAsUint32 = 0;
  if (readMember(value, name, xAsUint32)) {
    x = static_cast<T>(xAsUint32);
    return true;
  }
  return false;
}

/**
 * @brief Helper to convert an array of objects to a json array object.
 *
 * Don't use this directly to serialize stl vectors; use addMember(d, "myvec",
 * myvec, allocator) instead.
 *
 * Note there is no corresponding "from" helper because that operation requires
 * more error-handling and must be done case-by-case.
 *
 * @param objects pointer to objects
 * @param count
 * @param allocator
 * @return serialized json value
 */
template <typename T>
inline JsonGenericValue toJsonArrayHelper(const T* objects,
                                          int count,
                                          JsonAllocator& allocator);

// wrappers for rapidjson's standard Value type

inline void addMember(JsonGenericValue& value,
                      const rapidjson::GenericStringRef<char>& name,
                      JsonGenericValue& child,
                      JsonAllocator& allocator) {
  value.AddMember(name, child, allocator);
}

inline void addMember(JsonGenericValue& value,
                      const rapidjson::GenericStringRef<char>& name,
                      JsonGenericValue&& child,
                      JsonAllocator& allocator) {
  value.AddMember(name, child, allocator);
}

}  // namespace io
}  // namespace esp

#endif
