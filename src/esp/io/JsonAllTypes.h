// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONALLTYPES_H_
#define ESP_IO_JSONALLTYPES_H_

/** @file
 * @brief Include file for all helper functions for serializing types to
 * rapidjson.
 *
 * These helpers are designed so that every builtin type and user type can be
 * serialized/deserialized with esp::io::addMember/readMember. The leads to
 * uniform and readable serialization code. To achieve this, every user type
 * defines toJsonValue/fromJsonValue, and then template versions of
 * addMember/readMember will automatically use these.
 *
 * toJsonValue/fromJsonValue for all types should go in the headers below. If
 * the implementation is short, prefer an inline definition (for brevity). If
 * the implementation is long, prefer a separate definition in the corresponding
 * cpp.
 *
 * See IOTest.cpp for example usage.
 */

#include "JsonBuiltinTypes.h"
#include "JsonEspTypes.h"
#include "JsonMagnumTypes.h"
#include "JsonStlTypes.h"

namespace esp {
namespace io {

// These were declared in JsonBuiltinTypes.h. The definitions are placed here,
// after all type-specific toJsonValue/fromJsonValue definitions. The quirky
// ordering is to ensure these definitions see the correct versions of
// toJsonValue/fromJsonValue for the template type objects.

template <typename T>
inline JsonGenericValue toJsonArrayHelper(const T* objects,
                                          int count,
                                          JsonAllocator& allocator) {
  JsonGenericValue arr(rapidjson::kArrayType);
  for (int i = 0; i < count; ++i) {
    arr.PushBack(toJsonValue(objects[i], allocator), allocator);
  }
  return arr;
}

template <typename T>
void addMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               JsonAllocator& allocator) {
  addMember(value, name, toJsonValue(obj, allocator), allocator);
}

template <typename T>
bool readMember(const rapidjson::Value& value, const char* tag, T& x) {
  JsonGenericValue::ConstMemberIterator jsonIter = value.FindMember(tag);

  if (jsonIter == value.MemberEnd()) {
    // return false but don't log an error
    return false;
  }

  if (!fromJsonValue(jsonIter->value, x)) {
    ESP_ERROR() << "Failed to parse JSON tag \"" << tag << "\"";
    return false;
  }
  return true;
}

}  // namespace io
}  // namespace esp

#endif
