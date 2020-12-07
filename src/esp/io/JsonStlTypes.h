// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "JsonBuiltinTypes.h"

namespace esp {
namespace io {

// TODO: consider RAPIDJSON_HAS_STDSTRING instead of implementing these
// std::string helpers
inline RJsonValue ToRJsonValue(const std::string& str,
                               RJsonAllocator& allocator) {
  RJsonValue strObj;
  strObj.SetString(str.c_str(), allocator);
  return strObj;
}

inline void FromRJsonValue(const RJsonValue& strObj, std::string& str) {
  str = strObj.GetString();
}

// For std::vector, we use rapidjson::kArrayType. For an empty vector, we
// omit the member altogether rather than add an empty array.
template <typename T>
void AddMember(RJsonValue& value,
               rapidjson::GenericStringRef<char> name,
               const std::vector<T>& vec,
               RJsonAllocator& allocator) {
  if (!vec.empty()) {
    RJsonValue arr(rapidjson::kArrayType);
    for (const auto& item : vec) {
      arr.PushBack(esp::io::ToRJsonValue(item, allocator), allocator);
    }
    AddMember(value, name, arr, allocator);
  }
}

template <typename T>
void ReadMember(const RJsonValue& value,
                const char* name,
                std::vector<T>& vec) {
  RJsonValue::ConstMemberIterator itr = value.FindMember(name);
  if (itr != value.MemberEnd()) {
    const RJsonValue& arr = itr->value;
    vec.reserve(arr.Size());
    for (const auto& itemObj : arr.GetArray()) {
      T item;
      FromRJsonValue(itemObj, item);
      vec.emplace_back(std::move(item));
    }
  }
}

}  // namespace io
}  // namespace esp
