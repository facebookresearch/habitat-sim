// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include <rapidjson/document.h>

#include <string>

namespace esp {
namespace io {

// Make these types easier to read/type. We'll use them everywhere.
using RJsonValue = rapidjson::Value;
using RJsonAllocator = rapidjson::MemoryPoolAllocator<>;

// AddMember wrappers for the 7 rapidjson builtin types
inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      bool obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      int obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      unsigned obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      int64_t obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      uint64_t obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      double obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      float obj,
                      RJsonAllocator& allocator) {
  value.AddMember(name, obj, allocator);
}

// ReadMember wrappers for the 7 rapidjson builtin types
inline void ReadMember(const RJsonValue& value, const char* name, bool& obj) {
  obj = value[name].Get<bool>();
}

inline void ReadMember(const RJsonValue& value, const char* name, int& obj) {
  obj = value[name].Get<int>();
}

inline void ReadMember(const RJsonValue& value,
                       const char* name,
                       unsigned& obj) {
  obj = value[name].Get<unsigned>();
}

inline void ReadMember(const RJsonValue& value,
                       const char* name,
                       int64_t& obj) {
  obj = value[name].Get<int64_t>();
}

inline void ReadMember(const RJsonValue& value,
                       const char* name,
                       uint64_t& obj) {
  obj = value[name].Get<uint64_t>();
}

inline void ReadMember(const RJsonValue& value, const char* name, double& obj) {
  obj = value[name].Get<double>();
}

inline void ReadMember(const RJsonValue& value, const char* name, float& obj) {
  obj = value[name].Get<float>();
}

// TODO: consider RAPIDJSON_HAS_STDSTRING instead of implementing these
// std::string helpers
inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      const std::string& str,
                      RJsonAllocator& allocator) {
  RJsonValue strObj;
  strObj.SetString(str.c_str(), allocator);
  value.AddMember(name, strObj, allocator);
}

inline void ReadMember(const RJsonValue& value,
                       const char* name,
                       std::string& str) {
  const RJsonValue& strObj = value[name];
  str = strObj.GetString();
}

// wrappers intended for enums
template <typename T>
void AddMemberAsUint32(RJsonValue& value,
                       rapidjson::GenericStringRef<char> name,
                       const T& x,
                       RJsonAllocator& allocator) {
  static_assert(sizeof(T) == sizeof(uint32_t), "size match");
  uint32_t xAsUint32 = (uint32_t)x;
  AddMember(value, name, xAsUint32, allocator);
}

template <typename T>
void ReadMemberAsUint32(const RJsonValue& value, const char* name, T& x) {
  static_assert(sizeof(T) == sizeof(uint32_t), "size match");
  uint32_t xAsUint32;
  ReadMember(value, name, xAsUint32);
  x = (T)(xAsUint32);
}

// wrappers for rapidjson's standard Value type
inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      RJsonValue& child,
                      RJsonAllocator& allocator) {
  value.AddMember(name, child, allocator);
}

inline void AddMember(RJsonValue& value,
                      rapidjson::GenericStringRef<char> name,
                      RJsonValue&& child,
                      RJsonAllocator& allocator) {
  value.AddMember(name, child, allocator);
}

// These are template versions of AddMember/ReadMember that will match any
// type T. They expect to find ToJsonValue/FromJsonValue overloads for T.
// These are declared here, before all ToRJsonValue/FromRJsonValue definitions,
// and they are defined later in JsonBuiltinTypes.hpp. The quirky ordering is to
// avoid some compile errors.
template <typename T>
void AddMember(RJsonValue& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               RJsonAllocator& allocator);

template <typename T>
void ReadMember(const RJsonValue& value, const char* name, T& x);

}  // namespace io
}  // namespace esp
