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

// template AddMember/ReadMember to match any type. These are declared here,
// before all ToRJsonValue/FromRJsonValue definitions, and they are defined
// later in JsonBuiltinTypes.hpp. The quirky ordering is to avoid some compile
// errors.
template <typename T>
void AddMember(RJsonValue& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               RJsonAllocator& allocator);

template <typename T>
void ReadMember(const RJsonValue& value, const char* name, T& x);

// ToRJsonValue wrappers for the 7 rapidjson builtin types. A ToRJsonValue can
// be directly constructed from the builtin types.
inline RJsonValue ToRJsonValue(bool x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(int x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(unsigned x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(int64_t x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(uint64_t x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(double x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

inline RJsonValue ToRJsonValue(float x, RJsonAllocator& allocator) {
  return RJsonValue(x);
}

// FromRJsonValue wrappers for the 7 rapidjson builtin types
inline void FromRJsonValue(const RJsonValue& obj, bool& x) {
  x = obj.Get<bool>();
}

inline void FromRJsonValue(const RJsonValue& obj, int& x) {
  x = obj.Get<int>();
}

inline void FromRJsonValue(const RJsonValue& obj, unsigned& x) {
  x = obj.Get<unsigned>();
}

inline void FromRJsonValue(const RJsonValue& obj, int64_t& x) {
  x = obj.Get<int64_t>();
}

inline void FromRJsonValue(const RJsonValue& obj, uint64_t& x) {
  x = obj.Get<uint64_t>();
}

inline void FromRJsonValue(const RJsonValue& obj, double& x) {
  x = obj.Get<double>();
}

inline void FromRJsonValue(const RJsonValue& obj, float& x) {
  x = obj.Get<float>();
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

}  // namespace io
}  // namespace esp
