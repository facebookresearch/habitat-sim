// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#pragma once

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "JsonBuiltinTypes.h"

#include <Corrade/Containers/Optional.h>
#include <Magnum/Math/Matrix4.h>

namespace esp {
namespace io {

inline RJsonValue ToRJsonValue(const Magnum::Matrix4& obj,
                               RJsonAllocator& allocator) {
  // perf todo: optimize for common case where last column is (0, 0, 0, 1)
  // TODO: helper for objects that are simply arrays, with error-handling
  RJsonValue floatsArray(rapidjson::kArrayType);
  for (int i = 0; i < obj.Rows * obj.Cols; i++) {
    floatsArray.PushBack(obj.data()[i], allocator);
  }
  return floatsArray;
}

inline void FromRJsonValue(const RJsonValue& floatsArray,
                           Magnum::Matrix4& obj) {
  // TODO: helper for objects that are simply arrays, with error-handling
  ASSERT(floatsArray.Size() == obj.Rows * obj.Cols);
  for (int i = 0; i < floatsArray.Size(); i++) {
    obj.data()[i] = floatsArray[i].GetFloat();
  }
}

inline RJsonValue ToRJsonValue(const Magnum::Vector3& vec,
                               RJsonAllocator& allocator) {
  RJsonValue floatsArray(rapidjson::kArrayType);
  for (int i = 0; i < vec.Size; i++) {
    floatsArray.PushBack(vec.data()[i], allocator);
  }
  return floatsArray;
}

inline void FromRJsonValue(const RJsonValue& floatsArray,
                           Magnum::Vector3& vec) {
  // TODO: helper for objects that are simply arrays, with error-handling
  ASSERT(floatsArray.Size() == vec.Size);
  for (int i = 0; i < vec.Size; i++) {
    vec.data()[i] = floatsArray[i].GetFloat();
  }
}

inline RJsonValue ToRJsonValue(const Magnum::Quaternion& quat,
                               RJsonAllocator& allocator) {
  // [vector object, w] is a verbose way to store a quat. As compared to an
  // array of 4 floats, it has the advantage of being unambiguous about order.
  RJsonValue obj(rapidjson::kObjectType);
  AddMember(obj, "v", quat.vector(), allocator);
  AddMember(obj, "w", quat.scalar(), allocator);
  return obj;
}

inline void FromRJsonValue(const RJsonValue& obj, Magnum::Quaternion& quat) {
  ReadMember(obj, "v", quat.vector());
  ReadMember(obj, "w", quat.scalar());
}

// Containers::Optional is handled differently than ordinary structs. Instead of
// offering ToRJsonValue/FromRJsonValue, we offer AddMember/ReadMember, which
// simply omits adding/reading a value for the case of NullOpt.
template <typename T>
void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Corrade::Containers::Optional<T>& x,
               RJsonAllocator& allocator) {
  if (x) {
    const T& item = *x;
    AddMember(value, name, item, allocator);
  }
}

template <typename T>
void ReadMember(const rapidjson::Value& value,
                const char* name,
                Corrade::Containers::Optional<T>& x) {
  rapidjson::Value::ConstMemberIterator itr = value.FindMember(name);
  if (value.HasMember(name)) {
    x = T();
    ReadMember(value, name, *x);
  } else {
    x = Corrade::Containers::NullOpt;
  }
}

}  // namespace io
}  // namespace esp
