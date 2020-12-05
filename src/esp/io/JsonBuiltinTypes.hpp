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

// These were declared in JsonBuiltinTypes.h. The declarations are placed here,
// after all type-specific ToRJsonValue/FromRJsonValue definitions. The quirky
// ordering is to avoid some compile errors.
template <typename T>
void AddMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const T& obj,
               RJsonAllocator& allocator) {
  // If you get a compile error here, you may need to implement ToRJsonValue
  // for your type in JsonAllTypes.h. See JasonEspTypes.h for examples. Beware
  // you can't directly serialize pointer types.
  AddMember(value, name, ToRJsonValue(obj, allocator), allocator);
}

template <typename T>
void ReadMember(const rapidjson::Value& value, const char* name, T& x) {
  // todo: error-handling on missing name
  // If you get a compile error here, you may need to implement FromRJsonValue
  // for your type in JsonAllTypes.h. See JasonEspTypes.h for examples. Beware
  // you can't directly deserialize pointer types.
  FromRJsonValue(value[name], x);
}

}  // namespace io
}  // namespace esp
