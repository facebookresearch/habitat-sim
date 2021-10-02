// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"
#include "JsonUtils.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const Magnum::Quaternion& quat,
                             JsonAllocator& allocator) {
  JsonGenericValue arr(rapidjson::kArrayType);
  // note squashing
  arr.PushBack(squashTinyDecimals(quat.scalar()), allocator);
  for (int i = 0; i < 3; i++) {
    arr.PushBack(squashTinyDecimals(quat.vector()[i]), allocator);
  }
  return arr;
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Quaternion& val) {
  if (obj.IsArray() && obj.Size() == 4) {
    for (rapidjson::SizeType i = 0; i < 4; ++i) {
      if (obj[i].IsNumber()) {
        if (i == 0) {
          val.scalar() = obj[0].GetFloat();
        } else {
          val.vector()[i - 1] = obj[i].GetFloat();
        }
      } else {
        ESP_ERROR()
            << "Invalid numeric value specified in JSON Quaternion, index :"
            << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

}  // namespace io
}  // namespace esp
