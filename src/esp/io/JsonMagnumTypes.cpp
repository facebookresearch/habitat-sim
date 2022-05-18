// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"
#include "JsonUtils.h"

namespace esp {
namespace io {

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3& mat) {
  int numElems = mat.Size * mat.Size;
  if (obj.IsArray() && obj.Size() == numElems) {
    for (rapidjson::SizeType i = 0; i < numElems; ++i) {
      if (!fromJsonValue(obj[i], mat.data()[i])) {
        ESP_ERROR()
            << "Invalid numeric value specified in JSON Matrix3, index :" << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

JsonGenericValue toJsonValue(const Magnum::Quaternion& quat,
                             JsonAllocator& allocator) {
  JsonGenericValue arr(rapidjson::kArrayType);
  // note squashing
  arr.PushBack(squashTinyDecimals(quat.scalar()), allocator);
  for (int i = 0; i < 3; ++i) {
    arr.PushBack(squashTinyDecimals(quat.vector()[i]), allocator);
  }
  return arr;
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Quaternion& val) {
  if (obj.IsArray() && obj.Size() == 4) {
    if (!fromJsonValue(obj[0], val.scalar())) {
      ESP_ERROR() << "Invalid numeric scalar value specified in JSON "
                     "Quaternion, index : 0";
      return false;
    }
    for (rapidjson::SizeType i = 1; i < 4; ++i) {
      if (!fromJsonValue(obj[i], val.vector()[i - 1])) {
        ESP_ERROR() << "Invalid numeric vector value specified in JSON "
                       "Quaternion, index :"
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
