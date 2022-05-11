// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"
#include "JsonUtils.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const Magnum::Matrix3& mat,
                             JsonAllocator& allocator) {
  JsonGenericValue arr(rapidjson::kArrayType);
  // save matrix as array of per-row arrays, for readability in json.
  for (int i = 0; i < mat.Size; ++i) {
    // per row vector as sub-array
    arr.PushBack(toJsonValue(mat.row(i), allocator), allocator);
  }
  return arr;
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3& val) {
  if (obj.IsArray() && obj.Size() == 3) {
    for (rapidjson::SizeType i = 0; i < 3; ++i) {
      if (obj[i].IsArray() && obj.Size() == 3) {
        Mn::Vector3 rowVec{};
        bool success = fromJsonValue(obj[i], rowVec);
        if (success) {
          val.setRow(i, rowVec);
        } else {
          // error in per-array-to-vector read already reported.
          return false;
        }
      } else {
        ESP_ERROR() << "Invalid row value specified in JSON Matrix3, index :"
                    << i;
        return false;
      }
    }  // for each matrix row
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
