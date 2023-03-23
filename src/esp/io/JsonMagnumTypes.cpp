// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"
#include "JsonUtils.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const Magnum::Matrix3& mat,
                             JsonAllocator& allocator) {
  return toJsonArrayHelper(mat.data(), (mat.Size * mat.Size), allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3& mat) {
  int numElems = mat.Size * mat.Size;
  if (obj.IsArray() && obj.Size() == numElems) {
    for (rapidjson::SizeType i = 0; i < numElems; ++i) {
      if (!fromJsonValue(obj[i], mat.data()[i])) {
        ESP_ERROR() << "Invalid numeric value specified in JSON "
                       "Magnum::Matrix3, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

JsonGenericValue toJsonValue(const Magnum::Vector2& vec,
                             JsonAllocator& allocator) {
  return toJsonArrayHelper(vec.data(), vec.Size, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector2& val) {
  if (obj.IsArray() && obj.Size() == val.Size) {
    for (rapidjson::SizeType i = 0; i < val.Size; ++i) {
      if (!fromJsonValue(obj[i], val.data()[i])) {
        ESP_ERROR() << "Invalid numeric value specified in JSON "
                       "Magnum::Vector2, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

JsonGenericValue toJsonValue(const Magnum::Vector3& vec,
                             JsonAllocator& allocator) {
  return toJsonArrayHelper(vec.data(), vec.Size, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector3& val) {
  if (obj.IsArray() && obj.Size() == val.Size) {
    for (rapidjson::SizeType i = 0; i < val.Size; ++i) {
      if (!fromJsonValue(obj[i], val.data()[i])) {
        ESP_ERROR() << "Invalid numeric value specified in JSON "
                       "Magnum::Vector3, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

JsonGenericValue toJsonValue(const Magnum::Vector4& vec,
                             JsonAllocator& allocator) {
  return toJsonArrayHelper(vec.data(), vec.Size, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector4& val) {
  if (obj.IsArray() && obj.Size() == val.Size) {
    for (rapidjson::SizeType i = 0; i < val.Size; ++i) {
      if (!fromJsonValue(obj[i], val.data()[i])) {
        ESP_ERROR() << "Invalid numeric value specified in JSON "
                       "Magnum::Vector4, index :"
                    << i;
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
                     "Magnum::Quaternion, index : 0";
      return false;
    }
    for (rapidjson::SizeType i = 1; i < 4; ++i) {
      if (!fromJsonValue(obj[i], val.vector()[i - 1])) {
        ESP_ERROR() << "Invalid numeric vector value specified in JSON "
                       "Magnum::Quaternion, index :"
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
