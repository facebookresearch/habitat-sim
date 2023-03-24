// Copyright (c) Meta Platforms, Inc. All Rights Reserved
// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"
#include "JsonUtils.h"

namespace esp {
namespace io {

namespace {

template <typename T>
JsonGenericValue toJsonValueMatrixHelper(const T& mat,
                             JsonAllocator& allocator) {
  constexpr auto numItems = T::Cols * T::Rows;
  return toJsonArrayHelper(mat.data(), numItems, allocator); 
}

template <typename T>
bool fromJsonValueMatrixHelper(const JsonGenericValue& obj, T& val) {
  constexpr auto numItems = T::Cols * T::Rows;
  if (obj.IsArray() && obj.Size() == numItems) {
    for (rapidjson::SizeType i = 0; i < numItems; ++i) {
      if (obj[i].IsNumber()) {
        val.data()[i] = obj[i].GetDouble();
      } else {
        ESP_ERROR() << "Invalid numeric value specified in JSON Matrix, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

template <typename T>
JsonGenericValue toJsonValueVectorHelper(const T& vec,
                                    JsonAllocator& allocator) {
  constexpr auto numItems = T::Size;
  return toJsonArrayHelper(vec.data(), numItems, allocator);
}

template <typename T>
bool fromJsonValueVectorHelper(const JsonGenericValue& obj, T& val) {
  constexpr auto numItems = T::Size;
  if (obj.IsArray() && obj.Size() == numItems) {
    for (rapidjson::SizeType i = 0; i < numItems; ++i) {
      if (obj[i].IsNumber()) {
        val[i] = obj[i].GetDouble();
      } else {
        ESP_ERROR() << "Invalid numeric value specified in JSON Vector, index :"
                    << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

}

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


bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3x3& val) {

  return fromJsonValueMatrixHelper(obj, val);
}

JsonGenericValue toJsonValue(const Magnum::Matrix3x3& mat, 
    JsonAllocator& allocator) {
  return toJsonValueMatrixHelper(mat, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Range3D& val) {
  bool success = true;
  success &= readMember(obj, "min", val.min());
  success &= readMember(obj, "max", val.max());
  return success;
}

JsonGenericValue toJsonValue(const Magnum::Range3D& x, 
    JsonAllocator& allocator) {
  JsonGenericValue obj(rapidjson::kObjectType);
  addMember(obj, "min", x.min(), allocator);
  addMember(obj, "max", x.max(), allocator);
  return obj;
}

JsonGenericValue toJsonValue(const Magnum::Vector3& vec,
                                    JsonAllocator& allocator) {
  return toJsonValueVectorHelper(vec, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector3& val) {
  return fromJsonValueVectorHelper(obj, val);
}

JsonGenericValue toJsonValue(const Magnum::Vector2& vec,
                                    JsonAllocator& allocator) {
  return toJsonValueVectorHelper(vec, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector2& val) {
  return fromJsonValueVectorHelper(obj, val);
}

JsonGenericValue toJsonValue(const Magnum::Color4& color,
                                    JsonAllocator& allocator) {
  return toJsonValueVectorHelper(color, allocator);
}

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Color4& val) {
  return fromJsonValueVectorHelper(obj, val);
}


}  // namespace io
}  // namespace esp
