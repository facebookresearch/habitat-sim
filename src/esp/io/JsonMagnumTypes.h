// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONMAGNUMTYPES_H_
#define ESP_IO_JSONMAGNUMTYPES_H_

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "JsonBuiltinTypes.h"

#include "esp/core/logging.h"

#include <Corrade/Containers/Optional.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix4.h>
#include <Magnum/Math/Quaternion.h>

namespace esp {
namespace io {

inline JsonGenericValue toJsonValue(const Magnum::Vector3& vec,
                                    JsonAllocator& allocator) {
  return toJsonArrayHelper(vec.data(), 3, allocator);
}

/**
 * @brief Specialization to handle Magnum::Vector3 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector3& val) {
  if (obj.IsArray() && obj.Size() == 3) {
    for (rapidjson::SizeType i = 0; i < 3; ++i) {
      if (obj[i].IsNumber()) {
        val[i] = obj[i].GetDouble();
      } else {
        LOG(ERROR) << " Invalid numeric value specified in JSON Vec3, index :"
                   << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

inline JsonGenericValue toJsonValue(const Magnum::Color4& color,
                                    JsonAllocator& allocator) {
  return toJsonArrayHelper(color.data(), 4, allocator);
}

/**
 * @brief Specialization to handle Magnum::Color4 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, Magnum::Color4& val) {
  if (obj.IsArray() && obj.Size() == 4) {
    Magnum::Vector4 vec4;
    for (rapidjson::SizeType i = 0; i < 4; ++i) {
      if (obj[i].IsNumber()) {
        vec4[i] = obj[i].GetDouble();
      } else {
        LOG(ERROR) << " Invalid numeric value specified in JSON Color4, index :"
                   << i;
        return false;
      }
    }
    val = Magnum::Color4(vec4);
    return true;
  }
  return false;
}

inline JsonGenericValue toJsonValue(const Magnum::Quaternion& quat,
                                    JsonAllocator& allocator) {
  JsonGenericValue arr(rapidjson::kArrayType);
  arr.PushBack(quat.scalar(), allocator);
  for (int i = 0; i < 3; i++) {
    arr.PushBack(quat.vector()[i], allocator);
  }
  return arr;
}

/**
 * @brief Specialization to handle Magnum::Quaternion values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj,
                          Magnum::Quaternion& val) {
  if (obj.IsArray() && obj.Size() == 4) {
    for (rapidjson::SizeType i = 0; i < 4; ++i) {
      if (obj[i].IsNumber()) {
        if (i == 0) {
          val.scalar() = obj[0].GetFloat();
        } else {
          val.vector()[i - 1] = obj[i].GetFloat();
        }
      } else {
        LOG(ERROR)
            << " Invalid numeric value specified in JSON Quaternion, index :"
            << i;
        return false;
      }
    }
    return true;
  }
  return false;
}

// Containers::Optional is handled differently than ordinary structs. Instead of
// offering toJsonValue/fromJsonValue, we offer addMember/readMember, which
// simply omits adding/reading a value for the case of NullOpt.
template <typename T>
void addMember(rapidjson::Value& value,
               rapidjson::GenericStringRef<char> name,
               const Corrade::Containers::Optional<T>& x,
               JsonAllocator& allocator) {
  if (x) {
    const T& item = *x;
    addMember(value, name, item, allocator);
  }
}

template <typename T>
bool readMember(const rapidjson::Value& value,
                const char* name,
                Corrade::Containers::Optional<T>& x) {
  if (value.HasMember(name)) {
    x = T();
    return readMember(value, name, *x);
  } else {
    x = Corrade::Containers::NullOpt;
    return true;
  }
}

/**
 * @brief Populate passed @p val with value. Returns whether successfully
 * populated, or not. Logs an error if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
inline bool fromJsonValue(const JsonGenericValue& obj, Magnum::Rad& val) {
  if (obj.IsNumber()) {
    val = Magnum::Rad{obj.GetFloat()};
    return true;
  }
  LOG(ERROR) << "Invalid double value";
  return true;
}

}  // namespace io
}  // namespace esp

#endif
