// Copyright (c) Meta Platforms, Inc. All Rights Reserved
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

JsonGenericValue toJsonValue(const Magnum::Vector3& vec,
                                    JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector3& val);

JsonGenericValue toJsonValue(const Magnum::Vector2& vec,
                                    JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector2& val);


JsonGenericValue toJsonValue(const Magnum::Color4& color,
                                    JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Color4& val);

JsonGenericValue toJsonValue(const Magnum::Quaternion& quat,
                             JsonAllocator& allocator);

/**
 * @brief Specialization to handle Magnum::Quaternion values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, Magnum::Quaternion& val);

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
  ESP_ERROR() << "Invalid double value";
  return true;
}


bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3x3& val);

JsonGenericValue toJsonValue(const Magnum::Matrix3x3& mat, 
    JsonAllocator& allocator);

bool fromJsonValue(const JsonGenericValue& obj, Magnum::Range3D& val);

JsonGenericValue toJsonValue(const Magnum::Range3D& val, 
    JsonAllocator& allocator);

}  // namespace io
}  // namespace esp

#endif
