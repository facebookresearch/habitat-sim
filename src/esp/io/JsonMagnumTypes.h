// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_IO_JSONMAGNUMTYPES_H_
#define ESP_IO_JSONMAGNUMTYPES_H_

/** @file
 * @brief See JsonAllTypes.h. Don't include this header directly in user code.
 */

#include "JsonBuiltinTypes.h"

#include "esp/core/Logging.h"

#include <Corrade/Containers/Optional.h>
#include <Magnum/Magnum.h>
#include <Magnum/Math/Color.h>
#include <Magnum/Math/Matrix3.h>
#include <Magnum/Math/Quaternion.h>

namespace esp {
namespace io {

/**
 * @brief Specialization to handle Magnum::Matrix3 values. Parses passed value
 * into JsonGenericValue.
 *
 * @param mat Source Magnum::Matrix3 to parse into Json
 * @param allocator
 * @return Json value containing data
 */
JsonGenericValue toJsonValue(const Magnum::Matrix3& mat,
                             JsonAllocator& allocator);
/**
 * @brief Specialization to handle Magnum::Matrix3 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, Magnum::Matrix3& val);

/**
 * @brief Specialization to handle Magnum::Vector2 values. Parses passed value
 * into JsonGenericValue.
 *
 * @param vec Source Magnum::Vector2 to parse into Json
 * @param allocator
 * @return Json value containing data
 */
JsonGenericValue toJsonValue(const Magnum::Vector2& vec,
                             JsonAllocator& allocator);
/**
 * @brief Specialization to handle Magnum::Vector2 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector2& val);

/**
 * @brief Specialization to handle Magnum::Vector3 values. Parses passed value
 * into JsonGenericValue.
 *
 * @param vec Source Magnum::Vector3 to parse into Json
 * @param allocator
 * @return Json value containing data
 */
JsonGenericValue toJsonValue(const Magnum::Vector3& vec,
                             JsonAllocator& allocator);
/**
 * @brief Specialization to handle Magnum::Vector3 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector3& val);

/**
 * @brief Specialization to handle Magnum::Vector4 values. Parses passed value
 * into JsonGenericValue.
 *
 * @param vec Source Magnum::Vector4 to parse into Json
 * @param allocator
 * @return Json value containing data
 */
JsonGenericValue toJsonValue(const Magnum::Vector4& vec,
                             JsonAllocator& allocator);
/**
 * @brief Specialization to handle Magnum::Vector4 values. Populate passed @p
 * val with value. Returns whether successfully populated, or not. Logs an error
 * if inappropriate type.
 *
 * @param obj json value to parse
 * @param val destination value to be populated
 * @return whether successful or not
 */
bool fromJsonValue(const JsonGenericValue& obj, Magnum::Vector4& val);

/**
 * @brief Specialization to handle Magnum::Quaternion values. Parses passed
 * value into JsonGenericValue.
 *
 * @param quat Source Magnum::Quaternion to parse into Json
 * @param allocator
 * @return Json value containing data
 */

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
  JsonGenericValue::ConstMemberIterator jsonIter = value.FindMember(name);
  if (jsonIter != value.MemberEnd()) {
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

}  // namespace io
}  // namespace esp

#endif
