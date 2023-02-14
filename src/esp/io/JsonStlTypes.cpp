// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "JsonAllTypes.h"

namespace esp {
namespace io {

JsonGenericValue toJsonValue(const std::string& str, JsonAllocator& allocator) {
  JsonGenericValue strObj;
  strObj.SetString(str.c_str(), allocator);
  return strObj;
}

bool fromJsonValue(const JsonGenericValue& obj, std::string& val) {
  if (obj.IsString()) {
    val = obj.GetString();
    return true;
  }
  ESP_ERROR() << "Invalid string value";
  return false;
}

}  // namespace io
}  // namespace esp
