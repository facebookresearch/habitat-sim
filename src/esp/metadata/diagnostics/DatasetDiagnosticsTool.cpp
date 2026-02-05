
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetDiagnosticsTool.h"
#include "DatasetDiagnostics.h"
#include "esp/core/Check.h"

namespace esp {
namespace metadata {
namespace diagnostics {

bool DatasetDiagnosticsTool::setDiagnosticesFromJSON(
    const io::JsonGenericValue& jsonObj,
    const std::string& msgStr) {
  if (jsonObj.IsString()) {
    // Single diagnostic string specified.
    return setNamedDiagnostic(jsonObj.GetString(), true, true);
  }
  if (jsonObj.IsArray()) {
    bool success = false;
    for (rapidjson::SizeType i = 0; i < jsonObj.Size(); ++i) {
      if (jsonObj[i].IsString()) {
        success = setNamedDiagnostic(jsonObj[i].GetString(), true, true);
      } else {
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << msgStr
            << " configuration specifies `request_diagnostics` array with a "
               "non-string element @idx "
            << i << ". Skipping unknown element.";
      }
    }
    return success;
  }
  // else json object support? tag->boolean map in json?
  // Tag present but referneces neither a string nor an array
  ESP_ERROR(Mn::Debug::Flag::NoSpace)
      << msgStr
      << " configuration specifies `request_diagnostics` but specification "
         "is unable to be parsed as either a string or an array, so "
         "diagnostics request is ignored.";
  return false;
}  // DatasetDiagnosticsTool::setDiagnosticesFromJSON

}  // namespace diagnostics
}  // namespace metadata
}  // namespace esp
