
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetDiagnosticsTool.h"
#include "esp/core/Check.h"

namespace esp {
namespace metadata {
namespace managers {

const std::map<std::string, DSDiagnosticType> DSDiagnosticTypeMap = {
    {"SaveCorrected", DSDiagnosticType::SaveCorrected},
    {"TestForSceneInstanceDuplicates",
     DSDiagnosticType::TestForDuplicateInstances},
    {"TestForSemanticRegionDuplicates",
     DSDiagnosticType::TestForDuplicateRegions},
};

bool DatasetDiagnosticsTool::setDiagnosticesFromJson(
    const io::JsonGenericValue& _jsonObj,
    const std::string& _msgStr) {
  if (_jsonObj.IsString()) {
    // Single diagnostic string specified.
    return setNamedDiagnostic(_jsonObj.GetString(), true, true);
  }
  if (_jsonObj.IsArray()) {
    bool success = false;
    for (rapidjson::SizeType i = 0; i < _jsonObj.Size(); ++i) {
      if (_jsonObj[i].IsString()) {
        success = setNamedDiagnostic(_jsonObj[i].GetString(), true, true);
      } else {
        ESP_ERROR(Mn::Debug::Flag::NoSpace)
            << _msgStr
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
      << _msgStr
      << " configuration specifies `request_diagnostics` but specification "
         "is unable to be parsed, so diagnostics request is ignored.";
  return false;
}  // DatasetDiagnosticsTool::setDiagnosticesFromJson

bool DatasetDiagnosticsTool::setNamedDiagnostic(const std::string& _diagnostic,
                                                bool _val,
                                                bool _abortOnFail) {
  auto mapIter = DSDiagnosticTypeMap.find(_diagnostic);
  if (_abortOnFail) {
    // If not found then abort.
    ESP_CHECK(mapIter != DSDiagnosticTypeMap.end(),
              "Unknown Diagnostic Test requested to be "
                  << (_val ? "Enabled" : "Disabled") << " :" << _diagnostic
                  << ". Aborting.");
  } else {
    if (mapIter == DSDiagnosticTypeMap.end()) {
      ESP_ERROR() << "Unknown Diagnostic Test requested to be "
                  << (_val ? "Enabled" : "Disabled") << " :" << _diagnostic
                  << " so ignoring request.";
      return false;
    }
  }
  _setFlags(mapIter->second, _val);
  return true;
}  // DatasetDiagnosticsTool::setNamedDiagnostic

}  // namespace managers
}  // namespace metadata
}  // namespace esp
