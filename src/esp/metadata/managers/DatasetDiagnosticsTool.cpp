
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetDiagnosticsTool.h"
#include "esp/core/Check.h"

namespace esp {
namespace metadata {
namespace managers {

const std::map<std::string, DSDiagnosticType> DSDiagnosticTypeMap = {
    {"savecorrected", DSDiagnosticType::SaveCorrected},
    {"testforsceneinstanceduplicates",
     DSDiagnosticType::TestForDuplicateInstances},
    {"testforsemanticregionduplicates",
     DSDiagnosticType::TestForDuplicateRegions},
    // Future diagnostics should be listed here
    {"all", DSDiagnosticType::AllDiagnostics},
    {"allsavecorrected", DSDiagnosticType::AllDiagnosticsSaveCorrected},
};

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
         "is unable to be parsed, so diagnostics request is ignored.";
  return false;
}  // DatasetDiagnosticsTool::setDiagnosticesFromJSON

bool DatasetDiagnosticsTool::setNamedDiagnostic(const std::string& diagnostic,
                                                bool val,
                                                bool abortOnFail) {
  const std::string diagnosticLC = Cr::Utility::String::lowercase(diagnostic);

  auto mapIter = DSDiagnosticTypeMap.find(diagnosticLC);
  if (abortOnFail) {
    // If not found then abort.
    ESP_CHECK(mapIter != DSDiagnosticTypeMap.end(),
              "Unknown Diagnostic Test requested to be "
                  << (val ? "Enabled" : "Disabled") << " :" << diagnostic
                  << ". Aborting.");
  } else {
    if (mapIter == DSDiagnosticTypeMap.end()) {
      ESP_ERROR() << "Unknown Diagnostic Test requested to be "
                  << (val ? "Enabled" : "Disabled") << " :" << diagnostic
                  << " so ignoring request.";
      return false;
    }
  }
  setFlags(mapIter->second, val);
  return true;
}  // DatasetDiagnosticsTool::setNamedDiagnostic

}  // namespace managers
}  // namespace metadata
}  // namespace esp
