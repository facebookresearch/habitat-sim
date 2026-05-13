
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#include "DatasetDiagnostics.h"

namespace esp {
namespace metadata {
namespace diagnostics {

const std::map<std::string, DSDiagnosticType> DSDiagnosticTypeMap = {
    {"savecorrected", DSDiagnosticType::SaveCorrected},
    {"sceneinstanceduplicates", DSDiagnosticType::TestForDuplicateInstances},
    {"semanticregionduplicates", DSDiagnosticType::TestForDuplicateRegions},
    // Future diagnostics should be listed here, before "all"
    {"all", DSDiagnosticType::AllDiagnostics},
    {"allsavecorrected", DSDiagnosticType::AllDiagnosticsSaveCorrected},
};

std::string getDSDiagnosticsTypeName(DSDiagnosticType dsDiagTypeName) {
  // this verifies that enum value being checked is supported by string-keyed
  // map. The values below should be the minimum and maximum enums supported by
  // DSDiagnosticTypeMap
  if (dsDiagTypeName <= DSDiagnosticType::Unknown) {
    return "unknown";
  }
  // Must always be valid value
  for (const auto& it : DSDiagnosticTypeMap) {
    if (it.second == dsDiagTypeName) {
      return it.first;
    }
  }
  return "unknown";
}  // getDSDiagnosticsTypeName

}  // namespace diagnostics

}  // namespace metadata
}  // namespace esp
