
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICS_H_
#define ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICS_H_

#include "esp/core/Esp.h"

namespace esp {
namespace metadata {
namespace diagnostics {

/**
 * @brief This enum class defines the various dataset diagnostics and remedies
 * that Habitat-Sim can accept and process. These flags will be specified in
 * config files and consumed by the config's manager.
 */
enum class DSDiagnosticType : uint32_t {
  /**
   * @brief Unknown/unspecified diagnostic
   */
  Unknown = 0,
  /**
   * @brief Save any dataset configurations that fail a requested diagnostic
   * and are consequently corrected. Ignored if no diagnostics are specified. If
   * diagnostics are specified but this flag is not set then the corrections
   * will only persist during current execution.
   */
  SaveCorrected = 1U,

  /**
   * @brief Test each @ref SceneInstanceAttributes file as it is loaded for
   * duplicate rigid and articulated object instances. Duplicates will have all
   * non-hidden fields equal. This would result in 2 identical objects being
   * instantiated in the same location with the same initial state.
   */
  TestForDuplicateInstances = (1U << 1),

  /**
   * @brief Test each @ref SemanticAttributes file as it is loaded for
   * duplicate region instances. Duplicates will have all non-hidden fields
   * equal. This would result in multiple semantic regions being defined for the
   * same area in the scene.
   */
  TestForDuplicateRegions = (1U << 2),

  /**
   * @brief Shortcut to perform all diagnostics but do not save corrected
   * results.
   */
  AllDiagnostics = ~SaveCorrected,

  /**
   * @brief Shortcut to perform all diagnostics and save corrected results.
   */
  AllDiagnosticsSaveCorrected = ~0U
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * DSDiagnosticType values. This will be used to match values set
 * in json for requested dataset diagnostics to @ref DSDiagnosticType values.
 */
const extern std::map<std::string, DSDiagnosticType> DSDiagnosticTypeMap;

/**
 * @brief This method will convert a @ref DSDiagnosticType value to the
 * string key that maps to it in the DSDiagnosticTypeMap
 */
std::string getDSDiagnosticsTypeName(DSDiagnosticType dsDiagTypeName);

}  // namespace diagnostics
}  // namespace metadata
}  // namespace esp

#endif
