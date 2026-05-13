
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSTOOL_H_
#define ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSTOOL_H_

#include "DatasetDiagnostics.h"
#include "esp/core/diagnostics/DiagnosticsTool.h"
#include "esp/io/Json.h"

namespace esp {
namespace metadata {
namespace diagnostics {

/**
 * @brief This class will track requested diagnostics for specific config
 * files/attributes. It is intended that each @ref AbstractAttributesManager
 * would instantiate one of these objects and use it to determine various
 * diagnostic behavior.
 */
class DatasetDiagnosticsTool : public core::diagnostics::DiagnosticsTool {
 public:
  DatasetDiagnosticsTool() : DiagnosticsTool() {}
  /**
   * @brief Set diagnostic values based on specifications in passed @p jsonObj.
   * @param jsonObj The json object referenced by the appropriate diagnostics
   * tag.
   * @param msgStr A string containing the type of manager and the name of the
   * attributes/config responsible for this call, for use in debug messages.
   * @return Whether the diagnostic values were set successfully or not.
   */
  bool setDiagnosticesFromJSON(const io::JsonGenericValue& jsonObj,
                               const std::string& msgStr);

  /**
   * @brief Merge the passed @ref DatasetDiagnosticsTool's @p _diagnosticsFlag
   * settings into this one's, preserving this one's diagnostic requests as
   * well.
   */
  void mergeDiagnosticsTool(const DatasetDiagnosticsTool& tool) {
    mergeDiagnosticsToolInternal(tool);
  }

  /**
   * @brief Take a list of string tags that are defined as keys in
   * @ref DSDiagnosticTypeMap and set their corresponding flag values to true.
   * @param keyMappings A list of diagnostics to enable.
   * @param abortOnFail Whether or not to abort the program if a diagnostic
   * string in
   * @p keyMappings is unable to be mapped (not found in DSDiagnosticType).
   * Otherwise the erroneous value will produce an error message but otherwise
   * will be ignored.
   */
  void enableDiagnostics(const std::vector<std::string>& keyMappings,
                         bool abortOnFail) {
    for (const auto& key : keyMappings) {
      setNamedDiagnostic(key, true, abortOnFail);
    }
  }  // enableDiagnostics

  /**
   * @brief Enable/disable the diagnostic given by @p diagnostic based on
   * @p val.
   *
   * @param diagnostic The string name of the diagnostic. This must be mappable
   * to a @ref DSDiagnosticType via @p DSDiagnosticTypeMap .
   * @param val Whether to enable or disable the given diagnostic
   * @param abortOnFail Whether or not to abort the program if the
   * @p diagnostic requeseted is not found in  @ref DSDiagnosticType. If false,
   * will print an error message and skip the unknown request.
   * @return Whether the passed string successfully mapped to a known diagnostic
   */
  bool setNamedDiagnostic(const std::string& diagnostic,
                          bool val,
                          bool abortOnFail = false) {
    return setNamedDiagnosticInternal(diagnostic, DSDiagnosticTypeMap, val,
                                      abortOnFail);
  }

  /**
   * @brief Specify whether or not we should save any corrected dataset
   * components if they received correction
   */
  void setShouldSaveCorrected(bool val) {
    setFlag(DSDiagnosticType::SaveCorrected, val);
  }
  /**
   * @brief Query whether or not we should save any corrected dataset components
   * if they received correction
   */
  bool shouldSaveCorrected() const {
    return getFlag(DSDiagnosticType::SaveCorrected);
  }

  /**
   * @brief Set that a save is required. This is to bridge from reading the json
   * file into the attributes and registering the attributes to the
   * post-registration code. True means we corrected dataset components due to
   * diagnostic activities and we wish to save those corrected components.
   */
  void setSaveRequired(bool saveRequired) {
    _requiresCorrectedSave = saveRequired;
  }

  /**
   * @brief Reset the DatasetDiagnostics-specific state of the tool
   */
  virtual void resetIndiv() override { clearSaveRequired(); }

  /**
   * @brief Clear save flag set due to specific diagnostic conditions
   */
  void clearSaveRequired() { _requiresCorrectedSave = false; }

  /**
   * @brief Get whether a save is required. This is to bridge from reading the
   * json file into the attributes and registering the attributes to the
   * post-registration code. True means we corrected dataset components due to
   * diagnostic activities and we wish to save those corrected components.
   */
  bool saveRequired() const { return _requiresCorrectedSave; }

  /**
   * @brief Specify whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSceneInstances(bool val) {
    setFlag(DSDiagnosticType::TestForDuplicateInstances, val);
  }
  /**
   * @brief Query whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  bool testDuplicateSceneInstances() const {
    return getFlag(DSDiagnosticType::TestForDuplicateInstances);
  }

  /**
   * @brief Specify whether or not to test for duplicate semantic region
   * specifications
   * in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSemanticRegions(bool val) {
    setFlag(DSDiagnosticType::TestForDuplicateRegions, val);
  }
  /**
   * @brief Query whether or not to test for duplicate semantic region
   * specifications in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  bool testDuplicateSemanticRegions() const {
    return getFlag(DSDiagnosticType::TestForDuplicateRegions);
  }

 private:
  /**
   * @brief Save the attributes current being processed. This flag is only so
   */
  bool _requiresCorrectedSave = false;

 public:
  ESP_SMART_POINTERS(DatasetDiagnosticsTool)

};  // class DatasetDiagnosticsTool

}  // namespace diagnostics
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSTOOL_H_
