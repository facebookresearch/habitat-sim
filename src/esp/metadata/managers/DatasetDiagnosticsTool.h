
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_
#define ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_

#include "esp/core/Esp.h"
#include "esp/io/Json.h"

namespace esp {
namespace metadata {
namespace managers {

/**
 * @brief This enum class defines the various dataset diagnostics and remedies
 * that Habitat-Sim can accept and process. These flags will be specified in
 * config files and consumed by the config's manager.
 */
enum class DSDiagnosticType : uint32_t {
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
   * @brief End cap value - no Dataset Diagnostic Type enums should be
   * defined at or past this enum.
   */
  EndDSDiagnosticTypesType = (1U << 31),
};

/**
 * @brief Constant map to provide mappings from string tags to @ref
 * DSDiagnosticType values. This will be used to match values set
 * in json for requested dataset diagnostics to @ref DSDiagnosticType values.
 */
const extern std::map<std::string, DSDiagnosticType> DSDiagnosticTypeMap;

/**
 * @brief This class will track requested diagnostics for specific config
 * files/attributes. It is intended that each @ref AbstractAttributesManager
 * would instantiate one of these objects and use it to determine various
 * diagnostic behavior.
 */
class DatasetDiagnosticsTool {
 public:
  DatasetDiagnosticsTool() {}

  /**
   * @brief Set diagnostic values based on specifications in passed @p _jsonObj.
   * @param _jsonObj The json object referenced by the appropriate diagnostics
   * tag.
   * @param _msgStr A string containing the type of manager and the name of the
   * attributes/config responsible for this call, for use in debug messages.
   * @return Whether the diagnostic values were set successfully or not.
   */
  bool setDiagnosticesFromJson(const io::JsonGenericValue& _jsonObj,
                               const std::string& _msgStr);

  /**
   * @brief Take a list of string tags that are defined as keys in
   * @ref DSDiagnosticTypeMap and set their corresponding flag values to true.
   * @param _keyMappings A list of diagnostics to enable.
   * @param _abortOnFail Whether or not to abort the program if a diagnostic
   * string in
   * @p _keyMappings is unable to be mapped (not found in DSDiagnosticType).
   * Otherwise the erroneous value will produce an error message but otherwise
   * will be ignored.
   */
  void enableDiagnostics(const std::vector<std::string>& _keyMappings,
                         bool _abortOnFail) {
    for (const auto& key : _keyMappings) {
      setNamedDiagnostic(key, true, _abortOnFail);
    }
  }  // enableDiagnostics

  /**
   * @brief Enable/disable the diagnostic given by @p _diagnostic based on
   * @p _val.
   *
   * @param _diagnostic The string name of the diagnostic. This must be mappable
   * to a @ref DSDiagnosticType via @p DSDiagnosticTypeMap .
   * @param _val Whether to enable or disable the given diagnostic
   * @param _abortOnFail Whether or not to abort the program if the
   * @p _diagnostic requeseted is not found in  @ref DSDiagnosticType. If false,
   * will print an error message and skip the unknown request.
   * @return Whether the passed string successfully mapped to a known diagnostic
   */
  bool setNamedDiagnostic(const std::string& _diagnostic,
                          bool _val,
                          bool _abortOnFail = false);

  /**
   * @brief Specify whether or not to save any corrected dataset components if
   * they received correction
   */
  void setSaveCorrected(bool _val) {
    _setFlags(DSDiagnosticType::SaveCorrected, _val);
  }
  /**
   * @brief Query whether or not to save any corrected dataset components if
   * they received correction
   */
  bool saveCorrected() const {
    return _getFlags(DSDiagnosticType::SaveCorrected);
  }

  /**
   * @brief Specify whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSceneInstances(bool _val) {
    _setFlags(DSDiagnosticType::TestForDuplicateInstances, _val);
  }
  /**
   * @brief Query whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  bool testDuplicateSceneInstances() const {
    return _getFlags(DSDiagnosticType::TestForDuplicateInstances);
  }

  /**
   * @brief Specify whether or not to test for duplicate semantic region
   * specifications
   * in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSemanticRegions(bool _val) {
    _setFlags(DSDiagnosticType::TestForDuplicateRegions, _val);
  }
  /**
   * @brief Query whether or not to test for duplicate semantic region
   * specifications in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  bool testDuplicateSemanticRegions() const {
    return _getFlags(DSDiagnosticType::TestForDuplicateRegions);
  }

 private:
  inline void _setFlags(DSDiagnosticType _flag, bool _val) {
    if (_val) {
      _diagnosticsFlags |= static_cast<uint32_t>(_flag);
    } else {
      _diagnosticsFlags &= ~static_cast<uint32_t>(_flag);
    }
  }

  inline bool _getFlags(DSDiagnosticType _flag) const {
    return (_diagnosticsFlags & static_cast<uint32_t>(_flag)) ==
           static_cast<uint32_t>(_flag);
  }
  uint32_t _diagnosticsFlags = 0u;

 public:
  ESP_SMART_POINTERS(DatasetDiagnosticsTool)

};  // class DatasetDiagnosticsTool

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_
