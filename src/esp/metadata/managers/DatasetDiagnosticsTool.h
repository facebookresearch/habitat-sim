
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_
#define ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_

#include "esp/core/Esp.h"
#include "esp/io/Json.h"
#include "esp/metadata/attributes/AbstractAttributes.h"

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
 * @brief Construct to record the results of a series of diagnostics against a
 * single attributes.
 */
template <class T>
class DSDiagnosticRecord {
 public:
  static_assert(
      std::is_base_of<esp::metadata::attributes::AbstractAttributes, T>::value,
      "AbstractManagedPhysicsObject :: Managed physics object type must be "
      "derived from esp::physics::PhysicsObjectBase");

  typedef std::weak_ptr<T> WeakObjRef;

  DSDiagnosticRecord(uint32_t diagnosticsFlags)
      : _diagnosticsFlags(diagnosticsFlags) {}

  /**
   * @brief set the reference to this diagnostic record's subject
   */
  void setObjectRef(const std::shared_ptr<T>& objRef) { weakObjRef_ = objRef; }

  inline void setFlags(DSDiagnosticType _flag, bool _val) {
    if (_val) {
      _diagnosticsFlags |= static_cast<uint32_t>(_flag);
    } else {
      _diagnosticsFlags &= ~static_cast<uint32_t>(_flag);
    }
  }

  inline bool getFlags(DSDiagnosticType _flag) const {
    return (_diagnosticsFlags & static_cast<uint32_t>(_flag)) ==
           static_cast<uint32_t>(_flag);
  }

 protected:
  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's @p weakObjRef_ if it exists; if not, it provides a message.
   * @return Either a shared pointer of this record's object, or nullptr if
   * dne.
   */
  std::shared_ptr<T> inline getObjectReference() const {
    std::shared_ptr<T> sp = weakObjRef_.lock();
    if (!sp) {
      ESP_ERROR()
          << "This attributes no longer exists. Please delete any variable "
             "references.";
    }
    return sp;
  }  // getObjectReference

  // Non-owning reference to attributes this record pertains to.
  WeakObjRef weakObjRef_;

 private:
  uint32_t _diagnosticsFlags = 0u;

 public:
  ESP_SMART_POINTERS(DSDiagnosticRecord)

};  // struct DSDiagnosticRecord

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
  DatasetDiagnosticsTool() = default;

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
    _diagnosticsFlags |= tool._diagnosticsFlags;
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
                          bool abortOnFail = false);

  /**
   * @brief Specify whether or not we should save any corrected dataset
   * components if they received correction
   */
  void setShouldSaveCorrected(bool val) {
    setFlags(DSDiagnosticType::SaveCorrected, val);
  }
  /**
   * @brief Query whether or not we should save any corrected dataset components
   * if they received correction
   */
  bool shouldSaveCorrected() const {
    return getFlags(DSDiagnosticType::SaveCorrected);
  }

  /**
   * @brief Set that a save is required. This is to bridge from reading the json
   * file into the attributes and registering the attributes to the
   * post-registration code.
   */
  void setSaveRequired(bool saveRequired) {
    _requiresCorrectedSave = saveRequired;
  }

  /**
   * @brief Clear any flags set due to specific diagnostics
   */
  void clearSaveRequired() { _requiresCorrectedSave = false; }

  /**
   * @brief Get whether a save is required. This is to bridge from reading the
   * json file into the attributes and registering the attributes to the
   * post-registration code.
   */
  bool saveRequired() const { return _requiresCorrectedSave; }

  /**
   * @brief Specify whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSceneInstances(bool val) {
    setFlags(DSDiagnosticType::TestForDuplicateInstances, val);
  }
  /**
   * @brief Query whether or not to test for duplicate scene object instances
   * in loaded @ref SceneInstanceAttributes and not process the duplicates if found.
   */
  bool testDuplicateSceneInstances() const {
    return getFlags(DSDiagnosticType::TestForDuplicateInstances);
  }

  /**
   * @brief Specify whether or not to test for duplicate semantic region
   * specifications
   * in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  void setTestDuplicateSemanticRegions(bool val) {
    setFlags(DSDiagnosticType::TestForDuplicateRegions, val);
  }
  /**
   * @brief Query whether or not to test for duplicate semantic region
   * specifications in loaded @ref SemanticAttributes and not process the duplicates if found.
   */
  bool testDuplicateSemanticRegions() const {
    return getFlags(DSDiagnosticType::TestForDuplicateRegions);
  }

 private:
  inline void setFlags(DSDiagnosticType _flag, bool _val) {
    if (_val) {
      _diagnosticsFlags |= static_cast<uint32_t>(_flag);
    } else {
      _diagnosticsFlags &= ~static_cast<uint32_t>(_flag);
    }
  }

  inline bool getFlags(DSDiagnosticType _flag) const {
    return (_diagnosticsFlags & static_cast<uint32_t>(_flag)) ==
           static_cast<uint32_t>(_flag);
  }
  uint32_t _diagnosticsFlags = 0u;

  /**
   * @brief Save the attributes current being processed. This flag is only so
   */
  bool _requiresCorrectedSave = false;

 public:
  ESP_SMART_POINTERS(DatasetDiagnosticsTool)

};  // class DatasetDiagnosticsTool

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_DATASETDIAGNOSTICSTOOL_H_
