// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_DIAGNOSTICS_DIAGNOSTICSTOOL_H_
#define ESP_CORE_DIAGNOSTICS_DIAGNOSTICSTOOL_H_

#include <type_traits>
#include "esp/core/Check.h"
#include "esp/core/Esp.h"

namespace esp {
namespace core {
namespace diagnostics {

/**
 * @brief This class will track requested diagnostics for specific config
 * files/attributes. It is intended that each @ref AbstractAttributesManager
 * would instantiate one of these objects and use it to determine various
 * diagnostic behavior.
 */
class DiagnosticsTool {
 public:
  DiagnosticsTool() = default;

  /**
   * @brief Reset the current state of the Diagnostics tool.
   */
  void reset() {
    clearDiagnosticFlags();
    resetIndiv();
  }

  /**
   * @brief Implementation class-specific reset code
   */
  virtual void resetIndiv() = 0;

  /**
   * @brief Clear all existing diagnostic flag settings.
   */
  void clearDiagnosticFlags() { diagnosticsFlags_ = 0u; }

 protected:
  /**
   * @brief Merge the passed @ref DiagnosticsTool's @p _diagnosticsFlag
   * settings into this one's, preserving this one's diagnostic requests as
   * well. Only referenced internally to ensure tool type consistency.
   */
  void mergeDiagnosticsToolInternal(const DiagnosticsTool& tool) {
    diagnosticsFlags_ |= tool.diagnosticsFlags_;
  }

  /**
   * @brief Enable/disable the diagnostic given by @p diagnostic string based on
   * @p val.
   *
   * @param diagnostic The string name of the diagnostic. This must be mappable
   * to an appropriate enum type via @p diagMap .
   * @param diagMap Map holding the string to enum mapping for the implementing
   * diagnostic tool.
   * @param val Whether to enable or disable the given diagnostic
   * @param abortOnFail Whether or not to abort the program if the
   * @p diagnostic requeseted is not found in  @p diagMap. If false,
   * will print an error message and skip the unknown request.
   * @return Whether the passed string successfully mapped to a known diagnostic
   */
  template <class T>
  bool setNamedDiagnosticInternal(const std::string& diagnostic,
                                  const std::map<std::string, T>& diagMap,
                                  bool val,
                                  bool abortOnFail = false);

  template <class T>
  inline void setFlag(T _flag, bool _val) {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Tool's flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Tool's flag enum must be backed by a uint32_t");
    if (_val) {
      diagnosticsFlags_ |= static_cast<uint32_t>(_flag);
    } else {
      diagnosticsFlags_ &= ~static_cast<uint32_t>(_flag);
    }
  }

  template <class T>
  inline bool getFlag(T _flag) const {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Tool's flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Tool's flag enum must be backed by a uint32_t");
    return (diagnosticsFlags_ & static_cast<uint32_t>(_flag)) ==
           static_cast<uint32_t>(_flag);
  }

 private:
  uint32_t diagnosticsFlags_ = 0u;

 public:
  ESP_SMART_POINTERS(DiagnosticsTool)

};  // class DiagnosticsTool

/////////////////////////////
// Class Template Method Definitions
template <class T>
bool DiagnosticsTool::setNamedDiagnosticInternal(
    const std::string& diagnostic,
    const std::map<std::string, T>& diagMap,
    bool val,
    bool abortOnFail) {
  //_flag must be an enum value
  static_assert(std::is_enum<T>::value,
                "The Diagnostics Tool's flag must be an enum");
  //_flag must be backed by a uint32_t
  static_assert(
      std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
      "The Diagnostics Tool's flag enum must be backed by a uint32_t");
  const std::string diagnosticLC =
      Corrade::Utility::String::lowercase(diagnostic);

  auto mapIter = diagMap.find(diagnosticLC);
  if (abortOnFail) {
    // If not found then abort.
    ESP_CHECK(mapIter != diagMap.end(),
              "Unknown Diagnostic Test requested to be "
                  << (val ? "Enabled :" : "Disabled :") << diagnostic
                  << ". Aborting.");
  } else {
    if (mapIter == diagMap.end()) {
      ESP_ERROR() << "Unknown Diagnostic Test requested to be "
                  << (val ? "Enabled" : "Disabled") << " :" << diagnostic
                  << " so ignoring request.";
      return false;
    }
  }
  setFlag(mapIter->second, val);
  return true;
}  // DiagnosticsTool::setNamedDiagnostic

}  // namespace diagnostics
}  // namespace core
}  // namespace esp

#endif  // ESP_CORE_DIAGNOSTICS_DIAGNOSTICSTOOL_H_
