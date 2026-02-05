// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_DIAGNOSTICS_DIAGNOSTICSRECORD_H_
#define ESP_CORE_DIAGNOSTICS_DIAGNOSTICSRECORD_H_

#include <type_traits>
#include <unordered_map>
#include "esp/core/Esp.h"

namespace esp {
namespace core {
namespace diagnostics {

/**
 * @brief Construct to provide a foundation for building a record of the results
 * of a series of diagnostics.
 */
class DiagnosticsRecord {
 public:
  explicit DiagnosticsRecord(uint32_t diagnosticsFlags)
      : diagnosticsFlags_(diagnosticsFlags) {}

 protected:
  /**
   * @brief Set the diagnostic flag representing the type of diagnostics
   * recorded by this record
   */
  template <class T>
  inline void setFlag(T flag, bool val) {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Record flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Record flag enum must be backed by a uint32_t");
    if (val) {
      diagnosticsFlags_ |= static_cast<uint32_t>(flag);
    } else {
      diagnosticsFlags_ &= ~static_cast<uint32_t>(flag);
    }
  }

  /**
   * @brief Get the diagnostic flag representing the type of diagnostics
   * recorded by this record
   */
  template <class T>
  inline bool getFlag(T flag) const {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Record flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Record flag enum must be backed by a uint32_t");
    return (diagnosticsFlags_ & static_cast<uint32_t>(flag)) ==
           static_cast<uint32_t>(flag);
  }

  /**
   * @brief Record diagnostic information for specified test.
   * @param diagFlag The diagnostic test enum representing the type of the test.
   * @param report String report to be appended to the @p _resultsLog
   */
  template <class T>
  void recordDiagResult(T diagFlag, const std::string& report) {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Record flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Record flag enum must be backed by a uint32_t");
    // either add a new or retrieve existing
    auto emplaceRes = resultsLog_.emplace(static_cast<uint32_t>(diagFlag),
                                          std::vector<std::string>());
    emplaceRes.first->second.push_back(report);
  }

  /**
   * @brief Retrieve results log for specific diagnostic test type
   */
  template <class T>
  const std::vector<std::string> getDiagResult(T diagFlag) const {
    //_flag must be an enum value
    static_assert(std::is_enum<T>::value,
                  "The Diagnostics Record flag must be an enum");
    //_flag must be backed by a uint32_t
    static_assert(
        std::is_same<typename std::underlying_type<T>::type, uint32_t>::value,
        "The Diagnostics Record flag enum must be backed by a uint32_t");
    auto reportIter = resultsLog_.find(static_cast<uint32_t>(diagFlag));
    if (reportIter == resultsLog_.end()) {
      // Caller should check for empty and respond accordingly
      return {};
    }
    return reportIter->second;
  }  // getDiagResult

 private:
  /**
   * @brief Holds collections of strings recording the results of each enabled
   * diagnostic.
   */
  std::unordered_map<uint32_t, std::vector<std::string>> resultsLog_;
  /**
   * @brief Flags to record the diagnostics performed
   */
  uint32_t diagnosticsFlags_ = 0u;

 public:
  ESP_SMART_POINTERS(DiagnosticsRecord)
};  // class DiagnosticsRecord

}  // namespace diagnostics
}  // namespace core
}  // namespace esp
#endif  // ESP_CORE_DIAGNOSTICS_DIAGNOSTICSRECORD_H_
