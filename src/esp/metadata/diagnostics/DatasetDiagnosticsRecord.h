
// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSRECORD_H_
#define ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSRECORD_H_

#include "esp/core/Esp.h"
#include "esp/core/diagnostics/DiagnosticsRecord.h"
#include "esp/metadata/attributes/AbstractAttributes.h"

namespace esp {
namespace metadata {
namespace diagnostics {

/**
 * @brief Construct to record the results of a series of diagnostics against a
 * single attributes/configuration. TODO: build this record for each diagnostic
 * process to facilitate reporting.
 */
template <class T>
class DSDiagnosticRecord : public core::diagnostics::DiagnosticsRecord {
 public:
  static_assert(
      std::is_base_of<esp::metadata::attributes::AbstractAttributes, T>::value,
      "DSDiagnosticRecord :: Diagnostic record type must be derived "
      "from esp::metadata::attributes::AbstractAttributes");

  typedef std::weak_ptr<T> WeakObjRef;

  explicit DSDiagnosticRecord(uint32_t diagnosticsFlags)
      : DiagnosticsRecord(diagnosticsFlags) {}

  /**
   * @brief set the reference to this diagnostic record's subject
   */
  void setObjectRef(const std::shared_ptr<T>& objRef) { weakObjRef_ = objRef; }

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

 public:
  ESP_SMART_POINTERS(DSDiagnosticRecord)

};  // class DSDiagnosticRecord

}  // namespace diagnostics
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_DIAGNOSTICS_DATASETDIAGNOSTICSRECORD_H_
