// Copyright (c) Facebook, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_ABSTRACTMANAGEDOBJECT_H_
#define ESP_METADATA_ABSTRACTMANAGEDOBJECT_H_

#include "esp/core/Configuration.h"

namespace esp {
namespace metadata {
/**
 * @brief This abstract base class provides the interface of expected
 * functionality for an object to be manageable by @ref ManagedContainers class
 * template specializations.
 */
class AbstractManagedObject {
 public:
  virtual ~AbstractManagedObject() = default;
  /**
   * @brief Get the instancing class of the ManagedObject instantior.  Should
   * only be set from constructor. Used as key in constructor function pointer
   * maps in @ref ManagedContainers.
   */
  virtual std::string getClassKey() = 0;
  ESP_SMART_POINTERS(AbstractManagedObject)
};  // class AbstractAttributes

}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_ABSTRACTMANAGEDOBJECT_H_