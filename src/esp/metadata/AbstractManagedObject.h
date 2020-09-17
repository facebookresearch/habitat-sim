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
 * functionality for an object to be manageable by @ref ManagedContainer class
 * template specializations.
 */
class AbstractManagedObject {
 public:
  virtual ~AbstractManagedObject() = default;
  /**
   * @brief Get the instancing class of the ManagedObject instantior.  Should
   * only be set from implementer's constructor. Used as key in constructor
   * function pointer maps in @ref ManagedContainers.
   */
  virtual std::string getClassKey() const = 0;

  /**
   * @brief Set this attributes name/origin.  Some attributes derive their own
   * names based on their state, such as @ref AbstractPrimitiveAttributes;  in
   * such cases this should be overridden with NOP.
   * @param handle the handle to set.
   */
  virtual void setHandle(const std::string&) = 0;
  virtual std::string getHandle() const = 0;

  /**
   * @brief directory where files used to construct attributes can be found.
   */
  virtual void setFileDirectory(const std::string& fileDirectory) = 0;
  virtual std::string getFileDirectory() const = 0;

  /**
   *  @brief Unique ID referencing attributes
   */
  virtual void setID(int ID) = 0;
  virtual int getID() const = 0;

 protected:
  virtual void setClassKey(const std::string&) = 0;

 public:
  ESP_SMART_POINTERS(AbstractManagedObject)
};  // class AbstractAttributes
}  // namespace metadata
}  // namespace esp
#endif  // ESP_METADATA_ABSTRACTMANAGEDOBJECT_H_
