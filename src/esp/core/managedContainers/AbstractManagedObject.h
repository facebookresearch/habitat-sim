// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_CORE_ABSTRACTMANAGEDOBJECT_H_
#define ESP_CORE_ABSTRACTMANAGEDOBJECT_H_

#include "esp/core/Esp.h"

namespace esp {
namespace core {
namespace managedContainers {
/**
 * @brief This abstract base class provides the interface of expected
 * functionality for an object to be manageable by @ref
 * esp::core::ManagedContainer class template specializations. Any class that
 * inherits from this class properly can be managed by a @ref
 * esp::core::ManagedContainer specilization.
 */
class AbstractManagedObject {
 public:
  virtual ~AbstractManagedObject() = default;
  /**
   * @brief Get the instancing class of the ManagedObject instance.  Should
   * only be set from implementer's constructor. Used as key in constructor
   * function pointer maps in @ref esp::core::ManagedContainer.
   */
  virtual std::string getClassKey() const = 0;

  /**
   * @brief Set this ManagedObject name/origin.  Some ManagedObject derive their
   * own names based on their state, such as @ref
   * esp::metadata::attributes::AbstractPrimitiveAttributes;  in such cases this
   * should be overridden with NOP.
   * @param handle the handle to set.
   */
  virtual void setHandle(const std::string& handle) = 0;

  /**
   * @brief Retrieve this object's unique handle
   * @return The handle of the object
   */
  virtual std::string getHandle() const = 0;

  /**
   * @brief Set the unique ID referencing ManagedObject
   * @param ID the ID for this object.
   */
  virtual void setID(int ID) = 0;
  /**
   * @brief Retrieve this object's unique ID.
   * @return Unique ID for this object.
   */
  virtual int getID() const = 0;

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  virtual std::string getObjectInfo() const = 0;

  /**
   * @brief Retrieve a comma-separated list of the values the getObjectInfo
   * method returns.
   */
  virtual std::string getObjectInfoHeader() const = 0;

 protected:
  virtual void setClassKey(const std::string&) = 0;

 public:
  ESP_SMART_POINTERS(AbstractManagedObject)
};  // class AbstractManagedObject

}  // namespace managedContainers
}  // namespace core
}  // namespace esp
#endif  // ESP_CORE_ABSTRACTMANAGEDOBJECT_H_
