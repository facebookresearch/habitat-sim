// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDSENSORBASE_H_
#define ESP_SENSOR_MANAGEDSENSORBASE_H_

#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Macros.h>

#include "esp/core/managedContainers/AbstractManagedObject.h"
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

namespace Cr = Corrade;
namespace Mn = Magnum;
namespace esp {
namespace sensor {

/**
 * @brief Base class for wrappers of sensor objects of all kinds to enable
 * Managed Container access.
 */
class ManagedSensorBase
    : public esp::core::managedContainers::AbstractManagedObject {
 public:
  explicit ManagedSensorBase(const std::string& classKey) {
    ManagedSensorBase::setClassKey(classKey);
  }
  ~ManagedSensorBase() override = default;
  /**
   * @brief Get the instancing class of the ManagedSensorBase instance. Should
   * only be set from implementer's constructor. Used as key in constructor
   * function pointer maps in @ref
   * esp::core::managedContainers::ManagedContainer.
   */
  std::string getClassKey() const override { return classKey_; }

  /**
   *  @brief Managed Sensor objects manage their own handles, so this is
   * currently unsettable.
   */
  void setHandle(CORRADE_UNUSED const std::string& name) override {}

  /**
   * @brief Retrieve this Managed Sensor object's unique handle.
   */
  std::string getHandle() const override {
    if (auto sp = getObjectReferenceInternal<Sensor>()) {
      return sp->getSensorHandle();
    }
    return "";
  }

  /**
   *  @brief Managed Sensor objects manage their own IDs, so this is
   * unsettable.
   */
  void setID(CORRADE_UNUSED int ID) override {}

  /**
   * @brief Retrieve this object's unique ID.
   */
  int getID() const override {
    if (auto sp = getObjectReferenceInternal<Sensor>()) {
      return sp->getSensorID();
    }
    return ID_UNDEFINED;
  }  // getID()

 protected:
  void setObjectRefInternal(const std::shared_ptr<void>& objRef) {
    weakObjRef_ = objRef;
  }

  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's @p weakObjRef_ if it exists; if not, it provides a message.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * DNE.
   */
  template <class T>
  std::shared_ptr<T> inline getObjectReferenceInternal() const {
    std::shared_ptr<void> sp = weakObjRef_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      ESP_ERROR()
          << "This sensor object no longer exists. Please delete any variable "
             "references.";
    }
    return std::static_pointer_cast<T>(sp);
  }  // getObjectReference

  /**
   * @brief Set this managed object's class. Should only be set from
   * constructor. Used as key in constructor function pointer maps in Managed
   * Container.
   * @param classKey the string handle corresponding to the
   * constructors used to make copies of this object in copy constructor map.
   */
  void setClassKey(const std::string& classKey) override {
    classKey_ = classKey;
  }

  /**
   * @brief Weak ref to object. If user has copy of this wrapper but object
   * has been deleted, this will be nullptr.
   */
  std::weak_ptr<void> weakObjRef_{};

  /**
   * @brief Name of instancing class responsible for this managed object
   */
  std::string classKey_;

 public:
  ESP_SMART_POINTERS(ManagedSensorBase)
};  // class AbstractManagedSensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDSENSORBASE_H_
