// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDSENSORBASE_H_
#define ESP_SENSOR_MANAGEDSENSORBASE_H_

#include <Corrade/Containers/StringStl.h>
#include <Corrade/Utility/Macros.h>

#include "esp/core/managedContainers/AbstractManagedObject.h"
#include "esp/sensor/Sensor.h"

namespace Cr = Corrade;
namespace Mn = Magnum;
namespace esp {
namespace sensor {

/**
 * @brief Base class template for wrapper for sensor objects of all kinds to
 * enable Managed Container access.
 */
template <class T>
class AbstractManagedSensor
    : public esp::core::managedContainers::AbstractManagedObject {
 public:
  static_assert(std::is_base_of<esp::sensor::Sensor, T>::value,
                "AbstractManagedSensor :: Managed sensor object type must be "
                "derived from esp::sensor::Sensor");

  typedef std::weak_ptr<T> WeakObjRef;

  explicit AbstractManagedSensor(const std::string& classKey) {
    AbstractManagedSensor::setClassKey(classKey);
  }

  void setObjectRef(const std::shared_ptr<T>& objRef) { weakObjRef_ = objRef; }

  ~AbstractManagedSensor() override = default;

  /**
   * @brief Retrieve a comma-separated string holding the header values for the
   * info returned for this managed object.
   */
  std::string getObjectInfoHeader() const override {
    return "Type," + getSensorObjInfoHeaderInternal();
  }

  /**
   * @brief Retrieve a comma-separated informational string about the contents
   * of this managed object.
   */
  std::string getObjectInfo() const override {
    if (auto sp = this->getObjectReference()) {
      namespace CrUt = Cr::Utility;
      return Cr::Utility::formatString("{},{},", classKey_,
                                       getSensorObjInfoInternal(sp));
    }
    return Cr::Utility::formatString("Unknown classkey {},", classKey_);
  }

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */

  virtual std::string getSensorObjInfoHeaderInternal() const = 0;
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */
  virtual std::string getSensorObjInfoInternal(
      std::shared_ptr<T>& sp) const = 0;

  /**
   * @brief This function accesses the underlying shared pointer of this
   * object's @p weakObjRef_ if it exists; if not, it provides a message.
   * @return Either a shared pointer of this wrapper's object, or nullptr if
   * DNE.
   */
  std::shared_ptr<T> inline getObjectReference() const {
    std::shared_ptr<T> sp = weakObjRef_.lock();
    if (!sp) {
      // TODO: Verify object is removed from manager here?
      ESP_ERROR()
          << "This sensor object no longer exists.  Please delete any variable "
             "references.";
    }
    return sp;
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
  WeakObjRef weakObjRef_{};

  /**
   * @brief Name of instancing class responsible for this managed object
   */
  std::string classKey_;

 public:
  ESP_SMART_POINTERS(AbstractManagedSensor<T>)
};  // class AbstractManagedSensor
}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDSENSORBASE_H_
