// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_MANAGEDSENSORTEMPLATES_H_
#define ESP_SENSOR_MANAGEDSENSORTEMPLATES_H_

#include <Corrade/Utility/FormatStl.h>
#include <Corrade/Utility/Macros.h>

#include "ManagedSensorBase.h"
#include "esp/sensor/Sensor.h"
#include "esp/sensor/VisualSensor.h"

namespace Cr = Corrade;
namespace Mn = Magnum;
namespace esp {
namespace sensor {

/**
 * @brief Base class template for wrapper for sensor objects of all kinds to
 * enable Managed Container access.
 */
template <class T>
class AbstractManagedSensor : public ManagedSensorBase {
 public:
  static_assert(std::is_base_of<esp::sensor::Sensor, T>::value,
                "AbstractManagedSensor :: Managed sensor object type must be "
                "derived from esp::sensor::Sensor");

  typedef std::weak_ptr<T> WeakObjRef;

  explicit AbstractManagedSensor(const std::string& classKey)
      : ManagedSensorBase(classKey) {}

  void setObjectRef(const std::shared_ptr<T>& objRef) {
    setObjectRefInternal(objRef);
  }

  /**
   * @brief Return whether this is a visual sensor or not
   */
  bool isVisualSensor() const {
    if (auto sp = this->getObjectReference()) {
      return sp->isVisualSensor();
    }
    return false;
  }

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
    return getObjectReferenceInternal<T>();
  }  // getObjectReference

 public:
  ESP_SMART_POINTERS(AbstractManagedSensor<T>)
};  // class AbstractManagedSensor

/**
 * @brief Base class template for wrapper for visual sensor objects of all kinds
 * that extends AbstractManagedSensorAccess
 */

template <class T>
class AbstractManagedVisualSensor
    : public esp::sensor::AbstractManagedSensor<T> {
 public:
  static_assert(std::is_base_of<esp::sensor::VisualSensor, T>::value,
                "AbstractManagedVisualSensor :: Managed visual sensor object "
                "type must be derived from esp::sensor::VisualSensor");

  explicit AbstractManagedVisualSensor(const std::string& classKey)
      : AbstractManagedSensor<T>(classKey) {}
  // TODO Add appropriate abstract visual sensor getters/setters here

 protected:
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed object, type-specific.
   */

  std::string getSensorObjInfoHeaderInternal() const override {
    return "" + getVisualSensorObjInfoHeaderInternal();
  }
  /**
   * @brief Retrieve a comma-separated string holding the header values for
   * the info returned for this managed visual sensor, type-specific.
   */

  virtual std::string getVisualSensorObjInfoHeaderInternal() const = 0;
  /**
   * @brief Specialization-specific extension of getObjectInfo, comma
   * separated info ideal for saving to csv
   */
  std::string getSensorObjInfoInternal(
      CORRADE_UNUSED std::shared_ptr<VisualSensor>& sp) const override {
    // TODO provide info stream for sensors
    std::string res = Cr::Utility::formatString(
        "{},{}", "VisualSensor", getVisualSensorObjInfoInternal());

    return res;
  }

  /**
   * @brief Specialization-specific extension of getSensorObjInfoInternal, comma
   * separated info ideal for saving to csv
   */
  virtual std::string getVisualSensorObjInfoInternal(
      std::shared_ptr<T>& sp) const = 0;

 public:
  ESP_SMART_POINTERS(AbstractManagedVisualSensor<T>)
};  // class AbstractManagedVisualSensor

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_MANAGEDSENSORTEMPLATES_H_
