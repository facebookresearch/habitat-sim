// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_SENSOR_SENSORWRAPPERBASEMANAGER_H_
#define ESP_SENSOR_SENSORWRAPPERBASEMANAGER_H_

/** @file
 * @brief Class Template @ref esp::sensor::SensorWrapperBaseManager
 */

#include "esp/core/managedContainers/ManagedContainer.h"
#include "esp/sensor/sensorWrappers/ManagedSensorBase.h"

namespace esp {
namespace core {
namespace managedContainers {
enum class ManagedObjectAccess;
class ManagedContainerBase;
}  // namespace managedContainers
}  // namespace core
namespace sensor {

/**
 * @brief Class template defining responsibilities and functionality for
 * managineg sensor wrappers specializing @ref
 * esp::sensor::AbstractManagedSensor template.
 * @tparam T the type of managed sensor object wrapper a particular
 * specialization of this class works with. Must inherit from @ref
 * esp::sensor::AbstractManagedSensor
 */
template <class T>
class SensorWrapperBaseManager
    : public esp::core::managedContainers::ManagedContainer<
          T,
          core::managedContainers::ManagedObjectAccess::Copy> {
 public:
  typedef std::shared_ptr<T> ObjWrapperPtr;
  explicit SensorWrapperBaseManager(const std::string& SensorType)
      : core::managedContainers::ManagedContainer<
            T,
            core::managedContainers::ManagedObjectAccess::Copy>::
            ManagedContainer(SensorType) {}
  ~SensorWrapperBaseManager() override = default;

  /**
   * @brief Creates an empty @ref esp::sensor::AbstractManagedSensor of
   * the type managed by this manager.
   *
   * @param sensorTypeName Class name of the wrapper to create. This will be
   * used as a key in @p managedSensorTypeConstructorMap_.
   * @param registerObject whether to add this managed object to the
   * library or not. If the user is going to edit this managed object, this
   * should be false. Defaults to true. If specified as true, then this function
   * returns a copy of the registered managed object.
   * @return a reference to the desired managed object.
   */
  ObjWrapperPtr createObject(
      const std::string& sensorTypeName,
      CORRADE_UNUSED bool registerObject = false) override;

 protected:
  /**
   * @brief Any sensor-wrapper-specific resetting that needs to happen
   * on reset.
   */
  void resetFinalize() override {}

  /**
   * @brief Build a shared pointer to a AbstractManagedSensor wrapper around an
   * appropriately cast @ref esp::sensor::Sensor.
   * @tparam The type of the underlying wrapped object to create
   */
  template <typename U>
  ObjWrapperPtr createSensorObjectWrapper() {
    return U::create();
  }

  /**
   * @brief Used Internally. Create and configure newly-created managed object
   * with any default values, before any specific values are set.
   *
   * @param sensorTypeName Used to determine what kind of Rigid Object wrapper
   * to make (either kinematic or dynamic-library-specific)
   * @param builtFromConfig Unused for wrapper objects. All wrappers are
   * constructed from scratch.
   * @return Newly created but unregistered ManagedObject pointer, with only
   * default values set.
   */
  ObjWrapperPtr initNewObjectInternal(
      const std::string& sensorTypeName,
      CORRADE_UNUSED bool builtFromConfig) override {
    // construct a new wrapper based on the passed object
    auto mgdSensorTypeCtorMapIter =
        managedSensorTypeConstructorMap_.find(sensorTypeName);
    if (mgdSensorTypeCtorMapIter == managedSensorTypeConstructorMap_.end()) {
      ESP_ERROR(Mn::Debug::Flag::NoSpace)
          << "<" << this->objectType_ << "> Unknown constructor type "
          << sensorTypeName << ", so initNewObject aborted.";
      return nullptr;
    }
    auto newWrapper = (*this.*(mgdSensorTypeCtorMapIter->second))();

    return newWrapper;
  }  // RigidObjectManager::initNewObjectInternal(

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any essential updating to the managed object
   * before registration is performed. If this updating fails, registration will
   * also fail.
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Will register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(CORRADE_UNUSED ObjWrapperPtr object,
                            CORRADE_UNUSED const std::string& objectHandle,
                            CORRADE_UNUSED bool forceRegistration) override {
    // No pre-registration conditioning performed
    return core::managedContainers::ManagedObjectPreregistration::Success;
  }

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any final manager-related handling after
   * successfully registering an object.
   *
   * See @ref esp::attributes::managers::ObjectAttributesManager for an example.
   *
   * @param objectID the ID of the successfully registered managed object
   * @param objectHandle The name of the managed object
   */
  void postRegisterObjectHandling(
      CORRADE_UNUSED int objectID,
      CORRADE_UNUSED const std::string& objectHandle) override {}

  // ====== instance variables =====
  /** @brief Define a map type referencing function pointers to @ref
   * createRigidObjectWrapper() keyed by string names of classes being
   * instanced
   */
  typedef std::map<std::string,
                   ObjWrapperPtr (SensorWrapperBaseManager<T>::*)()>
      Map_Of_ManagedObjTypeCtors;

  /** @brief Map of function pointers to instantiate a @ref
   * esp::sensor::AbstractManagedSensor wrapper object, keyed by the
   * wrapper's class name. A ManagedRigidObject wrapper of the appropriate type
   * is instanced by accessing the constructor map with the appropriate
   * classname.
   */
  Map_Of_ManagedObjTypeCtors managedSensorTypeConstructorMap_;

 public:
  ESP_SMART_POINTERS(SensorWrapperBaseManager<T>)
};  // class SensorWrapperBaseManager

}  // namespace sensor
}  // namespace esp

#endif  // ESP_SENSOR_SENSORWRAPPERBASEMANAGER_H_
