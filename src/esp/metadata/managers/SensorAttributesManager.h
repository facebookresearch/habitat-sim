// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.

#ifndef ESP_METADATA_MANAGERS_SENSORATTRIBUTEMANAGER_H_
#define ESP_METADATA_MANAGERS_SENSORATTRIBUTEMANAGER_H_

/** @file
 * @brief Class Template @ref esp::metadata::managers::SensorAttributesManager
 * This class manages attributes describing/configuring habitat sensors.
 */

#include "AbstractAttributesManager.h"
#include "esp/metadata/attributes/AbstractSensorAttributes.h"
#include "esp/sensor/Sensor.h"

namespace esp {
namespace metadata {

namespace managers {

class SensorAtttributesManager
    : public AbstractAttributesManager<attributes::AbstractSensorAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  SensorAtttributesManager();

  /**
   * @brief Create an attributes from a SensorSpec.
   *
   * TODO : Once SensorSpecs are removed, this should be removed as well.
   *
   * @param sensorSpec The SensorSpec holding the values to use to create a
   * sensor.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */
  attributes::AbstractSensorAttributes::ptr createAttributesFromSensorSpec(
      const sensor::SensorSpec::ptr& sensorSpec,
      bool registerTemplate = true);

  /**
   * @brief Should only be called internally. Creates an instance of a sensor
   * attributes template described by the passed string.
   *
   * @param sensorClassName A string descriptor of the sensor attributes
   * template to be created.
   * @param registerTemplate whether to add this template to the library.
   * If the user is going to edit this template, this should be false - any
   * subsequent editing will require re-registration. Defaults to true. If
   * specified as true, then this function returns a copy of the registered
   * template.
   * @return a reference to the desired template.
   */

  attributes::AbstractSensorAttributes::ptr createObject(
      const std::string& sensorClassName,
      bool registerTemplate = true) override;

  /**
   * @brief Parse passed JSON Document specifically for @ref
   * esp::metadata::attributes::AbstractSensorAttributes object. It always
   * returns a valid @ref esp::metadata::attributes::AbstractSensorAttributes
   * shared_ptr object.
   *
   * TODO : currently do not support file-based Sensor attributes.
   *
   * @param filename the name of the file describing the sensor attributes
   * @param jsonConfig json document to parse
   * @return a reference to the desired template.
   */
  attributes::AbstractSensorAttributes::ptr buildObjectFromJSONDoc(
      const std::string& filename,
      const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief Method to take an existing attributes and set its values from passed
   * json config file.
   * @param attribs (out) an existing attributes to be modified.
   * @param jsonConfig json document to parse
   */
  void setValsFromJSONDoc(AttribsPtr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

 protected:
  /**
   * @brief Used Internally. Create and configure newly-created attributes with
   * any default values, before any specific values are set.
   *
   * @param handleName handle name to be assigned to attributes
   * @param builtFromConfig Whether this AbstractSensorAttributes is being built
   * from a config file (i.e. handleName is the name of a configuration file) or
   * from some other source.
   * @return Newly created but unregistered AbstractSensorAttributes pointer,
   * with only default values set.
   */
  attributes::AbstractSensorAttributes::ptr initNewObjectInternal(
      const std::string& handleName,
      CORRADE_UNUSED bool builtFromConfig) override;

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * PrimObjTypes::END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   */
  template <typename T>
  attributes::AbstractSensorAttributes::ptr createSensorAttributes() {
    return T::create();
  }  // AssetAttributeManager::createPrimAttributes

  /**
   * @brief This method will perform any necessary updating that is
   * AbstractAttributesManager-specific upon template removal, such as removing
   * a specific template handle from the list of file-based template handles in
   * ObjectAttributesManager.  This should only be called @ref
   * esp::core::managedContainers::ManagedContainerBase.
   *
   * @param templateID the ID of the template to remove
   * @param templateHandle the string key of the attributes desired.
   */
  void deleteObjectInternalFinalize(
      CORRADE_UNUSED int templateID,
      CORRADE_UNUSED const std::string& templateHandle) override {}

  /**
   * @brief Not required for this manager.
   *
   * This method will perform any essential updating to the managed object
   * before registration is performed. If this updating fails, registration will
   * also fail.
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Should register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(
      CORRADE_UNUSED attributes::AbstractSensorAttributes::ptr object,
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

  /**
   * @brief Any AbstractSensorAttributes-attributes-specific resetting that
   * needs to happen on ManagedContainerBase::reset().
   */
  void resetFinalize() override {}

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createSensorAttributes() keyed by esp::sensor::SensorSubType corresponding
   * to sensor classes being instanced,
   */
  typedef std::unordered_map<std::string,
                             attributes::AbstractSensorAttributes::ptr (
                                 SensorAtttributesManager::*)()>
      Map_Of_SensorTypeCtors;

  /**
   * @brief Map of function pointers to instantiate a sennsor attributes
   * object,
   */
  Map_Of_SensorTypeCtors sensorTypeConstructorMap_;

 public:
  ESP_SMART_POINTERS(SensorAtttributesManager)
};  // class SensorAtttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SENSORATTRIBUTEMANAGER_H_