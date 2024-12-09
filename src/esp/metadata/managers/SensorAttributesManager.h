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
#include "esp/metadata/attributes/AudioSensorAttributes.h"
#include "esp/metadata/attributes/CameraSensorAttributes.h"
#include "esp/metadata/attributes/CubeMapSensorAttributes.h"
#include "esp/metadata/attributes/CustomSensorAttributes.h"

namespace esp {
namespace metadata {
namespace managers {

class SensorAttributesManager
    : public AbstractAttributesManager<attributes::AbstractSensorAttributes,
                                       ManagedObjectAccess::Copy> {
 public:
  /**
   * @brief Constant Map holding names of concrete SensorAttributes classes,
   * keyed by SensorSubtype enum values used to denote them.
   */
  static const std::map<sensor::SensorSubType, const std::string>
      SenssorAttrsTypeNamesMap;
  SensorAttributesManager();

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

  template <typename T>
  std::shared_ptr<T> createAttributesFromSensorSpec(
      const sensor::SensorSpec::ptr& sensorSpec,
      bool registerTemplate = true) {
    static_assert(
        std::is_base_of<attributes::AbstractSensorAttributes, T>::value,
        "AbstractSensorAttributes must be the base type of the requested new "
        "SensorSpec-based attributes");

    attributes::AbstractSensorAttributes::ptr attrs =
        this->createAttributesFromSensorSpecInternal(sensorSpec,
                                                     registerTemplate);

    return std::static_pointer_cast<T>(attrs);
  }

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
  void setValsFromJSONDoc(attributes::AbstractSensorAttributes::ptr attribs,
                          const io::JsonGenericValue& jsonConfig) override;

  /**
   * @brief This function will be called to finalize attributes' paths before
   * registration, moving fully qualified paths to the appropriate hidden
   * attribute fields. This can also be called without registration to make sure
   * the paths specified in an attributes are properly configured.
   * @param attributes The attributes to be filtered.
   */
  void finalizeAttrPathsBeforeRegister(
      CORRADE_UNUSED const attributes::AbstractSensorAttributes::ptr&
          attributes) const override {}

 protected:
  /**
   * @brief Internal only. Create an attributes from a SensorSpec.
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
  attributes::AbstractSensorAttributes::ptr
  createAttributesFromSensorSpecInternal(
      const sensor::SensorSpec::ptr& sensorSpec,
      bool registerTemplate);

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
      bool builtFromConfig) override;

  /**
   * @brief Build a shared pointer to the appropriate attributes for passed
   * object type as defined in @ref PrimObjTypes, where each entry except @ref
   * PrimObjTypes::END_PRIM_OBJ_TYPES corresponds to a Magnum Primitive type
   */
  template <typename T>
  attributes::AbstractSensorAttributes::ptr createSensorAttributes() {
    static_assert(
        std::is_base_of<attributes::AbstractSensorAttributes, T>::value,
        "AbstractSensorAttributes must be base class of desired "
        "SensorAttributes class.");
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
   * @brief This method will perform any essential updating to the managed
   * object before registration is performed. If this updating fails,
   * registration will also fail. In the case of SensorAttributes, it will
   * verify the handle syntax and uniqueness.
   * @param object the managed object to be registered
   * @param objectHandle the name to register the managed object with.
   * Expected to be valid.
   * @param forceRegistration Should register object even if conditional
   * registration checks fail.
   * @return Whether the preregistration has succeeded and what handle to use to
   * register the object if it has.
   */
  core::managedContainers::ManagedObjectPreregistration
  preRegisterObjectFinalize(attributes::AbstractSensorAttributes::ptr object,
                            const std::string& objectHandle,
                            CORRADE_UNUSED bool forceRegistration) override;

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
  void resetFinalize() override {
    // build default attributes::AbstractSensorAttributes objects - reset does
    // not remove constructor mappings.
  }

  // ======== Typedefs and Instance Variables ========

  /**
   * @brief Define a map type referencing function pointers to @ref
   * createSensorAttributes() keyed by esp::sensor::SensorSubType corresponding
   * to sensor classes being instanced,
   */
  typedef std::unordered_map<std::string,
                             attributes::AbstractSensorAttributes::ptr (
                                 SensorAttributesManager::*)()>
      Map_Of_SensorTypeCtors;

  /**
   * @brief Map of function pointers to instantiate a sennsor attributes
   * object,
   */
  Map_Of_SensorTypeCtors sensorTypeConstructorMap_;

 public:
  ESP_SMART_POINTERS(SensorAttributesManager)
};  // class SensorAttributesManager

}  // namespace managers
}  // namespace metadata
}  // namespace esp

#endif  // ESP_METADATA_MANAGERS_SENSORATTRIBUTEMANAGER_H_
