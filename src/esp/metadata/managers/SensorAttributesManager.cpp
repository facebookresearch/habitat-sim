// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "SensorAttributesManager.h"

#include "esp/metadata/attributes/AbstractSensorAttributes.h"
#include "esp/metadata/attributes/AudioSensorAttributes.h"
#include "esp/metadata/attributes/CameraSensorAttributes.h"
#include "esp/metadata/attributes/CubeMapSensorAttributes.h"
#include "esp/metadata/attributes/CustomSensorAttributes.h"

namespace esp {
namespace metadata {
using attributes::AbstractSensorAttributes;
namespace managers {

const std::map<sensor::SensorSubType, const std::string>
    SensorAttributesManager::SenssorAttrsTypeNamesMap = {
        {sensor::SensorSubType::Unspecified, ""},
        {sensor::SensorSubType::Custom, "CustomSensorAttributes"},
        {sensor::SensorSubType::Pinhole, "CameraSensorAttributes"},
        {sensor::SensorSubType::Orthographic, "CameraSensorAttributes"},
        {sensor::SensorSubType::Equirectangular,
         "EquirectangularSensorAttributes"},
        {sensor::SensorSubType::ImpulseResponse, "AudioSensorAttributes"},
        {sensor::SensorSubType::Fisheye, "FisheyeSensorAttributes"},
        {sensor::SensorSubType::EndSensorSubType, ""}};

SensorAttributesManager::SensorAttributesManager()
    : AbstractAttributesManager<AbstractSensorAttributes,
                                ManagedObjectAccess::Copy>::
          AbstractAttributesManager("Sensor Attributes", "sensor_config.json") {
  // create constructor maps for various types of sensors mapping to their
  // attributes
  sensorTypeConstructorMap_["AudioSensorAttributes"] =
      &SensorAttributesManager::createSensorAttributes<
          attributes::AudioSensorAttributes>;
  sensorTypeConstructorMap_["CameraSensorAttributes"] =
      &SensorAttributesManager::createSensorAttributes<
          attributes::CameraSensorAttributes>;
  sensorTypeConstructorMap_["CustomSensorAttributes"] =
      &SensorAttributesManager::createSensorAttributes<
          attributes::CustomSensorAttributes>;
  sensorTypeConstructorMap_["EquirectangularSensorAttributes"] =
      &SensorAttributesManager::createSensorAttributes<
          attributes::EquirectangularSensorAttributes>;
  sensorTypeConstructorMap_["FisheyeSensorAttributes"] =
      &SensorAttributesManager::createSensorAttributes<
          attributes::FisheyeSensorAttributes>;

  // create copy constructor map entries for each type of Sensor Attributes
  this->copyConstructorMap_["AudioSensorAttributes"] =
      &SensorAttributesManager::createObjCopyCtorMapEntry<
          attributes::AudioSensorAttributes>;
  this->copyConstructorMap_["CameraSensorAttributes"] =
      &SensorAttributesManager::createObjCopyCtorMapEntry<
          attributes::CameraSensorAttributes>;
  this->copyConstructorMap_["CustomSensorAttributes"] =
      &SensorAttributesManager::createObjCopyCtorMapEntry<
          attributes::CustomSensorAttributes>;
  this->copyConstructorMap_["EquirectangularSensorAttributes"] =
      &SensorAttributesManager::createObjCopyCtorMapEntry<
          attributes::EquirectangularSensorAttributes>;
  this->copyConstructorMap_["FisheyeSensorAttributes"] =
      &SensorAttributesManager::createObjCopyCtorMapEntry<
          attributes::FisheyeSensorAttributes>;

}  // SensorAttributesManager ctor

AbstractSensorAttributes::ptr
SensorAttributesManager::createAttributesFromSensorSpecInternal(
    const sensor::SensorSpec::ptr& sensorSpec,
    bool registerTemplate) {
  // Get attributes class name from sensorSpec
  const std::string& sensorAttrClassName =
      SensorAttributesManager::SenssorAttrsTypeNamesMap.at(
          sensorSpec->sensorSubType);
  CORRADE_ASSERT(!sensorAttrClassName.empty(),
                 "Unknown SensorAttributes class for SensorSubType enum value :"
                     << static_cast<int32_t>(sensorSpec->sensorSubType),
                 nullptr);
  // Build and initialize appropriate class of Attributes based on class name
  auto sensorAttributes =
      this->initNewObjectInternal(sensorAttrClassName, false);
  // Populate attributes from sensorSpec
  sensorAttributes->populateWithSensorSpec(sensorSpec);
  // TODO : Rename attributes to appropriate name and register
  // Build name using more than just spec uuid?
  std::string newAttrHandle = sensorSpec->uuid;
  // set handle appropriately to be registration key
  sensorAttributes->setHandle(newAttrHandle);

  return this->postCreateRegister(std::move(sensorAttributes),
                                  registerTemplate);
}  // SensorAttributesManager::createAttributesFromSensorSpec

AbstractSensorAttributes::ptr SensorAttributesManager::createObject(
    const std::string& sensorFileName,
    bool registerTemplate) {
  auto sensorAttrs = this->createDefaultObject(sensorFileName, false);
  if (nullptr == sensorAttrs) {
    return sensorAttrs;
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Sensor attributes (" << sensorFileName << ":"
      << sensorAttrs->getHandle() << ") created"
      << (registerTemplate ? " and registered." : ".");

  // TODO : set handle appropriately to be registration key

  return this->postCreateRegister(std::move(sensorAttrs), registerTemplate);
}  // SensorAttributesManager::createObject

AbstractSensorAttributes::ptr SensorAttributesManager::buildObjectFromJSONDoc(
    const std::string& filename,
    const io::JsonGenericValue& jsonConfig) {
  // Get sensor class type from jsonconfig, use this to determine class
  // name to build
  std::string tmpStrVal = "";
  // Look for sensor_subtype string value in json config
  if (!io::readMember<std::string>(jsonConfig, "sensor_subtype", tmpStrVal)) {
    // Value not found in json so this is an error - we don't know what kind of
    // sensor to create
    return nullptr;
  }
  std::string strToLookFor = Cr::Utility::String::lowercase(tmpStrVal);
  if (strToLookFor == "unspecified") {
    // cannot instantiate unspecified sensor; error
    ESP_ERROR(Mn::Debug::Flag::NoSpace)
        << "Unable to determine Sensor Sub Type in JSON Config `" << filename
        << "` so unable to build correct SensorAttributes. "
           "Aborting.";
    return nullptr;
  }
  auto found = attributes::SensorSubTypeNamesMap.find(strToLookFor);
  if (found == attributes::SensorSubTypeNamesMap.end()) {
    // string representation of enum value was not found in mappings; error
    return nullptr;
  }
  // By here, legal sensor subtype is found
  sensor::SensorSubType sensorSubType = found->second;

  const std::string& sensorAttrClassName =
      SensorAttributesManager::SenssorAttrsTypeNamesMap.at(sensorSubType);
  CORRADE_ASSERT(!sensorAttrClassName.empty(),
                 "Unknown SensorAttributes class for SensorSubType enum value :"
                     << static_cast<int32_t>(sensorSubType),
                 nullptr);

  // Build and initialize appropriate class of Attributes based on class name
  auto sensorAttributes =
      this->initNewObjectInternal(sensorAttrClassName, true);
  if (nullptr == sensorAttributes) {
    return sensorAttributes;
  }
  // This should never fail
  this->setValsFromJSONDoc(sensorAttributes, jsonConfig);
  return sensorAttributes;
}  // SensorAttributesManager::buildObjectFromJSONDoc

void SensorAttributesManager::setValsFromJSONDoc(
    AbstractSensorAttributes::ptr attribs,
    const io::JsonGenericValue& jsonConfig) {
  // TODO support loading values from JSON docs for each type of
  // SensorAttributes.
  // position of sensor
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "position", [attribs](const Magnum::Vector3& position) {
        attribs->setPosition(position);
      });

  // orientation
  io::jsonIntoConstSetter<Magnum::Vector3>(
      jsonConfig, "orientation", [attribs](const Magnum::Vector3& orientation) {
        attribs->setOrientation(orientation);
      });

  // noise model
  io::jsonIntoConstSetter<std::string>(
      jsonConfig, "noise_model", [attribs](const std::string& noise_model) {
        attribs->setNoiseModel(noise_model);
      });

  // sensor type
  this->setEnumStringFromJsonDoc(
      jsonConfig, "sensor_type", "SensorTypeNamesMap", false,
      attributes::SensorTypeNamesMap,
      [attribs](const std::string& val) { attribs->setSensorType(val); });

  // sensor subtype
  this->setEnumStringFromJsonDoc(
      jsonConfig, "sensor_subtype", "SensorSubTypeNamesMap", false,
      attributes::SensorSubTypeNamesMap,
      [attribs](const std::string& val) { attribs->setSensorSubType(val); });

  // TODO populate other types of sensor attributes from json

  // check for user defined attributes
  this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // SensorAttributesManager::setValsFromJSONDoc

AbstractSensorAttributes::ptr SensorAttributesManager::initNewObjectInternal(
    const std::string& sensorAttrClassName,
    bool builtFromConfig) {
  // sensorAttrClassName is the class of the sensor attributes to build
  // Instead of building the attributes using whatever name is passed into the
  // create workflow, we use the sensorAttrClassName to determine which
  // constructor to invoke.

  auto sensorTypeCtorIter = sensorTypeConstructorMap_.find(sensorAttrClassName);
  if (sensorTypeCtorIter == sensorTypeConstructorMap_.end()) {
    ESP_ERROR()
        << "No sensor attributes class" << sensorAttrClassName
        << "constructor exists in the sensorTypeConstructorMap, so unable to "
           "create an appropriate new SensorAttributes object.";
    return nullptr;
  }
  // these attributes ignore any default settings.
  auto newAttributes = (*this.*sensorTypeCtorIter->second)();
  // set the attributes source filedirectory, from the attributes name
  this->setFileDirectoryFromHandle(newAttributes);
  if (builtFromConfig) {
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "New " << sensorAttrClassName
        << " object created and inited from json";
  } else {
    // Built from sensorspec
    ESP_VERY_VERBOSE(Mn::Debug::Flag::NoSpace)
        << "New " << sensorAttrClassName
        << " object created and inited from SensorSpec";
  }
  return newAttributes;
}  // SensorAttributesManager::initNewObjectInternal

core::managedContainers::ManagedObjectPreregistration
SensorAttributesManager::preRegisterObjectFinalize(
    AbstractSensorAttributes::ptr sensorAttrs,
    const std::string& sensorAttrsHandle,
    CORRADE_UNUSED bool forceRegistration) {
  // This method will verify syntax and uniqueness of given handle, and once it
  // is verified, it will set the object's handle and use it as the registration
  // key.

  // TODO: Verify attributes' field data. This should be done at Configuration
  // level.
  // Find first unique variant of given objectHandle.
  const std::string newHandle =
      this->getUniqueHandleFromCandidate(sensorAttrsHandle);
  // Set handle to be first unique variant of given objectHandle
  sensorAttrs->setHandle(newHandle);
  // if this succeeds the registration should use the object's handle and not
  // the given
  return core::managedContainers::ManagedObjectPreregistration::
      Success_Use_Object_Handle;
}  // SensorAttributesManager::preRegisterObjectFinalize

}  // namespace managers
}  // namespace metadata
}  // namespace esp
