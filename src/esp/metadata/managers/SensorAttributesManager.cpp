// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "SensorAttributesManager.h"

#include "AbstractAttributesManager.h"

#include "esp/metadata/attributes/AudioSensorAttributes.h"
#include "esp/metadata/attributes/CameraSensorAttributes.h"
#include "esp/metadata/attributes/CubeMapSensorAttributes.h"
#include "esp/metadata/attributes/CustomSensorAttributes.h"

namespace esp {
namespace metadata {
using attributes::AbstractSensorAttributes;
namespace managers {

const std::map<sensor::SensorSubType, const std::string>
    SensorAtttributesManager::SenssorAttrsTypeNamesMap = {
        {sensor::SensorSubType::Unspecified, ""},
        {sensor::SensorSubType::Custom, "CustomSensorAttributes"},
        {sensor::SensorSubType::Pinhole, "CameraSensorAttributes"},
        {sensor::SensorSubType::Orthographic, "CameraSensorAttributes"},
        {sensor::SensorSubType::Equirectangular,
         "EquirectangularSensorAttributes"},
        {sensor::SensorSubType::ImpulseResponse, "AudioSensorAttributes"},
        {sensor::SensorSubType::Fisheye, "FisheyeSensorAttributes"},
        {sensor::SensorSubType::EndSensorSubType, ""}};
SensorAtttributesManager::SensorAtttributesManager()
    : AbstractAttributesManager<attributes::AbstractSensorAttributes,
                                ManagedObjectAccess::Copy>::
          AbstractAttributesManager("Sensor", "sensor_config.json") {
  // create constructor maps for various types of sensors mapping to their
  // attributes
  sensorTypeConstructorMap_["AudioSensorAttributes"] =
      &SensorAtttributesManager::createSensorAttributes<
          attributes::AudioSensorAttributes>;
  sensorTypeConstructorMap_["CameraSensorAttributes"] =
      &SensorAtttributesManager::createSensorAttributes<
          attributes::CameraSensorAttributes>;
  sensorTypeConstructorMap_["CustomSensorAttributes"] =
      &SensorAtttributesManager::createSensorAttributes<
          attributes::CustomSensorAttributes>;
  sensorTypeConstructorMap_["EquirectangularSensorAttributes"] =
      &SensorAtttributesManager::createSensorAttributes<
          attributes::EquirectangularSensorAttributes>;
  sensorTypeConstructorMap_["FisheyeSensorAttributes"] =
      &SensorAtttributesManager::createSensorAttributes<
          attributes::FisheyeSensorAttributes>;

  // create copy constructor map entries for each type of Sensor Attributes
  this->copyConstructorMap_["AudioSensorAttributes"] =
      &SensorAtttributesManager::createObjCopyCtorMapEntry<
          attributes::AudioSensorAttributes>;
  this->copyConstructorMap_["CameraSensorAttributes"] =
      &SensorAtttributesManager::createObjCopyCtorMapEntry<
          attributes::CameraSensorAttributes>;
  this->copyConstructorMap_["CustomSensorAttributes"] =
      &SensorAtttributesManager::createObjCopyCtorMapEntry<
          attributes::CustomSensorAttributes>;
  this->copyConstructorMap_["EquirectangularSensorAttributes"] =
      &SensorAtttributesManager::createObjCopyCtorMapEntry<
          attributes::EquirectangularSensorAttributes>;
  this->copyConstructorMap_["FisheyeSensorAttributes"] =
      &SensorAtttributesManager::createObjCopyCtorMapEntry<
          attributes::FisheyeSensorAttributes>;

}  // SensorAtttributesManager ctor

AbstractSensorAttributes::ptr
SensorAtttributesManager::createAttributesFromSensorSpec(
    const sensor::SensorSpec::ptr& sensorSpec,
    bool registerTemplate) {
  // Get attributes class name from sensorSpec
  const std::string sensorAttrClassName =
      SensorAtttributesManager::SenssorAttrsTypeNamesMap.at(
          sensorSpec->sensorSubType);
  CORRADE_ASSERT(!sensorAttrClassName.empty(),
                 "Unknown SensorAttributes class for SensorSubType enum value :"
                     << static_cast<int32_t>(sensorSpec->sensorSubType),
                 nullptr);
  // Build and initialize appropriate class of Attributes based on class name
  auto sensorAttributes =
      this->initNewObjectInternal(sensorAttrClassName, false);
  // Populate attributes from sensorSpec
  // TODO : Rename attributes to appropriate name and register
  std::string newAttrHandle = sensorAttrClassName;
  // TODO : set handle appropriately to be registration key

  return this->postCreateRegister(std::move(sensorAttributes),
                                  registerTemplate);
}  // SensorAtttributesManager::createAttributesFromSensorSpec

AbstractSensorAttributes::ptr SensorAtttributesManager::createObject(
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
}  // SensorAtttributesManager::createObject

AbstractSensorAttributes::ptr SensorAtttributesManager::buildObjectFromJSONDoc(
    const std::string& filename,
    const io::JsonGenericValue& jsonConfig) {
  // TODO Get sensor class type from jsonconfig, use this to determine class
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
    return nullptr;
  }
  auto found = attributes::SensorSubTypeNamesMap.find(strToLookFor);
  if (found == attributes::SensorSubTypeNamesMap.end()) {
    // string representation of enum value was not found in mappings; error
    return nullptr;
  }
  // By here, legal sensor subtype is found
  sensor::SensorSubType sensorSubType = found->second;

  const std::string sensorAttrClassName =
      SensorAtttributesManager::SenssorAttrsTypeNamesMap.at(sensorSubType);
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
}  // SensorAtttributesManager::buildObjectFromJSONDoc

attributes::AbstractSensorAttributes::ptr
SensorAtttributesManager::initNewObjectInternal(
    const std::string& sensorAttrClassName,
    bool builtFromConfig) {
  // sensorAttrClassName is the class of the sensor attributes to build

  auto sensorTypeCtorIter = sensorTypeConstructorMap_.find(sensorAttrClassName);
  if (sensorTypeCtorIter == sensorTypeConstructorMap_.end()) {
    ESP_ERROR()
        << "No sensor attributes class" << sensorAttrClassName
        << "constructor exists in the sensorTypeConstructorMap, so unable to "
           "create an appropriate new SensorAttributes object.";
    return nullptr;
  }
  // these attributes ignore any default setttings.
  auto newAttributes = (*this.*sensorTypeCtorIter->second)();

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
}  // SensorAtttributesManager::initNewObjectInternal

void SensorAtttributesManager::setValsFromJSONDoc(
    AttribsPtr attribs,
    const io::JsonGenericValue& jsonConfig) {
  // TODO support loading values from JSON docs for each type os
  // SensorAttributes.

  // check for user defined attributes
  // this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // SensorAtttributesManager::setValsFromJSONDoc

}  // namespace managers
}  // namespace metadata
}  // namespace esp
