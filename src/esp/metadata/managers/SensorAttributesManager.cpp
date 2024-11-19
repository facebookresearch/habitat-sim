// Copyright (c) Meta Platforms, Inc. and its affiliates.
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
#include "SensorAttributesManager.h"

#include "AbstractAttributesManager.h"

#include "esp/metadata/attributes/AudioSensorAttributes.h"
#include "esp/metadata/attributes/CameraSensorAttributes.h"
#include "esp/metadata/attributes/CubeMapSensorAttributes.h"

namespace esp {
namespace metadata {
using attributes::AbstractSensorAttributes;
namespace managers {

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
  // Get class name from sensorSpec
  std::string sensorAttrType = "";
  switch (sensorSpec->sensorSubType) {
    case sensor::SensorSubType::Fisheye: {
      sensorAttrType = "FisheyeSensorAttributes";
      break;
    }
    case sensor::SensorSubType::Orthographic:
    case sensor::SensorSubType::Pinhole: {
      sensorAttrType = "CameraSensorAttributes";
      break;
    }
    case sensor::SensorSubType::Equirectangular: {
      sensorAttrType = "EquirectangularSensorAttributes";
      break;
    }
    case sensor::SensorSubType::ImpulseResponse: {
      sensorAttrType = "AudioSensorAttributes";
      break;
    }
    default:
      CORRADE_ASSERT_UNREACHABLE(
          "SensorSubType specified maps to no Sensor class",
          getSensorSubTypeName(sensorSpec->sensorSubType));
      return nullptr;
      // Unreachable
  };

  auto sensorAttrs = this->createObject(sensorAttrType, false);
  // TODO : Rename attributes to appropriate name and register
  std::string newAttrHandle = sensorAttrType;

  return this->postCreateRegister(std::move(sensorAttrs), registerTemplate);
}  // SensorAtttributesManager::createAttributesFromSensorSpec

AbstractSensorAttributes::ptr SensorAtttributesManager::createObject(
    const std::string& sensorClassName,
    bool registerTemplate) {
  auto sensorAttrs = this->createDefaultObject(sensorClassName, false);
  if (nullptr == sensorAttrs) {
    return sensorAttrs;
  }
  ESP_DEBUG(Mn::Debug::Flag::NoSpace)
      << "Sensor attributes (" << sensorClassName << ":"
      << sensorAttrs->getHandle() << ") created"
      << (registerTemplate ? " and registered." : ".");

  return this->postCreateRegister(std::move(sensorAttrs), registerTemplate);
}  // SensorAtttributesManager::createObject

AbstractSensorAttributes::ptr SensorAtttributesManager::buildObjectFromJSONDoc(
    const std::string& filename,
    const io::JsonGenericValue& jsonConfig) {
  // TODO Get sensor class type from filename, use to build specific sensor type
  const std::string sensorAttrClassName = "";
  // Build and initialize appropriate class of Attributes based on class name
  auto sensorAttributes =
      this->initNewObjectInternal(sensorAttrClassName, true);
  // This should never fail
  this->setValsFromJSONDoc(sensorAttributes, jsonConfig);
  return sensorAttributes;
}  // SensorAtttributesManager::buildObjectFromJSONDoc

void SensorAtttributesManager::setValsFromJSONDoc(
    AttribsPtr attribs,
    const io::JsonGenericValue& jsonConfig) {
  // TODO support loading values from JSON docs

  // check for user defined attributes
  // this->parseUserDefinedJsonVals(attribs, jsonConfig);

}  // AssetAttributesManager::setValsFromJSONDoc

}  // namespace managers
}  // namespace metadata
}  // namespace esp
